
"""한 domain 62 mapping ingress variant의 launch와 rosbag player를 소유한다."""
from contextlib import ExitStack
from dataclasses import dataclass, replace
import os
from pathlib import Path
import subprocess
from time import monotonic

from go2_validation.mapping_input_acceptance_runner import (
    ExternalReplayStatus,
    MappingInputObservation,
    MappingInputResult,
    MappingInputStatus,
    MappingInputVariant,
    assess_mapping_input,
)


@dataclass(frozen=True, slots=True)
class MappingVariantExecution:
    """한 replay의 verdict, raw observation과 process teardown 근거다."""

    result: MappingInputResult
    observation: MappingInputObservation
    player_exit_code: int
    launch_exit_codes: tuple[int, ...]
    residual_nodes: tuple[str, ...]
    residual_processes: tuple[str, ...]
    log_paths: tuple[Path, ...]


@dataclass(frozen=True, slots=True)
class MappingRuntimeError(Exception):
    """Readiness, player lifecycle 또는 bounded teardown이 실패했다."""

    reason_code: str
    detail: str = ""

    def __str__(self) -> str:
        return f"{self.reason_code}: {self.detail}" if self.detail else self.reason_code


def bag_play_command(path: Path) -> tuple[str, ...]:
    """두 canonical raw topic을 1.0x와 단일 `/clock`으로 replay한다."""
    return (
        "ros2",
        "bag",
        "play",
        str(path),
        "--rate",
        "1.0",
        "--clock",
        "100",
        "--wait-for-all-acked",
        "1000",
        "--topics",
        "/utlidar/cloud",
        "/utlidar/robot_odom",
        "--disable-keyboard-controls",
    )


def mapping_launch_commands(variant: MappingInputVariant) -> tuple[tuple[str, ...], ...]:
    """Mapping launch와 external variant의 odometry adapter만 구성한다."""
    execution_mode = (
        "onboard"
        if variant is MappingInputVariant.PROJECT_STATIONARY
        else "external_replay"
    )
    mapping = (
        "ros2",
        "launch",
        "go2_perception",
        "go2_mapping_scan.launch.py",
        "use_sim_time:=true",
        f"execution_mode:={execution_mode}",
    )
    if variant is MappingInputVariant.PROJECT_STATIONARY:
        return (mapping,)
    return (
        mapping,
        (
            "ros2",
            "launch",
            "bringup",
            "go2_odometry_adapter.launch.py",
            "use_sim_time:=true",
            "continuity_profile:=replay_enforce",
        ),
    )


def run_mapping_variant(
    variant: MappingInputVariant,
    bag_path: Path,
    source_checksum: str,
    log_root: Path,
) -> MappingVariantExecution:
    """Subscriber readiness 뒤 player를 시작하고 graph·teardown을 판정한다."""
    import rclpy

    from bringup.preflight_host import scan_residual_processes
    from go2_validation.mapping_input_capture import build_observation
    from go2_validation.mapping_input_observer import MappingInputObserver
    from go2_validation.offline_process import spin_for, spin_until, stop_owned_process

    if not bag_path.is_dir() or bag_path.is_symlink():
        raise MappingRuntimeError("mapping_bag_path_invalid", str(bag_path))
    observer = MappingInputObserver(variant.value)
    log_root.mkdir(parents=True, exist_ok=True)
    launch_processes: list[subprocess.Popen[str]] = []
    launch_logs: list[Path] = []
    player: subprocess.Popen[str] | None = None
    player_exit_code = -1
    with ExitStack() as stack:
        try:
            for index, command in enumerate(mapping_launch_commands(variant)):
                path = log_root / f"{variant.value}-launch-{index}.log"
                launch_logs.append(path)
                output = stack.enter_context(path.open("w", encoding="utf-8"))
                launch_processes.append(
                    subprocess.Popen(
                        command,
                        stdout=output,
                        stderr=subprocess.STDOUT,
                        text=True,
                        start_new_session=True,
                    )
                )
            ready = spin_until(
                observer,
                lambda: observer.ready_for_player(
                    variant is MappingInputVariant.EXTERNAL_DYNAMIC_SHORT
                ),
                15.0,
            )
            if not ready or any(process.poll() is not None for process in launch_processes):
                raise MappingRuntimeError("mapping_subscriber_readiness_failed")
            player_log_path = log_root / f"{variant.value}-player.log"
            launch_logs.append(player_log_path)
            player_output = stack.enter_context(
                player_log_path.open("w", encoding="utf-8")
            )
            player = subprocess.Popen(
                bag_play_command(bag_path),
                stdout=player_output,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            timeout_sec = (
                170.0
                if variant is MappingInputVariant.EXTERNAL_DYNAMIC_SHORT
                else 60.0
            )
            deadline = monotonic() + timeout_sec
            while rclpy.ok() and monotonic() < deadline and player.poll() is None:
                rclpy.spin_once(observer, timeout_sec=0.1)
                observer.observe_graph()
            if player.poll() is None:
                raise MappingRuntimeError("mapping_player_timeout", variant.value)
            player_exit_code = player.wait()
            if player_exit_code != 0:
                raise MappingRuntimeError(
                    "mapping_player_nonzero_exit",
                    str(player_exit_code),
                )
            spin_for(observer, 2.0)
            observer.observe_graph()
        finally:
            if player is not None:
                player_exit_code = stop_owned_process(player)
            launch_exit_codes = tuple(
                stop_owned_process(process) for process in reversed(launch_processes)
            )
    spin_for(observer, 2.0)
    observer.observe_graph()
    residual_nodes = observer.residual_nodes()
    residual_processes = scan_residual_processes(os.getpid())
    observation = build_observation(
        variant,
        observer.capture(),
        source_checksum,
        domain_id=62,
        loopback_only='name="lo"' in os.environ.get("CYCLONEDDS_URI", ""),
        minimum_rate_hz=1.0,
    )
    result = assess_mapping_input(observation, ExternalReplayStatus.PASSED)
    teardown_failures = tuple(
        reason
        for reason, failed in (
            ("residual_nodes", bool(residual_nodes)),
            ("residual_processes", bool(residual_processes)),
            ("launch_nonzero_exit", any(code != 0 for code in launch_exit_codes)),
        )
        if failed
    )
    if teardown_failures:
        result = replace(
            result,
            status=MappingInputStatus.FAILED,
            failed_checks=result.failed_checks + teardown_failures,
        )
    observer.destroy_node()
    return MappingVariantExecution(
        result=result,
        observation=observation,
        player_exit_code=player_exit_code,
        launch_exit_codes=launch_exit_codes,
        residual_nodes=residual_nodes,
        residual_processes=residual_processes,
        log_paths=tuple(launch_logs),
    )
