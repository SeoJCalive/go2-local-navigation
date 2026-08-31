"""Domain 64 localization의 launch·player·관찰·teardown을 소유한다.

stationary raw bag과 그 bag에서 저장한 지도만 짝지어 사용한다. 이 실행은 지도 정확도나
live localization을 판정하지 않고 AMCL runtime과 process safety 경계만 관찰한다.
"""

import os
import subprocess
from contextlib import ExitStack
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path
from time import monotonic

import rclpy
import yaml
from bringup.preflight_host import scan_residual_processes

from go2_validation.external_replay_converter import output_tree_checksum
from go2_validation.localization_acceptance import (
    LocalizationObservation,
    LocalizationResult,
    assess_localization,
)
from go2_validation.localization_runtime_observer import LocalizationRuntimeObserver
from go2_validation.mapping_command_builders import mapping_bag_play_command
from go2_validation.mapping_player_services import MappingPlayerServices
from go2_validation.mapping_runtime_data import read_bag_expectation
from go2_validation.offline_process import spin_for, spin_until, stop_owned_process


@dataclass(frozen=True, slots=True)
class LocalizationExecution:
    """Pure verdict, raw observation, fixture identity와 local log 경로다."""

    result: LocalizationResult
    observation: LocalizationObservation
    map_checksum: str
    bag_checksum: str
    log_paths: tuple[Path, ...]


@dataclass(frozen=True, slots=True)
class LocalizationRuntimeError(Exception):
    """Domain 64 child lifecycle 또는 fixture 경계가 깨진 이유다."""

    reason_code: str
    detail: str = ""

    def __str__(self) -> str:
        return f"{self.reason_code}: {self.detail}" if self.detail else self.reason_code


def localization_launch_command(map_path: Path) -> tuple[str, ...]:
    """저장 지도 localization launch의 shell-free argv를 만든다."""
    return (
        "ros2",
        "launch",
        "go2_nav2",
        "go2_saved_map_localization.launch.py",
        f"map:={map_path}",
        "use_sim_time:=true",
        "execution_mode:=onboard",
        "continuity_profile:=replay_enforce",
        "sensor_tf_profile:=project_default",
        "scan_projection_profile:=raw_single",
    )


def saved_map_checksum(map_path: Path) -> str:
    """Map YAML과 같은 디렉터리의 image bytes를 하나의 identity로 만든다."""
    document = yaml.safe_load(map_path.read_text(encoding="utf-8"))
    image_path = map_path.parent / str(document["image"])
    digest = sha256()
    digest.update(map_path.read_bytes())
    digest.update(image_path.read_bytes())
    return digest.hexdigest()


def run_saved_map_localization(
    map_path: Path,
    bag_path: Path,
    run_directory: Path,
) -> LocalizationExecution:
    """Readiness 뒤 full replay를 수행하고 두 child를 bounded하게 종료한다."""
    if not map_path.is_file() or map_path.is_symlink():
        raise LocalizationRuntimeError("localization_map_path_invalid", str(map_path))
    if not bag_path.is_dir() or bag_path.is_symlink():
        raise LocalizationRuntimeError("localization_bag_path_invalid", str(bag_path))
    try:
        run_directory.mkdir(parents=True, exist_ok=False)
    except FileExistsError as error:
        raise LocalizationRuntimeError(
            "localization_run_directory_exists",
            str(run_directory),
        ) from error
    expectation = read_bag_expectation(bag_path)
    observer = LocalizationRuntimeObserver()
    launch_log_path = run_directory / "launch.log"
    player_log_path = run_directory / "player.log"
    launch_process: subprocess.Popen[str] | None = None
    player_process: subprocess.Popen[str] | None = None
    launch_exit_code = -1
    player_exit_code = -1
    with ExitStack() as stack:
        try:
            launch_output = stack.enter_context(
                launch_log_path.open("w", encoding="utf-8")
            )
            launch_process = subprocess.Popen(
                localization_launch_command(map_path),
                stdout=launch_output,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            ready = spin_until(observer, observer.ready_for_player, 30.0)
            if not ready or launch_process.poll() is not None:
                raise LocalizationRuntimeError("localization_readiness_failed")
            observer.capture_lifecycle_states(10.0)
            player_output = stack.enter_context(
                player_log_path.open("w", encoding="utf-8")
            )
            player_process = subprocess.Popen(
                mapping_bag_play_command(bag_path, 1.0),
                stdout=player_output,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            deadline = MappingPlayerServices(observer).synchronize_start(
                readiness_timeout_seconds=30.0,
                resume_timeout_seconds=30.0,
                playback_timeout_seconds=expectation.playback_timeout_seconds,
            )
            while (
                rclpy.ok()
                and monotonic() < deadline
                and player_process.poll() is None
            ):
                rclpy.spin_once(observer, timeout_sec=0.05)
                observer.observe_graph()
            if player_process.poll() is None:
                raise LocalizationRuntimeError("localization_player_timeout")
            player_exit_code = player_process.wait()
            if player_exit_code != 0:
                raise LocalizationRuntimeError(
                    "localization_player_nonzero_exit",
                    str(player_exit_code),
                )
            spin_for(observer, 3.0)
            observer.observe_graph()
            observer.capture_lifecycle_states(10.0)
        finally:
            if player_process is not None:
                player_exit_code = stop_owned_process(player_process)
            if launch_process is not None:
                launch_exit_code = stop_owned_process(launch_process)
    spin_until(observer, observer.teardown_complete, 30.0)
    observer.observe_graph()
    observation = observer.observation(
        player_exit_code,
        launch_exit_code,
        scan_residual_processes(os.getpid()),
    )
    observer.destroy_node()
    return LocalizationExecution(
        result=assess_localization(observation),
        observation=observation,
        map_checksum=saved_map_checksum(map_path),
        bag_checksum=output_tree_checksum(bag_path),
        log_paths=(launch_log_path, player_log_path),
    )
