"""Domain 0 live navigation의 launch·관찰·teardown 경계를 소유한다."""

import os
import subprocess
from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path

import rclpy
from bringup.preflight_host import scan_residual_processes

from go2_validation.live_navigation_acceptance import (
    LiveNavigationObservation,
    LiveNavigationResult,
    assess_live_navigation,
)
from go2_validation.live_navigation_runtime_observer import (
    LiveNavigationRuntimeObserver,
)
from go2_validation.localization_runtime_execution import saved_map_checksum
from go2_validation.offline_process import spin_for, spin_until, stop_owned_process

READINESS_TIMEOUT_SECONDS = 60.0
TEARDOWN_TIMEOUT_SECONDS = 30.0


@dataclass(frozen=True, slots=True)
class LiveNavigationExecution:
    """Live verdict, 원시 관찰, map identity와 launch log다."""

    result: LiveNavigationResult
    observation: LiveNavigationObservation
    map_checksum: str
    log_path: Path


@dataclass(frozen=True, slots=True)
class LiveNavigationRuntimeError(Exception):
    """Live child readiness 또는 bounded 실행이 깨진 이유다."""

    reason_code: str
    detail: str = ""

    def __str__(self) -> str:
        return f"{self.reason_code}: {self.detail}" if self.detail else self.reason_code


def live_navigation_launch_command(map_path: Path) -> tuple[str, ...]:
    """실제 입력을 소비하는 no-goal launch argv를 만든다."""
    return (
        "ros2",
        "launch",
        "go2_nav2",
        "go2_nav2_live_observer.launch.py",
        f"map:={map_path}",
    )


def run_live_navigation(
    map_path: Path,
    run_directory: Path,
    duration_seconds: float,
) -> LiveNavigationExecution:
    """실제 입력 stack을 no-goal로 관찰하고 단일 SIGINT로 종료한다."""
    if not map_path.is_file() or map_path.is_symlink():
        raise LiveNavigationRuntimeError("live_map_path_invalid", str(map_path))
    if duration_seconds < 30.0 or duration_seconds > 600.0:
        raise LiveNavigationRuntimeError(
            "live_duration_out_of_range",
            str(duration_seconds),
        )
    try:
        run_directory.mkdir(parents=True, exist_ok=False)
    except FileExistsError as error:
        raise LiveNavigationRuntimeError(
            "live_run_directory_exists",
            str(run_directory),
        ) from error
    observer = LiveNavigationRuntimeObserver()
    log_path = run_directory / "launch.log"
    launch_process: subprocess.Popen[str] | None = None
    launch_exit_code = -1
    try:
        with ExitStack() as stack:
            launch_output = stack.enter_context(log_path.open("w", encoding="utf-8"))
            launch_process = subprocess.Popen(
                live_navigation_launch_command(map_path),
                stdout=launch_output,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            try:
                spin_until(
                    observer,
                    lambda: observer.ready() or launch_process.poll() is not None,
                    READINESS_TIMEOUT_SECONDS,
                )
                if launch_process.poll() is not None:
                    raise LiveNavigationRuntimeError(
                        "live_launch_early_exit",
                        str(launch_process.returncode),
                    )
                if not observer.ready():
                    raise LiveNavigationRuntimeError("live_readiness_timeout")
                observer.capture_lifecycle_states(10.0)
                spin_for(observer, duration_seconds)
                observer.observe_graph()
                observer.capture_lifecycle_states(10.0)
            finally:
                launch_exit_code = stop_owned_process(launch_process)
        spin_until(observer, observer.teardown_complete, TEARDOWN_TIMEOUT_SECONDS)
        observer.observe_graph()
        observation = observer.observation(
            launch_exit_code,
            scan_residual_processes(os.getpid()),
        )
        return LiveNavigationExecution(
            result=assess_live_navigation(observation),
            observation=observation,
            map_checksum=saved_map_checksum(map_path),
            log_path=log_path,
        )
    finally:
        if launch_process is not None and launch_process.poll() is None:
            stop_owned_process(launch_process)
        observer.destroy_node()
