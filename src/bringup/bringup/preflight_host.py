"""AGX host의 thermal trip·kernel event·잔류 process를 관찰한다."""

import os
import re
import signal
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Final

from bringup.preflight_resources import KernelObservation

KERNEL_FAILURE_PATTERN: Final = re.compile(
    r"out of memory|oom-kill|thermal.*thrott|thrott.*thermal",
    re.IGNORECASE,
)
PROCESS_MARKERS: Final = (
    "async_slam_toolbox_node",
    "base_to_utlidar_lidar_static_tf",
    "behavior_server",
    "bt_navigator",
    "controller_server",
    "go2_integrated_preflight_observer",
    "go2_fault_fixture",
    "go2_mapping_cloud_gate",
    "go2_motion_adapter",
    "go2_obstacle_candidates",
    "go2_odometry_adapter",
    "lifecycle_manager_navigation",
    "lifecycle_manager_controller",
    "map_server",
    "planner_server",
    "robot_state_publisher",
    "pointcloud_to_laserscan_node",
    "static_transform_publisher",
    "synthetic_navigation_fixture",
)


@dataclass(frozen=True, slots=True)
class TeardownObservation:
    """launch 종료 뒤 graph·process·프로젝트 command publisher 관찰값이다."""

    launch_return_code: int
    launch_timed_out: bool
    residual_nodes: tuple[str, ...]
    residual_processes: tuple[str, ...]
    sport_request_publishers: int
    lowcmd_publishers: int


def read_passive_trip_c() -> float | None:
    """현재 kernel thermal zone에서 가장 낮은 passive trip을 읽는다."""
    temperatures: list[float] = []
    for type_path in Path("/sys/class/thermal").glob(
        "thermal_zone*/trip_point_*_type"
    ):
        try:
            trip_type = type_path.read_text(encoding="utf-8").strip()
            if trip_type != "passive":
                continue
            temp_path = type_path.with_name(
                type_path.name.replace("_type", "_temp")
            )
            temperatures.append(
                float(temp_path.read_text(encoding="utf-8").strip()) / 1000.0
            )
        except (FileNotFoundError, OSError, ValueError):
            continue
    return min(temperatures) if temperatures else None


def collect_kernel_observation(start_epoch_seconds: float) -> KernelObservation:
    """실행 시작 이후 kernel log에서 OOM·thermal throttle만 추출한다."""
    completed = subprocess.run(
        [
            "journalctl",
            "-k",
            "--since",
            f"@{int(start_epoch_seconds)}",
            "--no-pager",
        ],
        capture_output=True,
        text=True,
        timeout=20,
        check=False,
    )
    if completed.returncode != 0:
        return KernelObservation(available=False, forced_failure_events=())
    events = tuple(
        line.strip()
        for line in completed.stdout.splitlines()
        if KERNEL_FAILURE_PATTERN.search(line)
    )
    return KernelObservation(available=True, forced_failure_events=events)


def scan_residual_processes(excluded_pid: int) -> tuple[str, ...]:
    """현재 runner를 제외하고 project process marker가 남았는지 확인한다."""
    residual: list[str] = []
    for cmdline_path in Path("/proc").glob("[0-9]*/cmdline"):
        pid = int(cmdline_path.parent.name)
        if pid == excluded_pid:
            continue
        try:
            command = cmdline_path.read_bytes().replace(b"\x00", b" ").decode(
                "utf-8", errors="replace"
            )
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        marker = next(
            (item for item in PROCESS_MARKERS if item in command),
            None,
        )
        if marker is not None:
            residual.append(f"{pid}:{marker}")
    return tuple(sorted(residual))


def stop_owned_process(process: subprocess.Popen[str]) -> int:
    """runner가 시작한 process group을 SIGINT, 필요 시 SIGTERM으로 종료한다."""
    return_code = process.poll()
    if return_code is not None:
        return return_code
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return process.wait()
    try:
        return process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
    try:
        return process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
        return process.wait()
