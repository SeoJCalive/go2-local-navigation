"""
AGX 자원을 관찰하고 runner가 만든 POSIX session만 종료·감사한다.

호출자는 start_new_session=True로 만든 Popen PID를 session ID로 전달한다.
launch parent에 SIGINT를 한 번 보내고, 종료 뒤 남은 같은 session의 child에만
TERM/KILL을 적용한다. 다른 ROS 실행의 이름이나 command marker로 소유권을 추정하지 않는다.
"""

import os
import re
import signal
import subprocess
from dataclasses import dataclass
from pathlib import Path
from time import monotonic, sleep
from typing import Final

from bringup.preflight_resources import KernelObservation

KERNEL_FAILURE_PATTERN: Final = re.compile(
    r"out of memory|oom-kill|thermal.*thrott|thrott.*thermal",
    re.IGNORECASE,
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


def scan_residual_processes(
    session_ids: tuple[int, ...], proc_root: Path = Path("/proc"),
) -> tuple[str, ...]:
    """소유한 session에 남은 실행 중 process만 반환한다; zombie는 제외한다."""
    residual: list[str] = []
    for cmdline_path in proc_root.glob("[0-9]*/cmdline"):
        pid = int(cmdline_path.parent.name)
        try:
            fields = cmdline_path.with_name("stat").read_text().rsplit(")", 1)[1].split()
            if int(fields[3]) not in session_ids or fields[0] == "Z":
                continue
            command = cmdline_path.read_bytes().replace(b"\x00", b" ").decode(
                "utf-8", errors="replace"
            )
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        residual.append(f"{pid}:{command.strip()}")
    return tuple(sorted(residual))


def stop_owned_process(process: subprocess.Popen[str]) -> int:
    """Parent-only SIGINT와 bounded group escalation으로 orphan child까지 정리한다."""
    if process.poll() is None:
        try:
            process.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            pass
    for action, timeout in ((signal.SIGTERM, 10), (signal.SIGKILL, 5)):
        if not scan_residual_processes((process.pid,)):
            break
        try:
            os.killpg(process.pid, action)
        except ProcessLookupError:
            break
        deadline = monotonic() + timeout
        while scan_residual_processes((process.pid,)) and monotonic() < deadline:
            sleep(0.05)
        process.poll()
    return process.wait()
