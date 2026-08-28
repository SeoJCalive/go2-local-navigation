
"""한 fault fixture launch attempt와 그 child lifecycle만 소유한다."""
from dataclasses import dataclass
import os
from pathlib import Path
import re
import subprocess
from time import monotonic
from typing import Final

from bringup.fault_contract import FaultScenario
from go2_validation.fault_fixture_model import FixtureEvent, FixturePhase


FIXTURE_EXIT_PATTERN: Final = re.compile(r"fault_fixture.*exit code (-?\d+)")


@dataclass(frozen=True, slots=True)
class FaultAttemptCapture:
    """Parent가 소유한 launch 한 번의 output·graph·teardown 결과다."""

    child_exit_code: int
    launch_exit_code: int
    events: tuple[FixtureEvent, ...]
    global_tf_owner_count: int
    residual_nodes: tuple[str, ...]
    residual_processes: tuple[str, ...]
    command_publisher_max: int
    control_node_seen: bool
    log_path: Path


@dataclass(frozen=True, slots=True)
class FaultRuntimeError(Exception):
    """Owned attempt가 terminal marker 또는 clean lifecycle을 만들지 못했다."""

    reason_code: str
    detail: str = ""

    def __str__(self) -> str:
        return f"{self.reason_code}: {self.detail}" if self.detail else self.reason_code


def fault_launch_command(
    scenario: FaultScenario,
    *,
    restart_attempt: bool,
) -> tuple[str, ...]:
    """Configured oracle 한 행을 shell 없는 ros2 launch argv로 만든다."""
    return (
        "ros2",
        "launch",
        "go2_validation",
        "go2_fault_acceptance.launch.py",
        f"scenario_id:={scenario.scenario_id}",
        f"fault_kind:={scenario.fault_kind}",
        f"reason_code:={scenario.reason_code}",
        f"recovery_deadline_ns:={scenario.recovery_deadline_seconds * 1_000_000_000}",
        f"restart_attempt:={'true' if restart_attempt else 'false'}",
        "use_sim_time:=true",
        "execution_mode:=onboard",
        "continuity_profile:=replay_enforce",
    )


def run_fault_attempt(
    scenario: FaultScenario,
    *,
    restart_attempt: bool,
    log_path: Path,
) -> FaultAttemptCapture:
    """Launch terminal을 관찰하고 마지막 callback 뒤 parent-only teardown한다."""
    import rclpy

    from bringup.preflight_host import scan_residual_processes
    from go2_validation.fault_runtime_observer import FaultRuntimeObserver
    from go2_validation.offline_process import spin_for, stop_owned_process

    observer = FaultRuntimeObserver(scenario.scenario_id)
    terminal_phase = _terminal_phase(scenario, restart_attempt)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    terminal_seen = False
    with log_path.open("w", encoding="utf-8") as launch_log:
        process = subprocess.Popen(
            fault_launch_command(scenario, restart_attempt=restart_attempt),
            stdout=launch_log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        deadline = monotonic() + max(15, scenario.recovery_deadline_seconds + 8)
        while rclpy.ok() and monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.1)
            observer.observe_graph()
            if terminal_phase is None:
                terminal_seen = _logged_fixture_exit(log_path) is not None
            else:
                terminal_seen = observer.terminal_seen(terminal_phase)
            if terminal_seen or process.poll() is not None:
                break
        if terminal_seen:
            spin_for(observer, 1.0)
            observer.observe_graph()
        launch_exit_code = stop_owned_process(process)
    spin_for(observer, 2.0)
    observer.observe_graph()
    child_exit_code = _child_exit_code(
        scenario,
        restart_attempt,
        observer.observed_events(),
        log_path,
    )
    capture = FaultAttemptCapture(
        child_exit_code=child_exit_code,
        launch_exit_code=launch_exit_code,
        events=observer.observed_events(),
        global_tf_owner_count=observer.global_tf_owner_count,
        residual_nodes=observer.residual_nodes(),
        residual_processes=scan_residual_processes(os.getpid()),
        command_publisher_max=observer.command_publisher_max,
        control_node_seen=observer.control_node_seen,
        log_path=log_path,
    )
    parse_errors = observer.parse_errors
    observer.destroy_node()
    if parse_errors:
        raise FaultRuntimeError("fixture_event_parse_failure", ",".join(parse_errors))
    if not terminal_seen:
        raise FaultRuntimeError("fixture_terminal_timeout", scenario.scenario_id)
    return capture


def _terminal_phase(
    scenario: FaultScenario,
    restart_attempt: bool,
) -> FixturePhase | None:
    if scenario.fault_kind == "launch_failure":
        return None
    if scenario.fault_kind == "process_exit" and not restart_attempt:
        return FixturePhase.OWNED_CHILD_EXIT
    return FixturePhase.RECOVERED


def _logged_fixture_exit(path: Path) -> int | None:
    if not path.is_file():
        return None
    match = FIXTURE_EXIT_PATTERN.search(path.read_text(encoding="utf-8", errors="replace"))
    return int(match.group(1)) if match is not None else None


def _child_exit_code(
    scenario: FaultScenario,
    restart_attempt: bool,
    events: tuple[FixtureEvent, ...],
    log_path: Path,
) -> int:
    if scenario.fault_kind == "launch_failure":
        return _logged_fixture_exit(log_path) or 0
    if scenario.fault_kind == "process_exit" and not restart_attempt:
        child_codes = tuple(
            event.child_exit_code
            for event in events
            if event.phase is FixturePhase.OWNED_CHILD_EXIT
            and event.child_exit_code is not None
        )
        return child_codes[-1] if child_codes else 0
    return 0
