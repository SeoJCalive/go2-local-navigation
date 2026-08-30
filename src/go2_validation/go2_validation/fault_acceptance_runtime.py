
"""Stage 11 fault matrix를 domain 61에서 순차 실행하고 JSON을 기록한다."""
from dataclasses import dataclass
from datetime import datetime
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rclpy.node import Node

from bringup.fault_contract import (
    FaultConfigurationError,
    FaultScenario,
    load_fault_scenarios,
)
from bringup.fault_result import (
    FaultAcceptanceReport,
    FaultScenarioResult,
    fault_report_document,
)
from bringup.mode_observer import ExecutionMode, ModeEnvironment, assess_mode_environment
from bringup.preflight_result import write_document
from bringup.preflight_types import CheckStatus
from go2_validation.fault_acceptance_runner import (
    AttemptOutcome,
    FaultExpectation,
    evaluate_fault_acceptance,
)
from go2_validation.fault_fixture_model import FaultKind, FaultScenario as FixtureScenario
from go2_validation.fault_runtime_execution import (
    FaultAttemptCapture,
    FaultRuntimeError,
    run_fault_attempt,
)


@dataclass(frozen=True, slots=True)
class ScenarioExecution:
    """한 oracle row의 pure verdict와 runtime safety 최대값이다."""

    result: FaultScenarioResult
    command_publisher_max: int
    control_node_seen: bool


def _execute_scenario(scenario: FaultScenario, log_root: Path) -> ScenarioExecution:
    first = run_fault_attempt(
        scenario,
        restart_attempt=False,
        log_path=log_root / f"{scenario.scenario_id}.log",
    )
    restart = (
        run_fault_attempt(
            scenario,
            restart_attempt=True,
            log_path=log_root / f"{scenario.scenario_id}-restart.log",
        )
        if scenario.fault_kind == "process_exit"
        else None
    )
    outcome = _attempt_outcome(first, restart)
    fixture_scenario = FixtureScenario(
        scenario_id=scenario.scenario_id,
        fault_kind=FaultKind(scenario.fault_kind),
        reason_code=scenario.reason_code,
        recovery_deadline_nanoseconds=scenario.recovery_deadline_seconds
        * 1_000_000_000,
    )
    verdict = evaluate_fault_acceptance(
        FaultExpectation.from_scenario(fixture_scenario),
        outcome,
    )
    return ScenarioExecution(
        result=FaultScenarioResult(
            scenario_id=scenario.scenario_id,
            status="passed" if verdict.passed else "failed",
            reason_code=(
                scenario.reason_code
                if verdict.passed
                else verdict.reason_code or "fault_acceptance_unknown"
            ),
            suppressed_outputs=scenario.suppressed_outputs,
            recovered_outputs=verdict.captured_streams,
            recovery_elapsed_nanoseconds=verdict.recovery_elapsed_nanoseconds,
            child_exit_code=first.child_exit_code,
        ),
        command_publisher_max=max(
            first.command_publisher_max,
            restart.command_publisher_max if restart is not None else 0,
        ),
        control_node_seen=first.control_node_seen
        or (restart.control_node_seen if restart is not None else False),
    )


def _attempt_outcome(
    first: FaultAttemptCapture,
    restart: FaultAttemptCapture | None,
) -> AttemptOutcome:
    captures = (first,) if restart is None else (first, restart)
    return AttemptOutcome(
        first_exit_code=first.child_exit_code,
        restart_exit_code=None if restart is None else restart.child_exit_code,
        events=tuple(event for capture in captures for event in capture.events),
        global_tf_owner_count=max(capture.global_tf_owner_count for capture in captures),
        residual_nodes=tuple(sorted({node for capture in captures for node in capture.residual_nodes})),
        residual_processes=tuple(
            sorted({process for capture in captures for process in capture.residual_processes})
        ),
        sport_request_publishers=max(capture.command_publisher_max for capture in captures),
        lowcmd_publishers=0,
        output_enabled=any(capture.control_node_seen for capture in captures),
        physical_validation_approved=False,
    )


def execute_fault_matrix(scenario_path: Path, output_path: Path) -> FaultAcceptanceReport:
    """모든 configured scenario를 순차 실행해 하나의 terminal report로 만든다."""
    scenarios = load_fault_scenarios(scenario_path)
    log_root = output_path.with_suffix("")
    log_root.mkdir(parents=True, exist_ok=True)
    executions = tuple(_execute_scenario(scenario, log_root) for scenario in scenarios)
    command_max = max((row.command_publisher_max for row in executions), default=0)
    gates_closed = not any(row.control_node_seen for row in executions)
    passed = all(row.result.status == "passed" for row in executions)
    return FaultAcceptanceReport(
        overall="passed" if passed and command_max == 0 and gates_closed else "failed",
        domain_id=61,
        command_publisher_count=command_max,
        motion_gates_closed=gates_closed,
        scenarios=tuple(row.result for row in executions),
    )


def _environment_is_valid() -> bool:
    check = assess_mode_environment(
        ExecutionMode.FAULT_RECOVERY,
        ModeEnvironment(
            rmw_implementation=os.environ.get("RMW_IMPLEMENTATION", ""),
            ros_domain_id=os.environ.get("ROS_DOMAIN_ID", ""),
            go2_interface=os.environ.get("GO2_AGX_INTERFACE", ""),
            cyclonedds_uri=os.environ.get("CYCLONEDDS_URI", ""),
        ),
    )
    return check.status is CheckStatus.PASS and 'name="lo"' in os.environ.get(
        "CYCLONEDDS_URI", ""
    )


def main(args: list[str] | None = None) -> None:
    """ROS parameter를 읽고 domain 61 matrix와 atomic result write를 실행한다."""
    rclpy.init(args=args)
    node = Node("go2_fault_acceptance_runner")
    project_root = Path(get_package_prefix("go2_validation")).parents[1]
    scenario_default = (
        Path(get_package_share_directory("bringup")) / "config/fault_scenarios.yaml"
    )
    scenario_path = Path(
        str(node.declare_parameter("scenario_manifest", str(scenario_default)).value)
    )
    output_path = Path(
        str(
            node.declare_parameter(
                "output_path",
                str(project_root / "data/runs/fault_acceptance/stage11.json"),
            ).value
        )
    )
    exit_code = 2
    try:
        if not _environment_is_valid():
            raise FaultRuntimeError("fault_environment_mismatch")
        report = execute_fault_matrix(scenario_path, output_path)
        document = dict(fault_report_document(report))
        document["recorded_at"] = datetime.now().astimezone().isoformat()
        document["loopback_only"] = True
        output_path.parent.mkdir(parents=True, exist_ok=True)
        write_document(document, output_path)
        exit_code = 0 if report.overall == "passed" else 2
    except (FaultConfigurationError, FaultRuntimeError, OSError, ValueError) as error:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        write_document(
            {
                "schema_version": 1,
                "record_kind": "software_fault_acceptance_result",
                "recorded_at": datetime.now().astimezone().isoformat(),
                "overall": "failed",
                "domain_id": 61,
                "reason_code": str(error),
            },
            output_path,
        )
        node.get_logger().error(f"fault acceptance failed: {error}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
