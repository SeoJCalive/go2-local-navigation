"""Domain 65 여섯 시나리오를 순차 실행하고 JSON 결과를 기록한다."""

import logging
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Final

from bringup.preflight_result import JsonDocument, JsonValue, write_document

from go2_validation.shadow_environment import (
    assess_shadow_environment,
    current_shadow_environment,
)
from go2_validation.shadow_runtime_execution import (
    ShadowRunResult,
    ShadowRuntimeError,
    run_shadow_scenario,
)
from go2_validation.shadow_scenarios import (
    ShadowScenario,
    ShadowScenarioError,
    load_shadow_scenarios,
)
from go2_validation.shadow_verdict import ShadowObservation, ShadowStatus

DOMAIN_ID: Final = 65


@dataclass(frozen=True, slots=True)
class ShadowSummaryError(Exception):
    """Scenario manifest와 실행 결과의 cardinality·순서 불일치다."""

    reason_code: str

    def __str__(self) -> str:
        return self.reason_code


@dataclass(frozen=True, slots=True)
class ShadowRunnerConfiguration:
    """ROS parameter 경계에서 파싱한 Domain 65 실행 경로다."""

    scenario_path: Path
    nav2_root: Path
    run_directory: Path


def build_shadow_summary(
    scenarios: tuple[ShadowScenario, ...],
    results: tuple[ShadowRunResult, ...],
) -> JsonDocument:
    """Manifest 순서를 보존해 여섯 결과와 전체 판정을 직렬화한다."""
    if len(scenarios) != len(results):
        raise ShadowSummaryError("shadow_summary_length_mismatch")
    if any(
        scenario.scenario_id != result.scenario_id
        for scenario, result in zip(scenarios, results)
    ):
        raise ShadowSummaryError("shadow_summary_order_mismatch")
    rows: list[JsonValue] = [_result_document(result) for result in results]
    overall = (
        ShadowStatus.PASSED.value
        if all(result.verdict.status is ShadowStatus.PASSED for result in results)
        else ShadowStatus.FAILED.value
    )
    return {
        "schema_version": 1,
        "record_kind": "nav2_shadow_acceptance_summary",
        "recorded_at": datetime.now().astimezone().isoformat(),
        "domain_id": DOMAIN_ID,
        "loopback_only": True,
        "physical_execution": False,
        "command_publication": False,
        "overall": overall,
        "scenarios": rows,
    }


def _result_document(result: ShadowRunResult) -> JsonDocument:
    return {
        "scenario_id": result.scenario_id,
        "status": result.verdict.status.value,
        "failed_checks": list(result.verdict.failed_checks),
        "observation": _observation_document(result.observation),
        "log_paths": [str(path) for path in result.log_paths],
    }


def _observation_document(observation: ShadowObservation) -> JsonDocument:
    return {
        "action_terminal": observation.action_terminal.value,
        "path_count": observation.path_count,
        "shadow_candidate_count": observation.shadow_candidate_count,
        "feedback_count": observation.feedback_count,
        "lifecycle_states": [list(item) for item in observation.lifecycle_states],
        "global_costmap_count": observation.global_costmap_count,
        "local_costmap_count": observation.local_costmap_count,
        "clock_publisher_max": observation.clock_publisher_max,
        "clock_progressed": observation.clock_progressed,
        "map_to_odom_owners": list(observation.map_to_odom_owners),
        "odom_to_base_owners": list(observation.odom_to_base_owners),
        "physical_command_publisher_max": (
            observation.physical_command_publisher_max
        ),
        "control_node_max": observation.control_node_max,
        "unitree_node_max": observation.unitree_node_max,
        "fixture_exit_code": observation.fixture_exit_code,
        "launch_exit_code": observation.launch_exit_code,
        "residual_nodes": list(observation.residual_nodes),
        "residual_processes": list(observation.residual_processes),
        "teardown_clock_publishers": observation.teardown_clock_publishers,
        "teardown_tf_owners": list(observation.teardown_tf_owners),
    }


def run_shadow_matrix(
    scenario_path: Path,
    nav2_root: Path,
    run_directory: Path,
) -> tuple[ShadowRunResult, ...]:
    """Manifest의 여섯 시나리오를 순차 실행하고 각 result.json을 보존한다."""
    scenarios = load_shadow_scenarios(scenario_path)
    run_directory.mkdir(parents=True, exist_ok=False)
    results: list[ShadowRunResult] = []
    for scenario in scenarios:
        result = run_shadow_scenario(
            scenario,
            nav2_root / scenario.map_relative_path,
            run_directory / scenario.scenario_id,
        )
        write_document(
            _result_document(result),
            run_directory / scenario.scenario_id / "result.json",
        )
        results.append(result)
    return tuple(results)


def _runner_configuration(args: list[str] | None) -> ShadowRunnerConfiguration:
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from rclpy.node import Node

    rclpy.init(args=args)
    node = Node("go2_nav2_shadow_acceptance_runner")
    try:
        project_root = Path(get_package_prefix("go2_validation")).parents[1]
        scenario_path = Path(
            str(
                node.declare_parameter(
                    "scenario_path",
                    str(
                        project_root
                        / "src/go2_validation/config/shadow_scenarios.yaml"
                    ),
                ).value
            )
        )
        output_root = Path(
            str(
                node.declare_parameter(
                    "output_root",
                    str(project_root / "data/runs/nav2_shadow"),
                ).value
            )
        )
        run_label = str(
            node.declare_parameter(
                "run_label",
                f"stage12-domain65-{datetime.now(timezone.utc):%Y%m%d_%H%M%S}",
            ).value
        )
        return ShadowRunnerConfiguration(
            scenario_path=scenario_path,
            nav2_root=project_root / "src/go2_nav2",
            run_directory=output_root / run_label,
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args: list[str] | None = None) -> None:
    """환경 hard gate 뒤 matrix를 실행하고 process exit로 overall을 전달한다."""
    configuration = _runner_configuration(args)
    output_path = configuration.run_directory / "summary.json"
    exit_code = 2
    try:
        environment_error = assess_shadow_environment(
            current_shadow_environment(use_sim_time=True)
        )
        if environment_error is not None:
            raise ShadowRuntimeError(environment_error)
        scenarios = load_shadow_scenarios(configuration.scenario_path)
        results = run_shadow_matrix(
            configuration.scenario_path,
            configuration.nav2_root,
            configuration.run_directory,
        )
        summary = build_shadow_summary(scenarios, results)
        write_document(summary, output_path)
        exit_code = 0 if summary["overall"] == ShadowStatus.PASSED.value else 2
    except (
        OSError,
        ShadowRuntimeError,
        ShadowScenarioError,
        ShadowSummaryError,
    ) as error:
        if not output_path.exists():
            output_path.parent.mkdir(parents=True, exist_ok=True)
            write_document(
                {
                    "schema_version": 1,
                    "record_kind": "nav2_shadow_acceptance_summary",
                    "recorded_at": datetime.now().astimezone().isoformat(),
                    "domain_id": DOMAIN_ID,
                    "overall": ShadowStatus.FAILED.value,
                    "reason_code": str(error),
                },
                output_path,
            )
        logging.getLogger(__name__).error("nav2 shadow acceptance failed: %s", error)
    raise SystemExit(exit_code)
