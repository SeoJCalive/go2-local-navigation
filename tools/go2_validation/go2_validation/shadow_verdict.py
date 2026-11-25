"""Domain 65 관찰을 six-scenario acceptance verdict로 판정한다."""

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Final

from go2_validation.shadow_scenarios import (
    ShadowTerminalStatus,
    load_shadow_scenarios,
)


class ShadowStatus(str, Enum):
    PASSED = "passed"
    FAILED = "failed"


@dataclass(frozen=True, slots=True)
class ShadowObservation:
    action_terminal: ShadowTerminalStatus
    path_count: int
    shadow_candidate_count: int
    feedback_count: int
    lifecycle_states: tuple[tuple[str, str], ...]
    global_costmap_count: int
    local_costmap_count: int
    clock_publisher_max: int
    clock_progressed: bool
    map_to_odom_owners: tuple[str, ...]
    odom_to_base_owners: tuple[str, ...]
    physical_command_publisher_max: int
    control_node_max: int
    unitree_node_max: int
    fixture_exit_code: int
    launch_exit_code: int
    residual_nodes: tuple[str, ...]
    residual_processes: tuple[str, ...]
    teardown_clock_publishers: int
    teardown_tf_owners: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class ShadowVerdict:
    status: ShadowStatus
    failed_checks: tuple[str, ...]


SCENARIO_PATH: Final = Path(__file__).parents[1] / "config" / "shadow_scenarios.yaml"


def assess_shadow_scenario(
    scenario_id: str,
    observation: ShadowObservation,
) -> ShadowVerdict:
    scenario = next(
        scenario
        for scenario in load_shadow_scenarios(SCENARIO_PATH)
        if scenario.scenario_id == scenario_id
    )
    lifecycle_states = dict(observation.lifecycle_states)
    checks = (
        ("action_terminal", observation.action_terminal is scenario.expected_terminal),
        ("path", (observation.path_count > 0) is scenario.expects_path),
        (
            "shadow_candidate",
            (observation.shadow_candidate_count > 0) is scenario.expects_candidate,
        ),
        (
            "feedback",
            observation.feedback_count > 0 if scenario_id == "cancel" else True,
        ),
        (
            "lifecycle",
            all(
                lifecycle_states.get(node) == "active"
                for node in (
                    "map_server",
                    "planner_server",
                    "controller_server",
                    "bt_navigator",
                    "behavior_server",
                )
            ),
        ),
        ("global_costmap", observation.global_costmap_count > 0),
        ("local_costmap", observation.local_costmap_count > 0),
        ("clock_owner", observation.clock_publisher_max == 1),
        ("clock_progress", observation.clock_progressed),
        (
            "map_to_odom_owner",
            observation.map_to_odom_owners
            == ("/synthetic_navigation_fixture",),
        ),
        (
            "odom_to_base_owner",
            observation.odom_to_base_owners
            == ("/synthetic_navigation_fixture",),
        ),
        ("physical_command_boundary", observation.physical_command_publisher_max == 0),
        ("control_node_boundary", observation.control_node_max == 0),
        ("unitree_boundary", observation.unitree_node_max == 0),
        ("fixture_exit", observation.fixture_exit_code == 0),
        ("launch_exit", observation.launch_exit_code == 0),
        ("residual_nodes", not observation.residual_nodes),
        ("residual_processes", not observation.residual_processes),
        ("teardown_clock", observation.teardown_clock_publishers == 0),
        ("teardown_tf", not observation.teardown_tf_owners),
    )
    failed_checks = tuple(check_id for check_id, passed in checks if not passed)
    return ShadowVerdict(
        status=ShadowStatus.PASSED if not failed_checks else ShadowStatus.FAILED,
        failed_checks=failed_checks,
    )
