"""Domain 0 live localization·no-goal Nav2 관찰의 순수 판정 모델이다."""

from dataclasses import dataclass
from enum import Enum
from typing import Final


EXPECTED_LIFECYCLE_STATES: Final = (
    ("amcl", "active"),
    ("behavior_server", "active"),
    ("bt_navigator", "active"),
    ("controller_server", "active"),
    ("map_server", "active"),
    ("planner_server", "active"),
)


class LiveNavigationStatus(str, Enum):
    """최종 고정 전 live observer의 terminal 판정이다."""

    PASSED = "passed"
    FAILED = "failed"


@dataclass(frozen=True, slots=True)
class LiveNavigationObservation:
    """실제 sensor, localization, Nav2와 안전·종료 관찰값이다."""

    scan_count: int
    odom_count: int
    map_count: int
    map_frames: tuple[str, ...]
    map_has_cells: bool
    pose_count: int
    finite_pose_count: int
    global_costmap_count: int
    local_costmap_count: int
    lifecycle_states: tuple[tuple[str, str], ...]
    global_edges: tuple[tuple[str, str], ...]
    global_owner_nodes: tuple[str, ...]
    plan_count: int
    nonempty_goal_status_count: int
    inert_velocity_count: int
    clock_publisher_max: int
    sport_total_publisher_max: int
    lowcmd_total_publisher_max: int
    sport_ros_publisher_max: int
    lowcmd_ros_publisher_max: int
    cmd_vel_publisher_max: int
    control_node_max: int
    unitree_node_max: int
    launch_exit_code: int
    residual_nodes: tuple[str, ...]
    residual_processes: tuple[str, ...]
    teardown_global_owner_nodes: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class LiveNavigationResult:
    """전체 상태와 실패한 machine-readable check ID다."""

    status: LiveNavigationStatus
    failed_checks: tuple[str, ...]


def assess_live_navigation(
    observation: LiveNavigationObservation,
) -> LiveNavigationResult:
    """Live no-goal 계약을 판정한다."""
    checks = (
        (
            "live_inputs",
            observation.scan_count > 0
            and observation.odom_count > 0
            and observation.map_count > 0,
        ),
        (
            "map_payload",
            observation.map_frames == ("map",) and observation.map_has_cells,
        ),
        (
            "finite_localization_pose",
            observation.pose_count > 0
            and observation.finite_pose_count == observation.pose_count,
        ),
        (
            "costmaps",
            observation.global_costmap_count > 0
            and observation.local_costmap_count > 0,
        ),
        (
            "lifecycle",
            observation.lifecycle_states == EXPECTED_LIFECYCLE_STATES,
        ),
        ("global_edge", observation.global_edges == (("map", "odom"),)),
        ("global_owner", observation.global_owner_nodes == ("/amcl",)),
        (
            "no_goal_output",
            observation.plan_count == 0
            and observation.nonempty_goal_status_count == 0
            and observation.inert_velocity_count == 0,
        ),
        ("wall_clock", observation.clock_publisher_max == 0),
        (
            "physical_command_boundary",
            observation.sport_ros_publisher_max == 0
            and observation.lowcmd_ros_publisher_max == 0
            and observation.cmd_vel_publisher_max == 0,
        ),
        (
            "physical_node_boundary",
            observation.control_node_max == 0
            and observation.unitree_node_max == 0,
        ),
        ("launch_exit", observation.launch_exit_code == 0),
        ("residual_nodes", not observation.residual_nodes),
        ("residual_processes", not observation.residual_processes),
        (
            "teardown_global_owner",
            not observation.teardown_global_owner_nodes,
        ),
    )
    failed_checks = tuple(check_id for check_id, passed in checks if not passed)
    return LiveNavigationResult(
        status=(
            LiveNavigationStatus.PASSED
            if not failed_checks
            else LiveNavigationStatus.FAILED
        ),
        failed_checks=failed_checks,
    )
