"""Domain 64 localization 관찰을 불변 결과로 판정하는 순수 경계다."""

from dataclasses import dataclass
from enum import Enum


class LocalizationStatus(str, Enum):
    """저장 지도 localization의 terminal 판정이다."""

    PASSED = "passed"
    FAILED = "failed"


@dataclass(frozen=True, slots=True)
class LocalizationObservation:
    """한 격리 replay에서 수집한 map·pose·owner·process 관찰이다."""

    scan_count: int
    odom_count: int
    map_count: int
    map_frames: tuple[str, ...]
    map_has_cells: bool
    pose_count: int
    finite_pose_count: int
    lifecycle_states: tuple[tuple[str, str], ...]
    global_edges: tuple[tuple[str, str], ...]
    global_owner_nodes: tuple[str, ...]
    clock_publisher_max: int
    clock_progressed: bool
    command_publisher_max: int
    control_node_max: int
    player_exit_code: int
    launch_exit_code: int
    residual_nodes: tuple[str, ...]
    residual_processes: tuple[str, ...]
    teardown_clock_publishers: int
    teardown_global_owner_nodes: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class LocalizationResult:
    """관찰에서 계산한 상태와 실패 check ID다."""

    status: LocalizationStatus
    failed_checks: tuple[str, ...]


def assess_localization(
    observation: LocalizationObservation,
) -> LocalizationResult:
    """모든 Domain 64 합격 조건을 순수하게 판정한다."""
    lifecycle_states = dict(observation.lifecycle_states)
    checks = (
        ("scan_input", observation.scan_count > 0),
        ("odom_input", observation.odom_count > 0),
        ("map_output", observation.map_count > 0),
        ("map_frame", observation.map_frames == ("map",)),
        ("map_cells", observation.map_has_cells),
        ("pose_output", observation.pose_count > 0),
        (
            "finite_pose",
            observation.finite_pose_count == observation.pose_count,
        ),
        (
            "lifecycle",
            lifecycle_states.get("map_server") == "active"
            and lifecycle_states.get("amcl") == "active",
        ),
        ("global_edge", observation.global_edges == (("map", "odom"),)),
        ("global_owner", observation.global_owner_nodes == ("/amcl",)),
        ("clock_owner", observation.clock_publisher_max == 1),
        ("clock_progress", observation.clock_progressed),
        ("command_boundary", observation.command_publisher_max == 0),
        ("control_boundary", observation.control_node_max == 0),
        ("player_exit", observation.player_exit_code == 0),
        ("launch_exit", observation.launch_exit_code == 0),
        ("residual_nodes", not observation.residual_nodes),
        ("residual_processes", not observation.residual_processes),
        ("teardown_clock", observation.teardown_clock_publishers == 0),
        (
            "teardown_global_owner",
            not observation.teardown_global_owner_nodes,
        ),
    )
    failed_checks = tuple(check_id for check_id, passed in checks if not passed)
    return LocalizationResult(
        status=(
            LocalizationStatus.PASSED
            if not failed_checks
            else LocalizationStatus.FAILED
        ),
        failed_checks=failed_checks,
    )
