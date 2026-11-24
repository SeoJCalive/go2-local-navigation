"""Domain 65 observer와 executor가 공유하는 불변 runtime 관찰 모델이다."""

from dataclasses import dataclass
from typing import Final

from go2_validation.shadow_scenarios import ShadowTerminalStatus

FIXTURE_NODE_PATH: Final = "/synthetic_navigation_fixture"
RUNTIME_NODES: Final = frozenset(
    {
        FIXTURE_NODE_PATH,
        "/map_server",
        "/planner_server",
        "/controller_server",
        "/bt_navigator",
        "/behavior_server",
        "/lifecycle_manager_navigation",
    }
)


@dataclass(frozen=True, slots=True)
class ShadowRuntimeSurface:
    """Action 실행 중 누적한 출력·lifecycle·owner·안전 graph 관찰이다."""

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


@dataclass(frozen=True, slots=True)
class ShadowTerminalEvidence:
    """Action terminal 뒤 child exit와 teardown 관찰을 묶는다."""

    action_terminal: ShadowTerminalStatus
    fixture_exit_code: int
    launch_exit_code: int
    residual_nodes: tuple[str, ...]
    residual_processes: tuple[str, ...]
