"""Live observer의 TF owner·command endpoint·금지 node 최대값을 집계한다."""

from dataclasses import dataclass
from typing import Final

from go2_validation.mapping_runtime_graph import (
    GraphEndpoint,
    MappingGraphNode,
    global_tf_owner_nodes,
    node_paths,
)

BARE_DDS_NODE_NAME: Final = "_CREATED_BY_BARE_DDS_APP_"


@dataclass(frozen=True, slots=True)
class LiveNavigationGraphSnapshot:
    """한 실행 동안 관찰한 graph 최대값과 누적 global TF owner다."""

    global_owner_nodes: tuple[str, ...]
    clock_publisher_max: int
    sport_total_publisher_max: int
    lowcmd_total_publisher_max: int
    sport_ros_publisher_max: int
    lowcmd_ros_publisher_max: int
    cmd_vel_publisher_max: int
    control_node_max: int
    unitree_node_max: int


class LiveNavigationGraphMonitor:
    """반복 graph sampling에서 최대값을 유지하기 위해 mutable한 집계기다."""

    def __init__(self) -> None:
        self._global_owner_nodes: set[str] = set()
        self._clock_publisher_max = 0
        self._sport_total_publisher_max = 0
        self._lowcmd_total_publisher_max = 0
        self._sport_ros_publisher_max = 0
        self._lowcmd_ros_publisher_max = 0
        self._cmd_vel_publisher_max = 0
        self._control_node_max = 0
        self._unitree_node_max = 0

    @staticmethod
    def _ros_publisher_count(endpoints: tuple[GraphEndpoint, ...]) -> int:
        return sum(endpoint.node_name != BARE_DDS_NODE_NAME for endpoint in endpoints)

    def observe(self, node: MappingGraphNode) -> None:
        """현재 graph를 누적 최대값에 반영한다."""
        sport = tuple(node.get_publishers_info_by_topic("/api/sport/request"))
        lowcmd = tuple(node.get_publishers_info_by_topic("/lowcmd"))
        cmd_vel = tuple(node.get_publishers_info_by_topic("/cmd_vel"))
        self._clock_publisher_max = max(
            self._clock_publisher_max,
            len(node.get_publishers_info_by_topic("/clock")),
        )
        self._sport_total_publisher_max = max(
            self._sport_total_publisher_max,
            len(sport),
        )
        self._lowcmd_total_publisher_max = max(
            self._lowcmd_total_publisher_max,
            len(lowcmd),
        )
        self._sport_ros_publisher_max = max(
            self._sport_ros_publisher_max,
            self._ros_publisher_count(sport),
        )
        self._lowcmd_ros_publisher_max = max(
            self._lowcmd_ros_publisher_max,
            self._ros_publisher_count(lowcmd),
        )
        self._cmd_vel_publisher_max = max(
            self._cmd_vel_publisher_max,
            len(cmd_vel),
        )
        self._global_owner_nodes.update(global_tf_owner_nodes(node))
        paths = node_paths(node)
        self._control_node_max = max(
            self._control_node_max,
            sum(path == "/go2_motion_adapter" for path in paths),
        )
        self._unitree_node_max = max(
            self._unitree_node_max,
            sum("unitree" in path.lower() for path in paths),
        )

    def observed_global_owner_nodes(self) -> tuple[str, ...]:
        """실행 중 한 번이라도 본 global-capable owner를 반환한다."""
        return tuple(sorted(self._global_owner_nodes))

    @staticmethod
    def current_global_owner_nodes(node: MappingGraphNode) -> tuple[str, ...]:
        """현재 graph에 남은 global-capable owner를 반환한다."""
        return tuple(sorted(global_tf_owner_nodes(node)))

    def snapshot(self) -> LiveNavigationGraphSnapshot:
        """누적 graph 상태를 최종 불변 관찰로 만든다."""
        return LiveNavigationGraphSnapshot(
            global_owner_nodes=self.observed_global_owner_nodes(),
            clock_publisher_max=self._clock_publisher_max,
            sport_total_publisher_max=self._sport_total_publisher_max,
            lowcmd_total_publisher_max=self._lowcmd_total_publisher_max,
            sport_ros_publisher_max=self._sport_ros_publisher_max,
            lowcmd_ros_publisher_max=self._lowcmd_ros_publisher_max,
            cmd_vel_publisher_max=self._cmd_vel_publisher_max,
            control_node_max=self._control_node_max,
            unitree_node_max=self._unitree_node_max,
        )
