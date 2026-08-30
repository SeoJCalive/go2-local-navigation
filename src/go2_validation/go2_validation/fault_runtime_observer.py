
"""Domain 61 fault attempt의 event·output·TF·command graph를 읽기 전용 관찰한다."""
from typing import Final

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from go2_validation.fault_fixture_model import FixtureEvent, FixturePhase
from go2_validation.fault_runtime_capture import (
    FixtureEventMarker,
    FixtureEventParseError,
    StreamStampObservation,
    correlate_fixture_events,
    parse_fixture_event,
)


NANOSECONDS_PER_SECOND: Final = 1_000_000_000
STREAM_QOS: Final = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    durability=DurabilityPolicy.VOLATILE,
)
EVENT_QOS: Final = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    durability=DurabilityPolicy.VOLATILE,
)
ATTEMPT_NODES: Final = frozenset(
    {
        "/base_to_utlidar_lidar_static_tf",
        "/go2_fault_fixture",
        "/go2_mapping_cloud_gate",
        "/go2_obstacle_candidates",
        "/go2_odometry_adapter",
        "/pointcloud_to_laserscan",
        "/robot_state_publisher",
    }
)


class FaultRuntimeObserver(Node):
    """실제 downstream callback stamp와 최대 위험 graph 상태를 누적한다."""

    def __init__(self, scenario_id: str) -> None:
        super().__init__(f"fault_observer_{scenario_id.replace('-', '_')}")
        self._markers: list[FixtureEventMarker] = []
        self._parse_errors: list[str] = []
        self._validated_cloud: list[int] = []
        self._scan: list[int] = []
        self._odom: list[int] = []
        self._tf: list[int] = []
        self._global_tf_parents: set[str] = set()
        self._command_publisher_max = 0
        self._control_node_seen = False
        self._observer_subscription_handles = (
            self.create_subscription(
                PointCloud2,
                "/go2_mapping/cloud_validated",
                lambda message: self._validated_cloud.append(_stamp(message)),
                STREAM_QOS,
            ),
            self.create_subscription(
                LaserScan,
                "/scan",
                lambda message: self._scan.append(_stamp(message)),
                STREAM_QOS,
            ),
            self.create_subscription(
                Odometry,
                "/odom",
                lambda message: self._odom.append(_stamp(message)),
                STREAM_QOS,
            ),
            self.create_subscription(TFMessage, "/tf", self._on_tf, STREAM_QOS),
            self.create_subscription(String, "/go2_fault/fixture_event", self._on_event, EVENT_QOS),
        )

    def _on_event(self, message: String) -> None:
        try:
            self._markers.append(parse_fixture_event(message.data))
        except FixtureEventParseError as error:
            self._parse_errors.append(str(error))

    def _on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.header.frame_id == "odom" and transform.child_frame_id == "base":
                self._tf.append(_stamp(transform))
            if transform.child_frame_id == "odom":
                self._global_tf_parents.add(transform.header.frame_id)

    def observe_graph(self) -> None:
        """Command publisher와 control-node 존재의 attempt 최대값을 갱신한다."""
        command_count = sum(
            len(self.get_publishers_info_by_topic(topic))
            for topic in ("/api/sport/request", "/lowcmd")
        )
        self._command_publisher_max = max(self._command_publisher_max, command_count)
        node_paths = _node_paths(self)
        self._control_node_seen = self._control_node_seen or "/go2_motion_adapter" in node_paths

    def terminal_seen(self, phase: FixturePhase) -> bool:
        """요청한 fixture terminal marker의 수신 여부를 반환한다."""
        return any(marker.phase is phase for marker in self._markers)

    def observed_events(self) -> tuple[FixtureEvent, ...]:
        """Marker를 실제 output callback과 결합한다."""
        return correlate_fixture_events(
            tuple(self._markers),
            StreamStampObservation(
                tuple(self._validated_cloud),
                tuple(self._scan),
                tuple(self._odom),
                tuple(self._tf),
            ),
        )

    @property
    def parse_errors(self) -> tuple[str, ...]:
        return tuple(self._parse_errors)

    @property
    def global_tf_owner_count(self) -> int:
        return len(self._global_tf_parents)

    @property
    def command_publisher_max(self) -> int:
        return self._command_publisher_max

    @property
    def control_node_seen(self) -> bool:
        return self._control_node_seen

    def residual_nodes(self) -> tuple[str, ...]:
        """Teardown 뒤 남은 attempt node path만 반환한다."""
        return tuple(sorted(_node_paths(self).intersection(ATTEMPT_NODES)))


def _stamp(message) -> int:
    return (
        message.header.stamp.sec * NANOSECONDS_PER_SECOND
        + message.header.stamp.nanosec
    )


def _node_paths(node: Node) -> set[str]:
    return {
        f"/{name}" if namespace == "/" else f"{namespace.rstrip('/')}/{name}"
        for name, namespace in node.get_node_names_and_namespaces()
    }
