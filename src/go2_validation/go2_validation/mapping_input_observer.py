
"""Domain 62 replay에서 scan·odom·clock·global TF·command를 관찰한다."""
from math import isnan
from typing import Final

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from go2_validation.mapping_input_capture import MappingStreamCapture


NANOSECONDS_PER_SECOND: Final = 1_000_000_000
OBSERVATION_QOS: Final = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    durability=DurabilityPolicy.VOLATILE,
)
RUNTIME_NODES: Final = frozenset(
    {
        "/base_to_utlidar_lidar_static_tf",
        "/go2_mapping_cloud_gate",
        "/go2_odometry_adapter",
        "/pointcloud_to_laserscan",
        "/robot_state_publisher",
        "/rosbag2_player",
    }
)


class MappingInputObserver(Node):
    """한 sequential replay variant의 acceptance 통계만 메모리에 누적한다."""

    def __init__(self, variant_id: str) -> None:
        super().__init__(f"mapping_input_observer_{variant_id}")
        self._scan_frames: list[str] = []
        self._scan_stamps: list[int] = []
        self._ranges_valid = True
        self._odom_stamps: list[int] = []
        self._global_tf_parents: set[str] = set()
        self._clock_publisher_max = 0
        self._command_publisher_max = 0
        self._observer_subscription_handles = (
            self.create_subscription(LaserScan, "/scan", self._on_scan, OBSERVATION_QOS),
            self.create_subscription(Odometry, "/odom", self._on_odom, OBSERVATION_QOS),
            self.create_subscription(Clock, "/clock", lambda _message: None, OBSERVATION_QOS),
            self.create_subscription(TFMessage, "/tf", self._on_tf, OBSERVATION_QOS),
        )

    def _on_scan(self, message: LaserScan) -> None:
        self._scan_frames.append(message.header.frame_id)
        self._scan_stamps.append(_stamp(message))
        self._ranges_valid = self._ranges_valid and not any(
            isnan(value) for value in message.ranges
        )

    def _on_odom(self, message: Odometry) -> None:
        self._odom_stamps.append(_stamp(message))

    def _on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.child_frame_id == "odom":
                self._global_tf_parents.add(transform.header.frame_id)

    def observe_graph(self) -> None:
        """Clock와 command publisher의 run 최대값을 갱신한다."""
        self._clock_publisher_max = max(
            self._clock_publisher_max,
            len(self.get_publishers_info_by_topic("/clock")),
        )
        command_count = sum(
            len(self.get_publishers_info_by_topic(topic))
            for topic in ("/api/sport/request", "/lowcmd")
        )
        self._command_publisher_max = max(self._command_publisher_max, command_count)

    def ready_for_player(self, require_odometry: bool) -> bool:
        """Raw subscriber와 scan publisher가 발견된 뒤에만 player를 허용한다."""
        cloud_ready = bool(self.get_subscriptions_info_by_topic("/utlidar/cloud"))
        scan_ready = bool(self.get_publishers_info_by_topic("/scan"))
        odometry_ready = bool(
            self.get_subscriptions_info_by_topic("/utlidar/robot_odom")
        )
        return cloud_ready and scan_ready and (odometry_ready or not require_odometry)

    def capture(self) -> MappingStreamCapture:
        """현재 accumulator를 불변 pure-contract 입력으로 고정한다."""
        return MappingStreamCapture(
            scan_type="sensor_msgs/msg/LaserScan" if self._scan_stamps else "",
            scan_frames=tuple(self._scan_frames),
            scan_stamps_ns=tuple(self._scan_stamps),
            scan_ranges=() if self._ranges_valid else (float("nan"),),
            odom_stamps_ns=tuple(self._odom_stamps),
            clock_publisher_max=self._clock_publisher_max,
            global_tf_owner_count=len(self._global_tf_parents),
            command_publisher_max=self._command_publisher_max,
        )

    def residual_nodes(self) -> tuple[str, ...]:
        """Teardown 뒤 남은 mapping/player node path만 반환한다."""
        return tuple(sorted(_node_paths(self).intersection(RUNTIME_NODES)))


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
