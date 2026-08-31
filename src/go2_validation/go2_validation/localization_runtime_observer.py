"""Domain 64의 scan·odom·map·AMCL pose·TF·안전 graph를 관찰한다.

원본 payload는 보존하지 않고 count, frame, finite 여부와 endpoint owner만 누적한다.
이 node는 initial pose나 command를 publish하지 않으며 판정은 별도 순수 모듈에 맡긴다.
"""

from math import isfinite
from time import monotonic
from typing import Final

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.client import Client
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from go2_validation.localization_acceptance import LocalizationObservation
from go2_validation.mapping_runtime_graph import (
    MAP_QOS,
    STREAM_QOS,
    endpoint_path,
    global_tf_owner_nodes,
    node_paths,
)

NANOSECONDS_PER_SECOND: Final = 1_000_000_000
GRAPH_SPIN_SECONDS: Final = 0.1
LOCALIZATION_NODES: Final = frozenset(
    {
        "/amcl",
        "/base_to_utlidar_lidar_static_tf",
        "/go2_mapping_cloud_gate",
        "/go2_odometry_adapter",
        "/lifecycle_manager_localization",
        "/map_server",
        "/pointcloud_to_laserscan",
        "/robot_state_publisher",
        "/rosbag2_player",
    }
)


class LocalizationRuntimeObserver(Node):
    """한 localization replay의 mutable counters와 graph maxima를 소유한다."""

    def __init__(self) -> None:
        super().__init__("saved_map_localization_observer")
        self._scan_count = 0
        self._odom_count = 0
        self._map_count = 0
        self._map_frames: set[str] = set()
        self._map_has_cells = False
        self._pose_count = 0
        self._finite_pose_count = 0
        self._clock_first: int | None = None
        self._clock_last: int | None = None
        self._clock_publisher_max = 0
        self._global_edges: set[tuple[str, str]] = set()
        self._global_owner_nodes: set[str] = set()
        self._command_publisher_max = 0
        self._control_node_max = 0
        self._lifecycle_states: tuple[tuple[str, str], ...] = ()
        self._owned_subscriptions = (
            self.create_subscription(LaserScan, "/scan", self._on_scan, STREAM_QOS),
            self.create_subscription(Odometry, "/odom", self._on_odom, STREAM_QOS),
            self.create_subscription(OccupancyGrid, "/map", self._on_map, MAP_QOS),
            self.create_subscription(
                PoseWithCovarianceStamped,
                "/amcl_pose",
                self._on_pose,
                10,
            ),
            self.create_subscription(Clock, "/clock", self._on_clock, STREAM_QOS),
            self.create_subscription(TFMessage, "/tf", self._on_tf, STREAM_QOS),
        )
        self._state_clients: Final = (
            ("amcl", self.create_client(GetState, "/amcl/get_state")),
            (
                "map_server",
                self.create_client(GetState, "/map_server/get_state"),
            ),
        )

    def _on_scan(self, _message: LaserScan) -> None:
        self._scan_count += 1

    def _on_odom(self, _message: Odometry) -> None:
        self._odom_count += 1

    def _on_map(self, message: OccupancyGrid) -> None:
        self._map_count += 1
        self._map_frames.add(message.header.frame_id)
        expected_cells = message.info.width * message.info.height
        self._map_has_cells = self._map_has_cells or (
            expected_cells > 0 and len(message.data) == expected_cells
        )

    def _on_pose(self, message: PoseWithCovarianceStamped) -> None:
        self._pose_count += 1
        pose = message.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            *message.pose.covariance,
        )
        if all(isfinite(value) for value in values):
            self._finite_pose_count += 1

    def _on_clock(self, message: Clock) -> None:
        stamp = (
            message.clock.sec * NANOSECONDS_PER_SECOND + message.clock.nanosec
        )
        if self._clock_first is None:
            self._clock_first = stamp
        self._clock_last = stamp

    def _on_tf(self, message: TFMessage) -> None:
        self._global_edges.update(
            (transform.header.frame_id, transform.child_frame_id)
            for transform in message.transforms
            if transform.child_frame_id == "odom"
        )

    def observe_graph(self) -> None:
        """Clock·global owner·physical command·control node 최대값을 갱신한다."""
        self._clock_publisher_max = max(
            self._clock_publisher_max,
            len(self.get_publishers_info_by_topic("/clock")),
        )
        command_count = sum(
            len(self.get_publishers_info_by_topic(topic))
            for topic in ("/api/sport/request", "/lowcmd", "/cmd_vel")
        )
        self._command_publisher_max = max(
            self._command_publisher_max,
            command_count,
        )
        nodes = node_paths(self)
        self._control_node_max = max(
            self._control_node_max,
            sum(path == "/go2_motion_adapter" for path in nodes),
        )
        self._global_owner_nodes.update(global_tf_owner_nodes(self))

    def ready_for_player(self) -> bool:
        """Map Server·AMCL·derived input owner가 active graph에 있는지 확인한다."""
        nodes = node_paths(self)
        scan_subscribers = {
            endpoint_path(endpoint.node_name, endpoint.node_namespace)
            for endpoint in self.get_subscriptions_info_by_topic("/scan")
        }
        return all(
            (
                self._map_count > 0,
                "/amcl" in nodes,
                "/map_server" in nodes,
                "/go2_odometry_adapter" in nodes,
                "/pointcloud_to_laserscan" in nodes,
                "/amcl" in scan_subscribers,
                bool(self.get_publishers_info_by_topic("/scan")),
                bool(self.get_publishers_info_by_topic("/odom")),
            )
        )

    def capture_lifecycle_states(self, timeout_seconds: float) -> None:
        """두 lifecycle service의 current state label을 bounded하게 저장한다."""
        self._lifecycle_states = tuple(
            (name, self._state_label(client, timeout_seconds))
            for name, client in self._state_clients
        )

    def _state_label(self, client: Client, timeout_seconds: float) -> str:
        deadline = monotonic() + timeout_seconds
        while rclpy.ok() and monotonic() < deadline:
            if client.service_is_ready():
                break
            rclpy.spin_once(self, timeout_sec=GRAPH_SPIN_SECONDS)
        if not client.service_is_ready():
            return "unavailable"
        future = client.call_async(GetState.Request())
        while rclpy.ok() and monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=GRAPH_SPIN_SECONDS)
        response = future.result() if future.done() else None
        return "unavailable" if response is None else response.current_state.label

    def teardown_complete(self) -> bool:
        """Owned node와 global TF owner가 graph에서 모두 제거됐는지 확인한다."""
        return not self.residual_nodes() and not self.teardown_global_owner_nodes()

    def residual_nodes(self) -> tuple[str, ...]:
        """현재 graph에 남은 Domain 64 owned node를 반환한다."""
        return tuple(sorted(node_paths(self).intersection(LOCALIZATION_NODES)))

    def teardown_global_owner_nodes(self) -> tuple[str, ...]:
        """현재 graph에 남은 global-capable TF endpoint를 반환한다."""
        return tuple(sorted(global_tf_owner_nodes(self)))

    def observation(
        self,
        player_exit_code: int,
        launch_exit_code: int,
        residual_processes: tuple[str, ...],
    ) -> LocalizationObservation:
        """누적 counters와 terminal process 상태를 불변 관찰로 만든다."""
        return LocalizationObservation(
            scan_count=self._scan_count,
            odom_count=self._odom_count,
            map_count=self._map_count,
            map_frames=tuple(sorted(self._map_frames)),
            map_has_cells=self._map_has_cells,
            pose_count=self._pose_count,
            finite_pose_count=self._finite_pose_count,
            lifecycle_states=self._lifecycle_states,
            global_edges=tuple(sorted(self._global_edges)),
            global_owner_nodes=tuple(sorted(self._global_owner_nodes)),
            clock_publisher_max=self._clock_publisher_max,
            clock_progressed=(
                self._clock_first is not None
                and self._clock_last is not None
                and self._clock_last > self._clock_first
            ),
            command_publisher_max=self._command_publisher_max,
            control_node_max=self._control_node_max,
            player_exit_code=player_exit_code,
            launch_exit_code=launch_exit_code,
            residual_nodes=self.residual_nodes(),
            residual_processes=residual_processes,
            teardown_clock_publishers=len(
                self.get_publishers_info_by_topic("/clock")
            ),
            teardown_global_owner_nodes=self.teardown_global_owner_nodes(),
        )
