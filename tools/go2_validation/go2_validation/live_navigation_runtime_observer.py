"""Domain 0 live localization·Nav2 graph를 publish 없이 관찰한다.

원본 sensor payload는 저장하지 않고 stream count, lifecycle, TF owner, goal·velocity와
command endpoint만 누적한다. 이 node는 publisher, action client, service client를
lifecycle 조회 외에는 생성하지 않는다.
"""

from math import isfinite
from time import monotonic
from typing import Final

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.client import Client
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from go2_validation.live_navigation_acceptance import LiveNavigationObservation
from go2_validation.live_navigation_graph import LiveNavigationGraphMonitor
from go2_validation.mapping_runtime_graph import (
    MAP_QOS,
    STREAM_QOS,
    node_paths,
)

LIFECYCLE_NODE_NAMES: Final = (
    "amcl",
    "behavior_server",
    "bt_navigator",
    "controller_server",
    "map_server",
    "planner_server",
)
RUNTIME_NODE_PATHS: Final = frozenset(
    {
        "/amcl",
        "/base_to_utlidar_lidar_static_tf",
        "/behavior_server",
        "/bt_navigator",
        "/controller_server",
        "/go2_mapping_cloud_gate",
        "/go2_odometry_adapter",
        "/lifecycle_manager_localization",
        "/lifecycle_manager_navigation",
        "/map_server",
        "/planner_server",
        "/pointcloud_to_laserscan",
        "/robot_state_publisher",
    }
)


class LiveNavigationRuntimeObserver(Node):
    """한 no-goal live run의 mutable graph·message counters를 소유한다."""

    def __init__(self) -> None:
        super().__init__("go2_live_navigation_observer")
        self._scan_count = 0
        self._odom_count = 0
        self._map_count = 0
        self._map_frames: set[str] = set()
        self._map_has_cells = False
        self._pose_count = 0
        self._finite_pose_count = 0
        self._global_costmap_count = 0
        self._local_costmap_count = 0
        self._plan_count = 0
        self._nonempty_goal_status_count = 0
        self._inert_velocity_count = 0
        self._global_edges: set[tuple[str, str]] = set()
        self._lifecycle_states: tuple[tuple[str, str], ...] = ()
        self._graph = LiveNavigationGraphMonitor()
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
            self.create_subscription(
                OccupancyGrid,
                "/global_costmap/costmap",
                self._on_global_costmap,
                MAP_QOS,
            ),
            self.create_subscription(
                OccupancyGrid,
                "/local_costmap/costmap",
                self._on_local_costmap,
                MAP_QOS,
            ),
            self.create_subscription(Path, "/plan", self._on_plan, STREAM_QOS),
            self.create_subscription(
                GoalStatusArray,
                "/navigate_to_pose/_action/status",
                self._on_goal_status,
                STREAM_QOS,
            ),
            self.create_subscription(
                Twist,
                "/go2_nav2/shadow_cmd_vel",
                self._on_velocity,
                STREAM_QOS,
            ),
            self.create_subscription(Clock, "/clock", self._on_clock, STREAM_QOS),
            self.create_subscription(TFMessage, "/tf", self._on_tf, STREAM_QOS),
        )
        self._state_clients = tuple(
            (name, self.create_client(GetState, f"/{name}/get_state"))
            for name in LIFECYCLE_NODE_NAMES
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

    def _on_global_costmap(self, _message: OccupancyGrid) -> None:
        self._global_costmap_count += 1

    def _on_local_costmap(self, _message: OccupancyGrid) -> None:
        self._local_costmap_count += 1

    def _on_plan(self, message: Path) -> None:
        if message.poses:
            self._plan_count += 1

    def _on_goal_status(self, message: GoalStatusArray) -> None:
        if message.status_list:
            self._nonempty_goal_status_count += 1

    def _on_velocity(self, message: Twist) -> None:
        if message.linear.x != 0.0 or message.angular.z != 0.0:
            self._inert_velocity_count += 1

    def _on_clock(self, _message: Clock) -> None:
        self.observe_graph()

    def _on_tf(self, message: TFMessage) -> None:
        self._global_edges.update(
            (transform.header.frame_id, transform.child_frame_id)
            for transform in message.transforms
            if transform.child_frame_id == "odom"
        )

    def observe_graph(self) -> None:
        """TF owner, clock, command endpoint와 금지 node 최대값을 갱신한다."""
        self._graph.observe(self)

    def ready(self) -> bool:
        """실제 입력, AMCL, 두 costmap과 전체 runtime node 준비 여부다."""
        self.observe_graph()
        return all(
            (
                RUNTIME_NODE_PATHS.issubset(node_paths(self)),
                self._scan_count > 0,
                self._odom_count > 0,
                self._map_count > 0,
                self._finite_pose_count > 0,
                self._global_costmap_count > 0,
                self._local_costmap_count > 0,
                ("map", "odom") in self._global_edges,
                "/amcl" in self._graph.observed_global_owner_nodes(),
            )
        )

    def capture_lifecycle_states(self, timeout_seconds: float) -> None:
        """필수 lifecycle service의 active label을 bounded하게 저장한다."""
        self._lifecycle_states = tuple(
            (name, self._state_label(client, timeout_seconds))
            for name, client in self._state_clients
        )

    def _state_label(self, client: Client, timeout_seconds: float) -> str:
        deadline = monotonic() + timeout_seconds
        while rclpy.ok() and monotonic() < deadline:
            if client.service_is_ready():
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        if not client.service_is_ready():
            return "unavailable"
        future = client.call_async(GetState.Request())
        while rclpy.ok() and monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        response = future.result() if future.done() else None
        return "unavailable" if response is None else response.current_state.label

    def residual_nodes(self) -> tuple[str, ...]:
        """현재 graph에 남은 live launch 소유 node를 반환한다."""
        return tuple(sorted(node_paths(self).intersection(RUNTIME_NODE_PATHS)))

    def teardown_global_owner_nodes(self) -> tuple[str, ...]:
        """종료 뒤 남은 global-capable TF publisher를 반환한다."""
        return self._graph.current_global_owner_nodes(self)

    def teardown_complete(self) -> bool:
        """Launch 소유 node와 global TF publisher가 모두 제거됐는지 확인한다."""
        return not self.residual_nodes() and not self.teardown_global_owner_nodes()

    def observation(
        self,
        launch_exit_code: int,
        residual_processes: tuple[str, ...],
    ) -> LiveNavigationObservation:
        """누적 counters와 terminal 상태를 불변 판정 입력으로 만든다."""
        self.observe_graph()
        graph = self._graph.snapshot()
        return LiveNavigationObservation(
            scan_count=self._scan_count,
            odom_count=self._odom_count,
            map_count=self._map_count,
            map_frames=tuple(sorted(self._map_frames)),
            map_has_cells=self._map_has_cells,
            pose_count=self._pose_count,
            finite_pose_count=self._finite_pose_count,
            global_costmap_count=self._global_costmap_count,
            local_costmap_count=self._local_costmap_count,
            lifecycle_states=self._lifecycle_states,
            global_edges=tuple(sorted(self._global_edges)),
            global_owner_nodes=graph.global_owner_nodes,
            plan_count=self._plan_count,
            nonempty_goal_status_count=self._nonempty_goal_status_count,
            inert_velocity_count=self._inert_velocity_count,
            clock_publisher_max=graph.clock_publisher_max,
            sport_total_publisher_max=graph.sport_total_publisher_max,
            lowcmd_total_publisher_max=graph.lowcmd_total_publisher_max,
            sport_ros_publisher_max=graph.sport_ros_publisher_max,
            lowcmd_ros_publisher_max=graph.lowcmd_ros_publisher_max,
            cmd_vel_publisher_max=graph.cmd_vel_publisher_max,
            control_node_max=graph.control_node_max,
            unitree_node_max=graph.unitree_node_max,
            launch_exit_code=launch_exit_code,
            residual_nodes=self.residual_nodes(),
            residual_processes=residual_processes,
            teardown_global_owner_nodes=self.teardown_global_owner_nodes(),
        )
