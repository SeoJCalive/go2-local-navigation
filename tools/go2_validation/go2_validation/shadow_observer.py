"""Domain 65 lifecycle·costmap·action·TF·safety graph를 읽기 전용으로 관찰한다.

원본 메시지는 보존하지 않고 시나리오 판정에 필요한 count, owner와 graph 최대값만
누적한다. 이 node는 goal 또는 command를 publish하지 않는다.
"""

from time import monotonic
from typing import Final

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.client import Client
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

from go2_validation.mapping_runtime_graph import (
    MAP_QOS,
    STREAM_QOS,
    endpoint_path,
    node_paths,
)
from go2_validation.shadow_runtime_model import (
    RUNTIME_NODES,
    ShadowRuntimeSurface,
    ShadowTerminalEvidence,
)
from go2_validation.shadow_verdict import ShadowObservation

CLOCK_TOPIC: Final = "/clock"
PLAN_TOPIC: Final = "/plan"
SHADOW_VELOCITY_TOPIC: Final = "/go2_nav2/shadow_cmd_vel"
GLOBAL_COSTMAP_TOPIC: Final = "/global_costmap/costmap"
LOCAL_COSTMAP_TOPIC: Final = "/local_costmap/costmap"
PHYSICAL_COMMAND_TOPICS: Final = ("/api/sport/request", "/lowcmd", "/cmd_vel")
NANOSECONDS_PER_SECOND: Final = 1_000_000_000


class ShadowRuntimeObserver(Node):
    """한 synthetic navigation 시나리오의 mutable counter를 소유한다."""

    def __init__(self) -> None:
        super().__init__("go2_shadow_observer")
        self._path_count = 0
        self._shadow_candidate_count = 0
        self._feedback_count = 0
        self._global_costmap_count = 0
        self._local_costmap_count = 0
        self._clock_first: int | None = None
        self._clock_last: int | None = None
        self._clock_publisher_max = 0
        self._map_to_odom_owners: set[str] = set()
        self._odom_to_base_owners: set[str] = set()
        self._physical_command_publisher_max = 0
        self._control_node_max = 0
        self._unitree_node_max = 0
        self._lifecycle_states: tuple[tuple[str, str], ...] = ()
        self._owned_subscriptions = (
            self.create_subscription(Path, PLAN_TOPIC, self._on_path, STREAM_QOS),
            self.create_subscription(
                Twist,
                SHADOW_VELOCITY_TOPIC,
                self._on_velocity,
                STREAM_QOS,
            ),
            self.create_subscription(
                OccupancyGrid,
                GLOBAL_COSTMAP_TOPIC,
                self._on_global_costmap,
                MAP_QOS,
            ),
            self.create_subscription(
                OccupancyGrid,
                LOCAL_COSTMAP_TOPIC,
                self._on_local_costmap,
                MAP_QOS,
            ),
            self.create_subscription(Clock, CLOCK_TOPIC, self._on_clock, STREAM_QOS),
            self.create_subscription(TFMessage, "/tf", self._on_tf, STREAM_QOS),
        )
        self._state_clients = tuple(
            (name, self.create_client(GetState, f"/{name}/get_state"))
            for name in (
                "map_server",
                "planner_server",
                "controller_server",
                "bt_navigator",
                "behavior_server",
            )
        )

    def _on_path(self, message: Path) -> None:
        if message.poses:
            self._path_count += 1

    def _on_velocity(self, message: Twist) -> None:
        if message.linear.x != 0.0 or message.angular.z != 0.0:
            self._shadow_candidate_count += 1

    def _on_global_costmap(self, _message: OccupancyGrid) -> None:
        self._global_costmap_count += 1

    def _on_local_costmap(self, _message: OccupancyGrid) -> None:
        self._local_costmap_count += 1

    def record_feedback(self, _feedback: NavigateToPose_FeedbackMessage) -> None:
        """NavigateToPose feedback callback 횟수를 누적한다."""
        self._feedback_count += 1

    def cancel_is_ready(self) -> bool:
        """Cancel 전 feedback·path·내부 velocity가 모두 관찰됐는지 반환한다."""
        return (
            self._feedback_count > 0
            and self._path_count > 0
            and self._shadow_candidate_count > 0
        )

    def _on_clock(self, message: Clock) -> None:
        stamp = message.clock.sec * NANOSECONDS_PER_SECOND + message.clock.nanosec
        if self._clock_first is None:
            self._clock_first = stamp
        self._clock_last = stamp
        self.observe_graph()

    def _on_tf(self, message: TFMessage) -> None:
        owners = {
            endpoint_path(endpoint.node_name, endpoint.node_namespace)
            for endpoint in self.get_publishers_info_by_topic("/tf")
        }
        for transform in message.transforms:
            edge = (transform.header.frame_id, transform.child_frame_id)
            if edge == ("map", "odom"):
                self._map_to_odom_owners.update(owners)
            if edge == ("odom", "base"):
                self._odom_to_base_owners.update(owners)

    def observe_graph(self) -> None:
        """Clock·physical command·control·Unitree node 최대값을 갱신한다."""
        self._clock_publisher_max = max(
            self._clock_publisher_max,
            len(self.get_publishers_info_by_topic(CLOCK_TOPIC)),
        )
        physical_publishers = sum(
            len(self.get_publishers_info_by_topic(topic))
            for topic in PHYSICAL_COMMAND_TOPICS
        )
        self._physical_command_publisher_max = max(
            self._physical_command_publisher_max,
            physical_publishers,
        )
        paths = node_paths(self)
        self._control_node_max = max(
            self._control_node_max,
            sum(path == "/go2_motion_adapter" for path in paths),
        )
        self._unitree_node_max = max(
            self._unitree_node_max,
            sum("unitree" in path.lower() for path in paths),
        )

    def capture_lifecycle_states(self, timeout_seconds: float) -> None:
        """필수 Nav2 lifecycle state를 bounded service call로 저장한다."""
        self._lifecycle_states = tuple(
            (name, self._state_label(client, timeout_seconds))
            for name, client in self._state_clients
        )

    def _state_label(self, client: Client, timeout_seconds: float) -> str:
        deadline = monotonic() + timeout_seconds
        last_label = "unavailable"
        while rclpy.ok() and monotonic() < deadline:
            if not client.service_is_ready():
                rclpy.spin_once(self, timeout_sec=0.05)
                continue
            future = client.call_async(GetState.Request())
            while rclpy.ok() and monotonic() < deadline and not future.done():
                rclpy.spin_once(self, timeout_sec=0.05)
            response = future.result() if future.done() else None
            if response is not None:
                last_label = response.current_state.label
                if last_label == "active":
                    return last_label
            rclpy.spin_once(self, timeout_sec=0.05)
        return last_label

    def ready_for_action(self) -> bool:
        """Action 전 필수 Nav2 server와 fixture node가 모두 발견됐는지 확인한다."""
        return (
            RUNTIME_NODES.issubset(node_paths(self))
            and self._global_costmap_count > 0
            and self._local_costmap_count > 0
            and self._clock_first is not None
            and self._clock_last is not None
            and self._clock_last > self._clock_first
            and bool(self._map_to_odom_owners)
            and bool(self._odom_to_base_owners)
        )

    def residual_nodes(self) -> tuple[str, ...]:
        """현재 graph에 남은 Domain 65 owned node를 반환한다."""
        return tuple(sorted(node_paths(self).intersection(RUNTIME_NODES)))

    def teardown_complete(self) -> bool:
        """Owned node와 fixture clock·TF publisher가 모두 사라졌는지 확인한다."""
        return (
            not self.residual_nodes()
            and not self.get_publishers_info_by_topic("/tf")
            and not self.get_publishers_info_by_topic(CLOCK_TOPIC)
        )

    def surface(self) -> ShadowRuntimeSurface:
        """현재까지 누적한 runtime 관찰을 불변 값으로 만든다."""
        self.observe_graph()
        return ShadowRuntimeSurface(
            path_count=self._path_count,
            shadow_candidate_count=self._shadow_candidate_count,
            feedback_count=self._feedback_count,
            lifecycle_states=self._lifecycle_states,
            global_costmap_count=self._global_costmap_count,
            local_costmap_count=self._local_costmap_count,
            clock_publisher_max=self._clock_publisher_max,
            clock_progressed=(
                self._clock_first is not None
                and self._clock_last is not None
                and self._clock_last > self._clock_first
            ),
            map_to_odom_owners=tuple(sorted(self._map_to_odom_owners)),
            odom_to_base_owners=tuple(sorted(self._odom_to_base_owners)),
            physical_command_publisher_max=self._physical_command_publisher_max,
            control_node_max=self._control_node_max,
            unitree_node_max=self._unitree_node_max,
        )

    def observation(self, terminal: ShadowTerminalEvidence) -> ShadowObservation:
        """누적 surface와 terminal evidence를 최종 판정 입력으로 결합한다."""
        surface = self.surface()
        return ShadowObservation(
            action_terminal=terminal.action_terminal,
            path_count=surface.path_count,
            shadow_candidate_count=surface.shadow_candidate_count,
            feedback_count=surface.feedback_count,
            lifecycle_states=surface.lifecycle_states,
            global_costmap_count=surface.global_costmap_count,
            local_costmap_count=surface.local_costmap_count,
            clock_publisher_max=surface.clock_publisher_max,
            clock_progressed=surface.clock_progressed,
            map_to_odom_owners=surface.map_to_odom_owners,
            odom_to_base_owners=surface.odom_to_base_owners,
            physical_command_publisher_max=surface.physical_command_publisher_max,
            control_node_max=surface.control_node_max,
            unitree_node_max=surface.unitree_node_max,
            fixture_exit_code=terminal.fixture_exit_code,
            launch_exit_code=terminal.launch_exit_code,
            residual_nodes=terminal.residual_nodes,
            residual_processes=terminal.residual_processes,
            teardown_clock_publishers=len(
                self.get_publishers_info_by_topic(CLOCK_TOPIC)
            ),
            teardown_tf_owners=tuple(
                sorted(
                    endpoint_path(endpoint.node_name, endpoint.node_namespace)
                    for endpoint in self.get_publishers_info_by_topic("/tf")
                )
            ),
        )
