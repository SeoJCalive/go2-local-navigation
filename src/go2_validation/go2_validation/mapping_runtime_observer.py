
"""Domain 63 mapping의 stream·clock·TF·command graph를 읽기 전용 관찰한다.
원본 message는 count와 시간 경계만 누적하고 payload를 보관하지 않는다. Humble
rclpy가 TF callback GID를 노출하지 않아 global owner는 관찰된 map→odom edge와
현재 `/tf` publisher node topology를 결합해 기록한다.
"""

from time import monotonic
from typing import Final

from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage

from go2_validation.mapping_acceptance import (
    MappingOwnershipObservation,
    MappingStreamObservation,
)
from go2_validation.mapping_runtime_data import BagExpectation
from go2_validation.mapping_runtime_graph import (
    MAP_QOS,
    RAW_CLOUD_QOS,
    RAW_ODOMETRY_QOS,
    RUNTIME_NODES,
    STREAM_QOS,
    global_tf_owner_nodes,
    node_paths,
)
from go2_validation.mapping_scan_quality import MappingScanQualityAccumulator
from go2_validation.mapping_pose_continuity import (
    MapCorrectionContinuityAccumulator,
    MappingCorrectionContinuityObservation,
)
from go2_validation.mapping_tf_continuity import (
    MappingTfContinuityAccumulator,
    MappingTfContinuityObservation,
)


NANOSECONDS_PER_SECOND: Final = 1_000_000_000
CLOCK_STALL_SECONDS: Final = 10.0
class MappingRuntimeObserver(Node):
    """한 replay 동안 bounded counters와 graph 최대값을 누적하는 mutable observer다."""

    def __init__(self, variant_id: str) -> None:
        super().__init__(f"mapping_runtime_observer_{variant_id}")
        self._raw_cloud_count = 0
        self._raw_odometry_count = 0
        self._scan_count = 0
        self._odom_count = 0
        self._map_count = 0
        self._map_frames: set[str] = set()
        self._map_has_cells = False
        self._clock_first: int | None = None
        self._clock_last: int | None = None
        self._clock_last_wall: float | None = None
        self._clock_max_wall_gap = 0.0
        self._clock_publisher_max = 0
        self._global_edges: set[tuple[str, str]] = set()
        self._global_owner_nodes_seen: set[str] = set()
        self._map_to_odom_continuity = MappingTfContinuityAccumulator()
        self._map_correction_continuity = MapCorrectionContinuityAccumulator()
        self._odometry_continuity = MappingTfContinuityAccumulator()
        self._scan_quality = MappingScanQualityAccumulator()
        self._command_publisher_max = 0
        self._control_node_max = 0
        self._observer_subscription_handles = (
            self.create_subscription(
                PointCloud2,
                "/utlidar/cloud",
                self._on_raw_cloud,
                RAW_CLOUD_QOS,
            ),
            self.create_subscription(
                Odometry,
                "/utlidar/robot_odom",
                self._on_raw_odometry,
                RAW_ODOMETRY_QOS,
            ),
            self.create_subscription(LaserScan, "/scan", self._on_scan, STREAM_QOS),
            self.create_subscription(Odometry, "/odom", self._on_odom, STREAM_QOS),
            self.create_subscription(OccupancyGrid, "/map", self._on_map, MAP_QOS),
            self.create_subscription(Clock, "/clock", self._on_clock, STREAM_QOS),
            self.create_subscription(TFMessage, "/tf", self._on_tf, STREAM_QOS),
        )

    def _on_raw_cloud(self, _message: PointCloud2) -> None:
        self._raw_cloud_count += 1

    def _on_raw_odometry(self, _message: Odometry) -> None:
        self._raw_odometry_count += 1

    def _on_scan(self, message: LaserScan) -> None:
        self._scan_count += 1
        self._scan_quality.observe(
            message.ranges,
            message.range_min,
            message.range_max,
        )

    def _on_odom(self, message: Odometry) -> None:
        self._odom_count += 1
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self._odometry_continuity.observe(
            (position.x, position.y, position.z),
            (orientation.x, orientation.y, orientation.z, orientation.w),
            stamp_nanoseconds=(
                message.header.stamp.sec * NANOSECONDS_PER_SECOND
                + message.header.stamp.nanosec
            ),
        )
        self._map_correction_continuity.observe_odometry(
            (position.x, position.y, position.z),
            (orientation.x, orientation.y, orientation.z, orientation.w),
            message.header.stamp.sec * NANOSECONDS_PER_SECOND
            + message.header.stamp.nanosec,
        )

    def _on_map(self, message: OccupancyGrid) -> None:
        self._map_count += 1
        self._map_frames.add(message.header.frame_id)
        expected_cells = message.info.width * message.info.height
        self._map_has_cells = self._map_has_cells or (
            expected_cells > 0 and len(message.data) == expected_cells
        )

    def _on_clock(self, message: Clock) -> None:
        stamp = message.clock.sec * NANOSECONDS_PER_SECOND + message.clock.nanosec
        wall_now = monotonic()
        if self._clock_first is None:
            self._clock_first = stamp
        if self._clock_last_wall is not None:
            self._clock_max_wall_gap = max(
                self._clock_max_wall_gap,
                wall_now - self._clock_last_wall,
            )
        self._clock_last = stamp
        self._clock_last_wall = wall_now

    def _on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.child_frame_id == "odom":
                edge = (transform.header.frame_id, transform.child_frame_id)
                self._global_edges.add(edge)
                if edge == ("map", "odom"):
                    self._map_to_odom_continuity.observe_transform(transform)
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    self._map_correction_continuity.observe_map_to_odom(
                        (translation.x, translation.y, translation.z),
                        (rotation.x, rotation.y, rotation.z, rotation.w),
                        transform.header.stamp.sec * NANOSECONDS_PER_SECOND
                        + transform.header.stamp.nanosec,
                    )

    def observe_graph(self) -> None:
        """Clock·command·control node와 global-capable TF endpoint 최대값을 갱신한다."""
        self._clock_publisher_max = max(
            self._clock_publisher_max,
            len(self.get_publishers_info_by_topic("/clock")),
        )
        command_count = sum(
            len(self.get_publishers_info_by_topic(topic))
            for topic in ("/api/sport/request", "/lowcmd")
        )
        self._command_publisher_max = max(self._command_publisher_max, command_count)
        nodes = node_paths(self)
        control_nodes = sum(path == "/go2_motion_adapter" for path in nodes)
        self._control_node_max = max(self._control_node_max, control_nodes)
        self._global_owner_nodes_seen.update(global_tf_owner_nodes(self))

    def ready_for_player(self) -> bool:
        """Raw subscribers, derived publishers와 SLAM scan subscriber가 준비됐는지 본다."""
        nodes = node_paths(self)
        return all(
            (
                bool(self.get_subscriptions_info_by_topic("/utlidar/cloud")),
                bool(self.get_subscriptions_info_by_topic("/utlidar/robot_odom")),
                bool(self.get_publishers_info_by_topic("/scan")),
                bool(self.get_subscriptions_info_by_topic("/scan")),
                bool(self.get_publishers_info_by_topic("/odom")),
                "/slam_toolbox" in nodes,
            )
        )

    def clock_stalled_during_playback(self) -> bool:
        """첫 clock 이후 10초 넘게 새 clock callback이 없으면 true다."""
        return (
            self._clock_last_wall is not None
            and monotonic() - self._clock_last_wall > CLOCK_STALL_SECONDS
        )

    def stream_observation(
        self,
        expectation: BagExpectation,
    ) -> MappingStreamObservation:
        """누적 counters를 expected full-input count와 결합한다."""
        return MappingStreamObservation(
            expected_cloud_count=expectation.cloud_count,
            observed_cloud_count=self._raw_cloud_count,
            expected_odometry_count=expectation.odometry_count,
            observed_odometry_count=self._raw_odometry_count,
            scan_count=self._scan_count,
            odom_count=self._odom_count,
            map_count=self._map_count,
            map_frames=tuple(sorted(self._map_frames)),
            map_has_cells=self._map_has_cells,
            scan_quality=self._scan_quality.observation(),
        )

    def ownership_observation(
        self,
        slam_services_ready: bool,
    ) -> MappingOwnershipObservation:
        """Run 동안의 SLAM service, clock, TF와 command 최대값을 고정한다."""
        clock_progressed = (
            self._clock_first is not None
            and self._clock_last is not None
            and self._clock_last > self._clock_first
        )
        return MappingOwnershipObservation(
            slam_services_ready=slam_services_ready,
            clock_publisher_max=self._clock_publisher_max,
            clock_progressed=clock_progressed,
            clock_stalled=self._clock_max_wall_gap > CLOCK_STALL_SECONDS,
            global_edges=tuple(sorted(self._global_edges)),
            global_owner_nodes=tuple(sorted(self._global_owner_nodes_seen)),
            command_publisher_max=self._command_publisher_max,
            control_node_max=self._control_node_max,
        )

    def residual_nodes(self) -> tuple[str, ...]:
        """Teardown 뒤 남은 Todo 12 node path만 반환한다."""
        return tuple(sorted(node_paths(self).intersection(RUNTIME_NODES)))

    def continuity_observation(self) -> MappingTfContinuityObservation:
        """Run 동안 누적한 `map → odom` bounded step 통계를 반환한다."""
        return self._map_to_odom_continuity.observation()

    def odometry_continuity_observation(self) -> MappingTfContinuityObservation:
        """Run 동안 관찰한 project odometry step 통계를 반환한다."""
        return self._odometry_continuity.observation()

    def map_correction_continuity_observation(
        self,
    ) -> MappingCorrectionContinuityObservation:
        """공통 current odometry pose에서 측정한 map 보정량을 반환한다."""
        return self._map_correction_continuity.observation()

    def teardown_clock_publishers(self) -> int:
        """현재 graph의 clock publisher 수를 반환한다."""
        return len(self.get_publishers_info_by_topic("/clock"))

    def teardown_global_owner_nodes(self) -> tuple[str, ...]:
        """현재 graph에서 local TF owner를 제외한 `/tf` publisher를 반환한다."""
        return tuple(sorted(global_tf_owner_nodes(self)))

    def teardown_complete(self) -> bool:
        """Owned node와 global TF owner가 graph에서 모두 제거됐는지 확인한다."""
        return not self.residual_nodes() and not self.teardown_global_owner_nodes()

    @property
    def map_count(self) -> int:
        """저장 service 진입 전에 확인할 occupancy message 수다."""
        return self._map_count
