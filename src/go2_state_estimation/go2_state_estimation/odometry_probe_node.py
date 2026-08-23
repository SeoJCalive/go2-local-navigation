"""Subscribe to the odometry source and log its contract without producing ROS output."""

from typing import Final

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from go2_state_estimation.odometry_contract import OdometrySample, validate_odometry_sample


ODOMETRY_TOPIC: Final = "/utlidar/robot_odom"
PROJECT_TARGET_FRAME: Final = "base"


class OdometryProbeNode(Node):
    def __init__(self) -> None:
        super().__init__("go2_odometry_source_probe")
        self._previous_timestamp_nanoseconds: int | None = None
        self._last_source_child_frame_id: str | None = None
        self._mapping_status_logged = False
        self._received_sample_count = 0
        self._invalid_sample_count = 0
        self._seen_warning_codes: set[str] = set()
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, ODOMETRY_TOPIC, self._on_odometry, qos_profile)
        self.get_logger().info(
            f"read-only odometry source probe subscribed to {ODOMETRY_TOPIC}; no ROS output is created"
        )

    def _on_odometry(self, message: Odometry) -> None:
        self._received_sample_count += 1
        sample = OdometrySample(
            timestamp_nanoseconds=(message.header.stamp.sec * 1_000_000_000)
            + message.header.stamp.nanosec,
            header_frame_id=message.header.frame_id,
            child_frame_id=message.child_frame_id,
            position_xyz=(
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            ),
            orientation_xyzw=(
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w,
            ),
            linear_velocity_xyz=(
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.linear.z,
            ),
            angular_velocity_xyz=(
                message.twist.twist.angular.x,
                message.twist.twist.angular.y,
                message.twist.twist.angular.z,
            ),
            pose_covariance=tuple(message.pose.covariance),
            twist_covariance=tuple(message.twist.covariance),
        )
        assessment = validate_odometry_sample(sample, self._previous_timestamp_nanoseconds)
        self._log_source_child_frame(assessment.source_child_frame_id)
        self._log_mapping_status(assessment.source_child_frame_id)
        if assessment.errors:
            self._invalid_sample_count += 1
            self.get_logger().error(f"odometry source contract invalid: {', '.join(assessment.errors)}")
        new_warnings = tuple(
            warning for warning in assessment.warnings if warning not in self._seen_warning_codes
        )
        if new_warnings:
            self._seen_warning_codes.update(new_warnings)
            self.get_logger().warning(
                f"odometry source contract warning: {', '.join(new_warnings)}"
            )
        if assessment.timestamp_accepted:
            self._previous_timestamp_nanoseconds = sample.timestamp_nanoseconds

    def _log_source_child_frame(self, source_child_frame_id: str) -> None:
        if source_child_frame_id != self._last_source_child_frame_id:
            self._last_source_child_frame_id = source_child_frame_id
            self.get_logger().info(
                f"odometry source child_frame_id observed as '{source_child_frame_id}'"
            )

    def _log_mapping_status(self, source_child_frame_id: str) -> None:
        if not self._mapping_status_logged:
            self._mapping_status_logged = True
            self.get_logger().info(
                "source status: read-only probe does not publish odom->base; "
                "project mapping is handled by the separate adapter "
                f"(source child='{source_child_frame_id}', project target='{PROJECT_TARGET_FRAME}')"
            )


def main() -> None:
    """Run only the read-only source subscription until ROS shuts down."""
    rclpy.init()
    node = OdometryProbeNode()
    try:
        rclpy.spin(node)
    finally:
        node.get_logger().info(
            f"odometry probe summary: received={node._received_sample_count} "
            f"invalid={node._invalid_sample_count} "
            f"warnings={tuple(sorted(node._seen_warning_codes))}"
        )
        node.destroy_node()
        rclpy.shutdown()
