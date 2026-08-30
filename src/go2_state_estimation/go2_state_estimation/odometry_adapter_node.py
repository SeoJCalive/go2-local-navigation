"""
Read the Unitree odometry source and expose the project odom interface.

Only the project-accepted ``base_link`` to ``base`` mapping is applied. The
node publishes ``/odom`` and ``odom -> base`` TF; it does not command Go2.
"""

from pathlib import Path
from typing import Final

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

from go2_state_estimation.odometry_contract import (
    ContinuityState,
    OdometrySample,
    PROJECT_TARGET_FRAME_ID,
    advance_continuity,
    assess_odometry_adapter,
    assess_odometry_loss,
    mark_odometry_loss,
)
from go2_state_estimation.continuity_profiles import (
    ContinuityProfile,
    ContinuityProfileError,
    load_continuity_profile,
)


SOURCE_ODOMETRY_TOPIC: Final = "/utlidar/robot_odom"
PROJECT_ODOMETRY_TOPIC: Final = "/odom"
DEFAULT_CONTINUITY_PROFILE: Final = "onboard_observe"
NANOSECONDS_PER_SECOND: Final = 1_000_000_000
SOURCE_ODOMETRY_QOS: Final = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=64,
    durability=DurabilityPolicy.VOLATILE,
)
OUTPUT_ODOMETRY_QOS: Final = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class OdometryAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("go2_odometry_adapter")
        self._previous_timestamp_nanoseconds: int | None = None
        self._continuity_profile = self._load_continuity_profile()
        self._continuity_state = ContinuityState()
        self._loss_reported = False
        self._received_sample_count = 0
        self._published_sample_count = 0
        self._rejected_sample_count = 0
        self._continuity_violation_count = 0
        self._seen_warning_codes: set[str] = set()
        self._odom_publisher = self.create_publisher(
            Odometry,
            PROJECT_ODOMETRY_TOPIC,
            OUTPUT_ODOMETRY_QOS,
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            SOURCE_ODOMETRY_TOPIC,
            self._on_odometry,
            SOURCE_ODOMETRY_QOS,
        )
        self.create_timer(0.1, self._check_loss)
        self.get_logger().info(
            f"adapter subscribed to {SOURCE_ODOMETRY_TOPIC}; "
            f"publishing {PROJECT_ODOMETRY_TOPIC} and odom->{PROJECT_TARGET_FRAME_ID}"
        )

    @property
    def published_sample_count(self) -> int:
        """Return the number of source samples exposed through the project interface."""
        return self._published_sample_count

    @property
    def rejected_sample_count(self) -> int:
        """Return the number of source samples rejected by the adapter gate."""
        return self._rejected_sample_count

    @property
    def continuity_violation_count(self) -> int:
        """Return source-valid continuity failures and recovery-pending observations."""
        return self._continuity_violation_count

    def _load_continuity_profile(self) -> ContinuityProfile:
        profile_id = self.declare_parameter(
            "continuity_profile",
            DEFAULT_CONTINUITY_PROFILE,
        ).value
        if not isinstance(profile_id, str):
            raise ContinuityProfileError("continuity_profile_parameter_invalid")
        profile_path = (
            Path(get_package_share_directory("go2_state_estimation"))
            / "config"
            / "odometry_contract.yaml"
        )
        return load_continuity_profile(profile_path, profile_id)

    def _on_odometry(self, message: Odometry) -> None:
        self._received_sample_count += 1
        sample = OdometrySample(
            timestamp_nanoseconds=(message.header.stamp.sec * NANOSECONDS_PER_SECOND)
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
        assessment = assess_odometry_adapter(
            sample,
            self._previous_timestamp_nanoseconds,
        )
        if assessment.errors:
            self._rejected_sample_count += 1
            self.get_logger().error(
                f"rejected odometry sample: {', '.join(assessment.errors)}"
            )
            return
        self._previous_timestamp_nanoseconds = sample.timestamp_nanoseconds
        transition = advance_continuity(
            self._continuity_state,
            sample,
            self.get_clock().now().nanoseconds,
            self._continuity_profile,
        )
        self._continuity_state = transition.state
        if transition.assessment.reason_code is not None:
            self._continuity_violation_count += 1
            if transition.assessment.publish:
                self.get_logger().warning(
                    "observed odometry continuity: "
                    f"{transition.assessment.reason_code}"
                )
            else:
                self._rejected_sample_count += 1
                self.get_logger().error(
                    "rejected odometry continuity: "
                    f"{transition.assessment.reason_code}"
                )
                return
        self._loss_reported = False

        new_warnings = tuple(
            warning for warning in assessment.warnings if warning not in self._seen_warning_codes
        )
        if new_warnings:
            self._seen_warning_codes.update(new_warnings)
            self.get_logger().warning(
                f"odometry adapter source warning: {', '.join(new_warnings)}"
            )

        output = Odometry()
        output.header.frame_id = message.header.frame_id
        output.header.stamp.sec = message.header.stamp.sec
        output.header.stamp.nanosec = message.header.stamp.nanosec
        output.child_frame_id = PROJECT_TARGET_FRAME_ID
        output.pose = message.pose
        output.twist = message.twist
        self._odom_publisher.publish(output)

        transform = TransformStamped()
        transform.header.frame_id = message.header.frame_id
        transform.header.stamp.sec = message.header.stamp.sec
        transform.header.stamp.nanosec = message.header.stamp.nanosec
        transform.child_frame_id = PROJECT_TARGET_FRAME_ID
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation.x = message.pose.pose.orientation.x
        transform.transform.rotation.y = message.pose.pose.orientation.y
        transform.transform.rotation.z = message.pose.pose.orientation.z
        transform.transform.rotation.w = message.pose.pose.orientation.w
        self._tf_broadcaster.sendTransform(transform)
        self._published_sample_count += 1

    def _check_loss(self) -> None:
        loss = assess_odometry_loss(
            self._continuity_state,
            self.get_clock().now().nanoseconds,
            self._continuity_profile,
        )
        if loss.continuity_valid or self._loss_reported:
            return
        self._continuity_state = mark_odometry_loss(self._continuity_state)
        self._loss_reported = True
        self._continuity_violation_count += 1
        if loss.publish:
            self.get_logger().warning(f"observed odometry continuity loss: {loss.reason_code}")
        else:
            self.get_logger().error(f"odometry continuity loss: {loss.reason_code}")


def main(args: list[str] | None = None) -> None:
    """Run the project odometry adapter until ROS shuts down."""
    rclpy.init(args=args)
    node = OdometryAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info(
                "odometry adapter stopped by keyboard interrupt"
            )
    except RuntimeError:
        if rclpy.ok():
            raise
    except ExternalShutdownException:
        return
    finally:
        if rclpy.ok():
            node.get_logger().info(
                f"odometry adapter summary: received={node._received_sample_count} "
                f"published={node.published_sample_count} "
                f"rejected={node.rejected_sample_count} "
                f"continuity_violations={node.continuity_violation_count} "
                f"warnings={tuple(sorted(node._seen_warning_codes))}"
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
