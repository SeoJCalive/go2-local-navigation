"""ROS message를 통합 preflight의 고정 크기 관찰값으로 변환한다."""

from math import atan2, isfinite
from time import monotonic_ns

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, PointCloud2
from unitree_go.msg import LowState

from bringup.preflight_types import ObservedMessage, Pose2D


def lowstate_sample(_message: LowState) -> ObservedMessage:
    """header가 없는 LowState의 수신 시각만 보존한다."""
    return ObservedMessage(
        receive_nanoseconds=monotonic_ns(),
        stamp_nanoseconds=None,
        frame_id=None,
        child_frame_id=None,
        is_valid=True,
        pose=None,
    )


def point_cloud_sample(message: PointCloud2) -> ObservedMessage:
    """PointCloud2 header·필수 field·buffer layout을 추출한다."""
    field_names = {field.name for field in message.fields}
    stamp_nanoseconds = _stamp_nanoseconds(message.header.stamp)
    valid = (
        stamp_nanoseconds > 0
        and message.width > 0
        and message.height > 0
        and message.point_step > 0
        and {"x", "y", "z"}.issubset(field_names)
        and len(message.data) == message.row_step * message.height
    )
    return ObservedMessage(
        receive_nanoseconds=monotonic_ns(),
        stamp_nanoseconds=stamp_nanoseconds,
        frame_id=message.header.frame_id,
        child_frame_id=None,
        is_valid=valid,
        pose=None,
    )


def imu_sample(message: Imu) -> ObservedMessage:
    """IMU header와 finite orientation·motion 값을 추출한다."""
    stamp_nanoseconds = _stamp_nanoseconds(message.header.stamp)
    values = (
        message.orientation.x,
        message.orientation.y,
        message.orientation.z,
        message.orientation.w,
        message.angular_velocity.x,
        message.angular_velocity.y,
        message.angular_velocity.z,
        message.linear_acceleration.x,
        message.linear_acceleration.y,
        message.linear_acceleration.z,
    )
    return ObservedMessage(
        receive_nanoseconds=monotonic_ns(),
        stamp_nanoseconds=stamp_nanoseconds,
        frame_id=message.header.frame_id,
        child_frame_id=None,
        is_valid=stamp_nanoseconds > 0 and all(isfinite(value) for value in values),
        pose=None,
    )


def odometry_sample(message: Odometry) -> ObservedMessage:
    """Odometry header·child·finite pose/twist와 평면 pose를 추출한다."""
    stamp_nanoseconds = _stamp_nanoseconds(message.header.stamp)
    orientation = message.pose.pose.orientation
    position = message.pose.pose.position
    twist = message.twist.twist
    values = (
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
        twist.angular.x,
        twist.angular.y,
        twist.angular.z,
    )
    yaw = atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
    )
    return ObservedMessage(
        receive_nanoseconds=monotonic_ns(),
        stamp_nanoseconds=stamp_nanoseconds,
        frame_id=message.header.frame_id,
        child_frame_id=message.child_frame_id,
        is_valid=stamp_nanoseconds > 0 and all(isfinite(value) for value in values),
        pose=Pose2D(x=position.x, y=position.y, yaw=yaw),
    )


def occupancy_grid_sample(message: OccupancyGrid) -> ObservedMessage:
    """Costmap header와 grid 크기 일치를 추출한다."""
    stamp_nanoseconds = _stamp_nanoseconds(message.header.stamp)
    expected_cells = message.info.width * message.info.height
    return ObservedMessage(
        receive_nanoseconds=monotonic_ns(),
        stamp_nanoseconds=stamp_nanoseconds,
        frame_id=message.header.frame_id,
        child_frame_id=None,
        is_valid=(
            stamp_nanoseconds > 0
            and expected_cells > 0
            and len(message.data) == expected_cells
        ),
        pose=None,
    )


def _stamp_nanoseconds(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec
