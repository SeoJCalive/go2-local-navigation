"""
읽기 전용 /utlidar/cloud acceptance node를 제공한다.

이 node는 명시 QoS로 PointCloud2를 구독하고 계약 결과만 기록한다. TF,
명령, diagnostics, service를 publish하거나 생성하지 않는다.
"""

from typing import Final

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2

from go2_sensor.lidar_contract import CloudLayout, CloudSample, validate_cloud_sample


LIDAR_TOPIC: Final = "/utlidar/cloud"
NANOSECONDS_PER_SECOND: Final = 1_000_000_000
LIDAR_QOS: Final = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class LidarAcceptanceNode(Node):
    """LiDAR cloud 하나를 읽고 acceptance 계약만 적용하는 node다."""

    def __init__(self) -> None:
        super().__init__("lidar_acceptance")
        self._first_layout_logged = False
        self._previous_stamp_nanoseconds: int | None = None
        self._valid_sample_count = 0
        self._invalid_sample_count = 0
        self.create_subscription(
            PointCloud2,
            LIDAR_TOPIC,
            self._on_cloud,
            LIDAR_QOS,
        )

    @property
    def valid_sample_count(self) -> int:
        """현재 node 수명 동안 수용한 cloud 수를 반환한다."""
        return self._valid_sample_count

    @property
    def invalid_sample_count(self) -> int:
        """현재 node 수명 동안 거부한 cloud 수를 반환한다."""
        return self._invalid_sample_count

    def _on_cloud(self, message: PointCloud2) -> None:
        """수신 cloud의 layout을 한 번 기록하고 acceptance 결과를 집계한다."""
        layout = CloudLayout(
            height=message.height,
            width=message.width,
            point_step=message.point_step,
            field_names=tuple(field.name for field in message.fields),
        )
        if not self._first_layout_logged:
            self.get_logger().info(
                f"first LiDAR layout: height={layout.height} width={layout.width} "
                f"point_step={layout.point_step} fields={layout.field_names}"
            )
            self._first_layout_logged = True

        stamp_nanoseconds = (
            message.header.stamp.sec * NANOSECONDS_PER_SECOND
            + message.header.stamp.nanosec
        )
        sample = CloudSample(
            frame_id=message.header.frame_id,
            stamp_nanoseconds=stamp_nanoseconds,
            layout=layout,
        )
        result = validate_cloud_sample(sample, self._previous_stamp_nanoseconds)
        if stamp_nanoseconds > 0:
            self._previous_stamp_nanoseconds = stamp_nanoseconds
        if result.is_valid:
            self._valid_sample_count += 1
            return

        self._invalid_sample_count += 1
        self.get_logger().warning(
            f"rejected LiDAR cloud: reason={result.reason} "
            f"valid={self._valid_sample_count} invalid={self._invalid_sample_count}"
        )


def main(args: list[str] | None = None) -> None:
    """Console entry point에서 node를 시작하고 종료 시 ROS resource를 해제한다."""
    rclpy.init(args=args)
    node: LidarAcceptanceNode | None = None
    try:
        node = LidarAcceptanceNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.get_logger().info(
                f"LiDAR acceptance summary: valid={node.valid_sample_count} "
                f"invalid={node.invalid_sample_count}"
            )
            node.destroy_node()
        rclpy.shutdown()
