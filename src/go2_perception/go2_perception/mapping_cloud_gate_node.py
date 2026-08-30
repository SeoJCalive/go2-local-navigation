"""Forward validated PointCloud2 input to the mapping-only output topic."""

from struct import error as StructError
from typing import Final

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from go2_perception.mapping_cloud_contract import MappingCloudSample, assess_mapping_cloud
from go2_perception.obstacle_candidate_node import POINT_CLOUD_QOS


INPUT_TOPIC: Final = "/utlidar/cloud"
OUTPUT_TOPIC: Final = "/go2_mapping/cloud_validated"
NANOSECONDS_PER_SECOND: Final = 1_000_000_000


class MappingCloudGateNode(Node):
    """Publish only intact, non-stale mapping clouds; it has no control interfaces."""

    def __init__(self) -> None:
        super().__init__("go2_mapping_cloud_gate")
        self._previous_stamp_nanoseconds: int | None = None
        self._publisher = self.create_publisher(PointCloud2, OUTPUT_TOPIC, POINT_CLOUD_QOS)
        self.create_subscription(PointCloud2, INPUT_TOPIC, self._on_cloud, POINT_CLOUD_QOS)

    def _on_cloud(self, message: PointCloud2) -> None:
        stamp_nanoseconds = (
            message.header.stamp.sec * NANOSECONDS_PER_SECOND
            + message.header.stamp.nanosec
        )
        sample = MappingCloudSample(
            stamp_nanoseconds=stamp_nanoseconds,
            width=message.width,
            height=message.height,
            point_step=message.point_step,
            row_step=message.row_step,
            data_length=len(message.data),
            finite_point_count=1,
            field_names=tuple(field.name for field in message.fields),
        )
        now_nanoseconds = self.get_clock().now().nanoseconds
        layout = assess_mapping_cloud(
            sample,
            now_nanoseconds,
            self._previous_stamp_nanoseconds,
        )
        if not layout.publish:
            self.get_logger().warning(
                f"mapping cloud suppressed: {layout.reason_code}"
            )
            return
        try:
            finite_point_count = sum(
                1
                for _ in point_cloud2.read_points(
                    message,
                    field_names=("x", "y", "z"),
                    skip_nans=True,
                )
            )
        except (AssertionError, StructError, TypeError, ValueError) as error:
            self.get_logger().warning(
                f"mapping cloud suppressed: malformed_layout ({error})"
            )
            return
        assessment = assess_mapping_cloud(
            MappingCloudSample(
                stamp_nanoseconds=sample.stamp_nanoseconds,
                width=sample.width,
                height=sample.height,
                point_step=sample.point_step,
                row_step=sample.row_step,
                data_length=sample.data_length,
                finite_point_count=finite_point_count,
                field_names=sample.field_names,
            ),
            now_nanoseconds,
            self._previous_stamp_nanoseconds,
        )
        if not assessment.publish:
            self.get_logger().warning(
                f"mapping cloud suppressed: {assessment.reason_code}"
            )
            return
        self._previous_stamp_nanoseconds = stamp_nanoseconds
        self._publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    """Run the read-only mapping cloud gate until ROS shuts down."""
    rclpy.init(args=args)
    node = MappingCloudGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info(
                "mapping cloud gate stopped by keyboard interrupt"
            )
    except ExternalShutdownException:
        return
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
