"""Publish base-frame LiDAR obstacle candidates without proving occupancy or free space.

The node reads only `/utlidar/cloud`. Every point transform comes from the
existing project TF tree; this module stores no sensor extrinsic values and
never publishes commands, calls services, or accesses physical-control APIs.
"""

from collections.abc import Iterator
from typing import Final

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from go2_perception.perception_contract import (
    INPUT_TOPIC,
    OUTPUT_FRAME_ID,
    OUTPUT_TOPIC,
    SOURCE_FRAME_ID,
    PointXYZ,
    RigidTransform,
    transform_and_filter_points,
)


POINT_CLOUD_QOS: Final = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class ObstacleCandidateNode(Node):
    """Transform and filter read-only LiDAR samples into obstacle candidates."""

    def __init__(self) -> None:
        super().__init__("go2_obstacle_candidates")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(PointCloud2, OUTPUT_TOPIC, POINT_CLOUD_QOS)
        self.create_subscription(PointCloud2, INPUT_TOPIC, self._on_cloud, POINT_CLOUD_QOS)
        self.get_logger().info(
            f"subscribed to {INPUT_TOPIC}; publishing base-frame obstacle candidates to {OUTPUT_TOPIC}"
        )

    def _on_cloud(self, message: PointCloud2) -> None:
        if message.header.frame_id != SOURCE_FRAME_ID:
            self.get_logger().error(
                f"rejected cloud with frame '{message.header.frame_id}'; expected '{SOURCE_FRAME_ID}'"
            )
            return
        try:
            tf_message = self._tf_buffer.lookup_transform(
                OUTPUT_FRAME_ID,
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
            )
        except TransformException as error:
            self.get_logger().warning(f"candidate cloud skipped: TF lookup failed: {error}")
            return

        candidates = transform_and_filter_points(
            _source_points(message),
            _rigid_transform_from(tf_message),
        )
        header = Header()
        header.stamp = message.header.stamp
        header.frame_id = OUTPUT_FRAME_ID
        self._publisher.publish(
            point_cloud2.create_cloud_xyz32(
                header,
                [(point.x, point.y, point.z) for point in candidates],
            )
        )


def _source_points(message: PointCloud2) -> Iterator[PointXYZ]:
    for raw_point in point_cloud2.read_points(
        message,
        field_names=("x", "y", "z"),
        skip_nans=False,
    ):
        yield PointXYZ(
            x=float(raw_point[0]),
            y=float(raw_point[1]),
            z=float(raw_point[2]),
        )


def _rigid_transform_from(tf_message: TransformStamped) -> RigidTransform:
    translation = tf_message.transform.translation
    rotation = tf_message.transform.rotation
    return RigidTransform(
        translation_xyz=(translation.x, translation.y, translation.z),
        rotation_xyzw=(rotation.x, rotation.y, rotation.z, rotation.w),
    )


def main(args: list[str] | None = None) -> None:
    """Run the read-only candidate publisher until ROS shuts down."""
    rclpy.init(args=args)
    node = ObstacleCandidateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info(
                "obstacle candidate node stopped by keyboard interrupt"
            )
    except ExternalShutdownException:
        return
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
