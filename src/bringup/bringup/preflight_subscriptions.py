"""필수 ROS topic type을 preflight 표본 변환기와 연결한다."""

from typing import Protocol

from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.subscription import Subscription
from sensor_msgs.msg import Imu, PointCloud2
from unitree_go.msg import LowState

from bringup.preflight_ros_samples import (
    imu_sample,
    lowstate_sample,
    occupancy_grid_sample,
    odometry_sample,
    point_cloud_sample,
)
from bringup.preflight_types import ObservedMessage


TOPIC_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


class TopicSampleSink(Protocol):
    """변환된 표본을 topic accumulator로 전달하는 최소 계약이다."""

    def observe_topic(self, topic: str, sample: ObservedMessage) -> None:
        """지정 topic의 표본 하나를 누적한다."""
        ...


def bind_preflight_subscriptions(
    node: Node,
    sink: TopicSampleSink,
) -> tuple[Subscription, ...]:
    """필수 raw·derived topic의 읽기 전용 subscription을 생성한다."""
    return (
        node.create_subscription(
            LowState,
            "/lf/lowstate",
            lambda message: sink.observe_topic(
                "/lf/lowstate", lowstate_sample(message)
            ),
            TOPIC_QOS,
        ),
        node.create_subscription(
            PointCloud2,
            "/utlidar/cloud",
            lambda message: sink.observe_topic(
                "/utlidar/cloud", point_cloud_sample(message)
            ),
            TOPIC_QOS,
        ),
        node.create_subscription(
            Imu,
            "/utlidar/imu",
            lambda message: sink.observe_topic(
                "/utlidar/imu", imu_sample(message)
            ),
            TOPIC_QOS,
        ),
        node.create_subscription(
            Odometry,
            "/utlidar/robot_odom",
            lambda message: sink.observe_topic(
                "/utlidar/robot_odom", odometry_sample(message)
            ),
            TOPIC_QOS,
        ),
        node.create_subscription(
            Odometry,
            "/odom",
            lambda message: sink.observe_topic(
                "/odom", odometry_sample(message)
            ),
            TOPIC_QOS,
        ),
        node.create_subscription(
            PointCloud2,
            "/perception/obstacle_candidates",
            lambda message: sink.observe_topic(
                "/perception/obstacle_candidates",
                point_cloud_sample(message),
            ),
            TOPIC_QOS,
        ),
        node.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            lambda message: sink.observe_topic(
                "/local_costmap/costmap",
                occupancy_grid_sample(message),
            ),
            TOPIC_QOS,
        ),
    )
