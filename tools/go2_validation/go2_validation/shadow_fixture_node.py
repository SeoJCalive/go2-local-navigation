"""Domain 65에서 `/clock`, `map→odom`, `odom→base`만 소유하는 fixture node다."""

from math import cos, sin

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

from go2_validation.shadow_fixture import fixture_plan_for


class ShadowFixtureNode(Node):
    """합성 clock·odometry와 두 dynamic TF edge를 단독 publish한다."""

    def __init__(self) -> None:
        super().__init__("synthetic_navigation_fixture")
        scenario_id = str(self.declare_parameter("scenario_id", "success").value)
        self._plan = fixture_plan_for(scenario_id)
        self._nanoseconds = 0
        self._x = -1.75
        self._y = -1.75
        self._yaw = 0.0
        self._velocity = Twist()
        qos = QoSProfile(depth=10)
        self._clock_publisher = self.create_publisher(Clock, "/clock", qos)
        self._tf_publisher = self.create_publisher(TFMessage, "/tf", qos)
        self._odom_publisher = self.create_publisher(Odometry, "/odom", qos)
        self.create_subscription(
            Twist,
            self._plan.shadow_velocity_topic,
            self._velocity_callback,
            qos,
        )
        self.create_timer(0.05, self._tick)

    def _velocity_callback(self, message: Twist) -> None:
        self._velocity = message

    def _tick(self) -> None:
        self._nanoseconds += 50_000_000
        if self._plan.integrates_shadow_velocity:
            seconds = 0.05
            self._x += self._velocity.linear.x * cos(self._yaw) * seconds
            self._y += self._velocity.linear.x * sin(self._yaw) * seconds
            self._yaw += self._velocity.angular.z * seconds
        clock = Clock()
        clock.clock.sec = self._nanoseconds // 1_000_000_000
        clock.clock.nanosec = self._nanoseconds % 1_000_000_000
        self._clock_publisher.publish(clock)
        odom = Odometry()
        odom.header.stamp = clock.clock
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base"
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = sin(self._yaw / 2.0)
        odom.pose.pose.orientation.w = cos(self._yaw / 2.0)
        self._odom_publisher.publish(odom)
        self._tf_publisher.publish(
            TFMessage(
                transforms=[self._map_to_odom(clock), self._odom_to_base(clock)]
            )
        )

    def _map_to_odom(self, clock: Clock) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = clock.clock
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.rotation.w = 1.0
        return transform

    def _odom_to_base(self, clock: Clock) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = clock.clock
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base"
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.rotation.z = sin(self._yaw / 2.0)
        transform.transform.rotation.w = cos(self._yaw / 2.0)
        return transform


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ShadowFixtureNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        return
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
