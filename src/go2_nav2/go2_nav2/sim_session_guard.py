"""모드 소유권을 감시하고 active AMCL에 초기 pose를 정확히 한 번 보낸다.

AMCL의 새 map-frame pose가 요청 근처에 도착해야 navigation lifecycle을 시작한다.
이 확인은 초기화 수락 관찰이며 ground truth 위치 정확도 증명이 아니다.
시작 제한 시간과 소유권 위반은 예외로 종료해 W03 launch가 전체 ROS 세션을 정리한다.
simulator/RL/bridge의 종료 소유권은 이 node에 없다.
"""

import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from go2_nav2.sim_session import pose_accepted


class SessionGuard(Node):
    def __init__(self):
        super().__init__("go2_sim_session_guard")
        self.mode = self.declare_parameter("mode", "").value
        self.pose = [
            float(self.declare_parameter(name, 0.0).value)
            for name in ("initial_x", "initial_y", "initial_yaw")
        ]
        self.started = time.monotonic()
        self.sent = None
        self.accepted = False
        self.navigation_active = False
        self.state_future = None
        self.nav_future = None
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_pose, 10)
        self.state = self.create_client(GetState, "/amcl/get_state")
        self.navigation = self.create_client(
            ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes")
        self.create_timer(0.1, self.check, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_pose(self, message):
        if self.sent is None or self.accepted or message.header.frame_id != "map":
            return
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if stamp < self.sent:
            return
        pose = message.pose.pose
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        if not pose_accepted(self.pose, pose.position.x, pose.position.y, yaw):
            raise RuntimeError("AMCL response is outside initial pose acceptance window")
        self.accepted = True
        self.get_logger().info(
            f"INITIAL_POSE_ACCEPTED x={pose.position.x} y={pose.position.y} yaw={yaw} stamp={stamp}")

    def check(self):
        names = [name for name, _namespace in self.get_node_names_and_namespaces()]
        forbidden = "amcl" if self.mode == "mapping" else "slam_toolbox"
        owner = "slam_toolbox" if self.mode == "mapping" else "amcl"
        if forbidden in names or names.count(owner) > 1:
            raise RuntimeError(f"map->odom ownership conflict: mode={self.mode}, nodes={names}")
        if self.mode == "mapping":
            return
        if not self.navigation_active and time.monotonic()-self.started > 90:
            raise RuntimeError("AMCL initial pose/navigation startup not confirmed within 90 wall seconds")
        if self.sent is None:
            if self.state_future is None:
                if self.state.service_is_ready():
                    self.state_future = self.state.call_async(GetState.Request())
                return
            if not self.state_future.done():
                return
            active = self.state_future.result().current_state.id == 3
            self.state_future = None
            if not active or self.publisher.get_subscription_count() == 0:
                return
            message = PoseWithCovarianceStamped()
            message.header.frame_id = "map"
            now = self.get_clock().now()
            if now.nanoseconds == 0:
                return
            message.header.stamp = now.to_msg()
            message.pose.pose.position.x, message.pose.pose.position.y = self.pose[:2]
            message.pose.pose.orientation.z = math.sin(self.pose[2]/2)
            message.pose.pose.orientation.w = math.cos(self.pose[2]/2)
            message.pose.covariance[0] = message.pose.covariance[7] = 0.25
            message.pose.covariance[35] = (math.pi/12)**2
            self.sent = now.nanoseconds
            self.publisher.publish(message)
            self.get_logger().info(f"INITIAL_POSE_SENT_ONCE pose={self.pose} stamp={self.sent}")
        if self.accepted and not self.navigation_active:
            if self.nav_future is None:
                if self.navigation.service_is_ready():
                    request = ManageLifecycleNodes.Request()
                    request.command = ManageLifecycleNodes.Request.STARTUP
                    self.nav_future = self.navigation.call_async(request)
            elif self.nav_future.done():
                if not self.nav_future.result().success:
                    raise RuntimeError("Nav2 lifecycle startup failed")
                self.navigation_active = True
                self.get_logger().info("NAVIGATION_READY initial pose accepted; map->odom owner=amcl")


def main():
    rclpy.init()
    node = SessionGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
