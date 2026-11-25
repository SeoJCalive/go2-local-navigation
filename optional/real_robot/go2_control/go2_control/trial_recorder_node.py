"""
제한적 물리 시험의 candidate·preview·odometry를 구독만 해 JSON record로 남긴다.

이 node는 세 topic의 ROS 메시지를 읽어 shutdown 시 future trial record를 생성한다.
publisher, service client, control interface를 생성하지 않아 명령 전송 경로가 없다.
"""

from datetime import datetime, timezone
from math import atan2
from pathlib import Path
from time import monotonic_ns
from typing import Final

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from unitree_api.msg import Request

from go2_control.trial_record import (
    MotionCandidateObservation,
    OdometryObservation,
    PreviewObservation,
    TrialRecordAccumulator,
    write_trial_record,
)


CANDIDATE_TOPIC: Final = "/go2_control/cmd_vel_candidate"
PREVIEW_TOPIC: Final = "/go2_control/sport_request_preview"
ODOMETRY_TOPIC: Final = "/odom"


class TrialRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("go2_limited_motion_trial_recorder")
        record_path = self._declare_string("record_path", "")
        self._record_path = Path(record_path) if record_path else None
        self._run_label = self._declare_string("run_label", "unlabeled")
        self._records = TrialRecordAccumulator()
        self.create_subscription(Twist, CANDIDATE_TOPIC, self._on_candidate, 10)
        self.create_subscription(Request, PREVIEW_TOPIC, self._on_preview, 10)
        self.create_subscription(Odometry, ODOMETRY_TOPIC, self._on_odometry, 10)

    def _declare_string(self, name: str, default: str) -> str:
        return (
            self.declare_parameter(name, default)
            .get_parameter_value()
            .string_value
        )

    def _on_candidate(self, message: Twist) -> None:
        self._records.observe_candidate(
            received_at_nanoseconds=monotonic_ns(),
            observation=MotionCandidateObservation(
                velocity_x=message.linear.x,
                velocity_y=message.linear.y,
                yaw_rate=message.angular.z,
            ),
        )

    def _on_preview(self, message: Request) -> None:
        self._records.observe_preview(
            received_at_nanoseconds=monotonic_ns(),
            observation=PreviewObservation(
                api_id=message.header.identity.api_id,
                parameter=message.parameter,
            ),
        )

    def _on_odometry(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        yaw_radians = atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        position = message.pose.pose.position
        self._records.observe_odometry(
            received_at_nanoseconds=monotonic_ns(),
            observation=OdometryObservation(
                position_x=position.x,
                position_y=position.y,
                yaw_radians=yaw_radians,
            ),
        )

    def write_record(self) -> None:
        if self._record_path is None:
            return
        record = self._records.snapshot(
            run_label=self._run_label,
            recorded_at=datetime.now(timezone.utc).isoformat(),
        )
        write_trial_record(self._record_path, record)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrialRecorderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        return
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.write_record()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
