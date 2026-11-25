"""
Nav2 속도 후보를 제한하고 Unitree Sport request preview로 변환한다.

기본 설정에서는 `/api/sport/request` publisher를 만들지 않는다. 실제 publisher는
`output_enabled`와 `physical_validation_approved`가 모두 참일 때만 생성되며, 현재
고정 전 단계에서는 두 값을 활성화하지 않는다.
"""

from time import monotonic_ns
from typing import Callable, Final

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.publisher import Publisher
from unitree_api.msg import Request

from go2_control.motion_contract import (
    AccelerationLimits,
    ActuationGate,
    CommandDecision,
    DecisionKind,
    MotionCommand,
    MotionInput,
    MotionLimits,
    VelocityLimits,
    assess_motion_input,
)
from go2_control.sport_request import (
    SportRequestData,
    build_move_request,
    build_stop_request,
)


INPUT_TOPIC: Final = "/go2_control/cmd_vel_candidate"
PREVIEW_TOPIC: Final = "/go2_control/sport_request_preview"
CONTROL_TOPIC: Final = "/api/sport/request"
NANOSECONDS_PER_SECOND: Final = 1_000_000_000


class MotionAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("go2_motion_adapter")
        self._limits = self._declare_limits()
        self._gate = ActuationGate(
            output_enabled=self._declare_bool("output_enabled", False),
            physical_validation_approved=self._declare_bool(
                "physical_validation_approved",
                False,
            ),
        )
        self._preview_publisher = self.create_publisher(
            Request,
            PREVIEW_TOPIC,
            10,
        )
        self._control_publisher: Publisher | None = None
        if (
            self._gate.output_enabled
            and self._gate.physical_validation_approved
        ):
            self._control_publisher = self.create_publisher(
                Request,
                CONTROL_TOPIC,
                10,
            )
        self.create_subscription(Twist, INPUT_TOPIC, self._on_twist, 10)
        self.create_timer(0.05, self._on_watchdog)
        self._previous_command = MotionCommand(0.0, 0.0, 0.0)
        self._last_command_time_ns: int | None = None
        self._stop_sent = False
        self._rejected_count = 0
        self._preview_count = 0
        self._published_count = 0
        publisher_created = self.control_publisher_created
        self.get_logger().info(
            f"motion adapter ready: input={INPUT_TOPIC} "
            f"preview={PREVIEW_TOPIC} "
            f"control_publisher_created={publisher_created}"
        )

    @property
    def control_publisher_created(self) -> bool:
        """Return whether this process owns a Go2 control publisher."""
        return self._control_publisher is not None

    def _declare_limits(self) -> MotionLimits:
        return MotionLimits(
            velocity=VelocityLimits(
                forward=self._declare_double("max_forward_m_s", 0.30),
                reverse=self._declare_double("max_reverse_m_s", 0.20),
                lateral=self._declare_double("max_lateral_m_s", 0.15),
                yaw=self._declare_double("max_yaw_rad_s", 0.40),
            ),
            acceleration=AccelerationLimits(
                linear=self._declare_double(
                    "max_linear_acceleration_m_s2",
                    0.50,
                ),
                yaw=self._declare_double(
                    "max_yaw_acceleration_rad_s2",
                    1.00,
                ),
            ),
            timeout_nanoseconds=(
                self.declare_parameter("command_timeout_ms", 250)
                .get_parameter_value()
                .integer_value
                * 1_000_000
            ),
        )

    def _declare_bool(self, name: str, default: bool) -> bool:
        return (
            self.declare_parameter(name, default)
            .get_parameter_value()
            .bool_value
        )

    def _declare_double(self, name: str, default: float) -> float:
        return (
            self.declare_parameter(name, default)
            .get_parameter_value()
            .double_value
        )

    def _on_twist(self, message: Twist) -> None:
        now_ns = monotonic_ns()
        elapsed_seconds = (
            1.0
            if self._last_command_time_ns is None
            else (now_ns - self._last_command_time_ns) / NANOSECONDS_PER_SECOND
        )
        decision = assess_motion_input(
            MotionInput(
                command=MotionCommand(
                    velocity_x=message.linear.x,
                    velocity_y=message.linear.y,
                    yaw_rate=message.angular.z,
                ),
                age_nanoseconds=0,
                previous_command=self._previous_command,
                elapsed_seconds=elapsed_seconds,
            ),
            self._limits,
            self._gate,
        )
        if decision.kind is DecisionKind.REJECTED:
            self._rejected_count += 1
            self.get_logger().warning(
                f"motion command rejected: errors={decision.errors}"
            )
            return
        self._emit_request(decision, build_move_request)
        if decision.command is not None:
            self._previous_command = decision.command
        self._last_command_time_ns = now_ns
        self._stop_sent = False

    def _on_watchdog(self) -> None:
        if self._last_command_time_ns is None or self._stop_sent:
            return
        now_ns = monotonic_ns()
        decision = assess_motion_input(
            MotionInput(
                command=self._previous_command,
                age_nanoseconds=now_ns - self._last_command_time_ns,
                previous_command=self._previous_command,
                elapsed_seconds=(now_ns - self._last_command_time_ns)
                / NANOSECONDS_PER_SECOND,
            ),
            self._limits,
            self._gate,
        )
        stop_kinds = (DecisionKind.STOP_PREVIEW, DecisionKind.STOP_READY)
        if decision.kind not in stop_kinds:
            return
        self._emit_request(decision, lambda _command: build_stop_request())
        self._previous_command = MotionCommand(0.0, 0.0, 0.0)
        self._stop_sent = True

    def _emit_request(
        self,
        decision: CommandDecision,
        builder: Callable[[MotionCommand], SportRequestData],
    ) -> None:
        if decision.command is None:
            return
        request_data = builder(decision.command)
        message = self._to_ros_request(request_data)
        self._preview_publisher.publish(message)
        self._preview_count += 1
        if decision.should_publish and self._control_publisher is not None:
            self._control_publisher.publish(message)
            self._published_count += 1

    @staticmethod
    def _to_ros_request(request_data: SportRequestData) -> Request:
        message = Request()
        message.header.identity.api_id = request_data.api_id
        message.parameter = request_data.parameter
        return message


def main(args: list[str] | None = None) -> None:
    """Run the motion adapter until ROS shuts down."""
    rclpy.init(args=args)
    node = MotionAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info(
                "motion adapter stopped by keyboard interrupt"
            )
    except ExternalShutdownException:
        return
    finally:
        if rclpy.ok():
            node.get_logger().info(
                f"motion adapter summary: previews={node._preview_count} "
                f"published={node._published_count} "
                f"rejected={node._rejected_count}"
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
