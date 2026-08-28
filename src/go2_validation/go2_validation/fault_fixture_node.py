
"""Publish deterministic raw cloud and odometry faults for isolated domain61 runs.
The fixture owns `/clock` and publishes only synthetic inputs consumed by the
Todo 4 cloud gate and Todo 5 odometry adapter.  It never creates a control
publisher, calls a service, or reads a live Go2 topic.
"""

import json
from math import nan
import struct
from typing import Final

import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from go2_validation.fault_fixture_model import (
    BASELINE_CLOCK_NANOSECONDS,
    FAULT_OFFSET_NANOSECONDS,
    FaultKind,
    FaultScenario,
    FixtureEvent,
    FixturePhase,
    build_attempt_events,
    expected_child_exit_code,
)


CLOCK_TOPIC: Final = "/clock"
RAW_CLOUD_TOPIC: Final = "/utlidar/cloud"
RAW_ODOMETRY_TOPIC: Final = "/utlidar/robot_odom"
EVENT_TOPIC: Final = "/go2_fault/fixture_event"
NANOSECONDS_PER_SECOND: Final = 1_000_000_000
STARTUP_TICKS: Final = 15
TICK_NANOSECONDS: Final = 50_000_000
QOS: Final = QoSProfile(depth=10)
ODOMETRY_RECOVERY_FAULTS: Final = frozenset(
    {
        FaultKind.ODOM_REGRESSION,
        FaultKind.ODOM_JUMP,
        FaultKind.ODOM_LOSS,
    }
)


class FaultFixtureNode(Node):
    """Emit one fault-oracle attempt after downstream subscriptions can connect."""

    def __init__(self) -> None:
        super().__init__("go2_fault_fixture")
        scenario = FaultScenario(
            scenario_id=self.declare_parameter("scenario_id", "unset").value,
            fault_kind=FaultKind(self.declare_parameter("fault_kind", "empty_cloud").value),
            reason_code=self.declare_parameter("reason_code", "EMPTY_CLOUD").value,
            recovery_deadline_nanoseconds=int(
                self.declare_parameter("recovery_deadline_ns", NANOSECONDS_PER_SECOND).value
            ),
        )
        restart_attempt = bool(self.declare_parameter("restart_attempt", False).value)
        self._scenario = scenario
        self._events = build_attempt_events(scenario, restart_attempt=restart_attempt)
        self._event_index = 0
        self._current_clock_nanoseconds = (
            BASELINE_CLOCK_NANOSECONDS + FAULT_OFFSET_NANOSECONDS
            if restart_attempt
            else BASELINE_CLOCK_NANOSECONDS - STARTUP_TICKS * TICK_NANOSECONDS
        )
        self._exit_code = expected_child_exit_code(scenario) if not self._events else 0
        self._done = not self._events
        self._clock_publisher = self.create_publisher(Clock, CLOCK_TOPIC, QOS)
        self._cloud_publisher = self.create_publisher(PointCloud2, RAW_CLOUD_TOPIC, QOS)
        self._odometry_publisher = self.create_publisher(Odometry, RAW_ODOMETRY_TOPIC, QOS)
        self._event_publisher = self.create_publisher(String, EVENT_TOPIC, QOS)
        self.create_timer(0.05, self._tick)

    @property
    def done(self) -> bool:
        """Return whether the fixture has emitted its final attempt boundary."""
        return self._done

    @property
    def exit_code(self) -> int:
        """Return the child result expected by the owning acceptance runner."""
        return self._exit_code

    def _tick(self) -> None:
        if self._done:
            return
        event = self._events[self._event_index]
        if self._current_clock_nanoseconds + TICK_NANOSECONDS < event.clock_nanoseconds:
            self._current_clock_nanoseconds += TICK_NANOSECONDS
            self._publish_clock(self._current_clock_nanoseconds)
            return
        recovery_seed_nanoseconds = event.clock_nanoseconds - 1
        if (
            event.phase is FixturePhase.RECOVERED
            and self._scenario.fault_kind in ODOMETRY_RECOVERY_FAULTS
            and self._current_clock_nanoseconds < recovery_seed_nanoseconds
        ):
            self._current_clock_nanoseconds = recovery_seed_nanoseconds
            self._publish_clock(recovery_seed_nanoseconds)
            self._publish_odometry(recovery_seed_nanoseconds, False, 0.0)
            return
        self._current_clock_nanoseconds = event.clock_nanoseconds
        self._publish_clock(event.clock_nanoseconds)
        self._publish_inputs(event)
        self._event_publisher.publish(String(data=_event_json(event)))
        if event.phase is FixturePhase.OWNED_CHILD_EXIT:
            self._exit_code = expected_child_exit_code(self._scenario)
        if self._event_index == len(self._events) - 1:
            self._done = True
        else:
            self._event_index += 1

    def _publish_inputs(self, event: FixtureEvent) -> None:
        if event.phase is FixturePhase.SUPPRESSED:
            self._publish_fault_input(event)
            return
        if event.output_counts.validated_cloud > 0:
            self._cloud_publisher.publish(_cloud(event.clock_nanoseconds, False, False))
        if event.output_counts.odom > 0:
            self._publish_odometry(event.clock_nanoseconds, False, 0.0)

    def _publish_fault_input(self, event: FixtureEvent) -> None:
        kind = self._scenario.fault_kind
        match kind:
            case FaultKind.MALFORMED_LAYOUT:
                self._cloud_publisher.publish(
                    _cloud(event.clock_nanoseconds, True, False)
                )
                self._publish_odometry(event.clock_nanoseconds, False, 0.0)
            case FaultKind.EMPTY_CLOUD:
                self._cloud_publisher.publish(
                    _cloud(event.clock_nanoseconds, False, True)
                )
                self._publish_odometry(event.clock_nanoseconds, False, 0.0)
            case FaultKind.NAN_CLOUD:
                self._cloud_publisher.publish(
                    _cloud(event.clock_nanoseconds, False, False, nan)
                )
                self._publish_odometry(event.clock_nanoseconds, False, 0.0)
            case FaultKind.STALE_CLOUD:
                self._cloud_publisher.publish(_cloud(1, False, False))
                self._publish_odometry(event.clock_nanoseconds, False, 0.0)
            case FaultKind.TF_LOSS:
                self._cloud_publisher.publish(
                    _cloud(
                        event.clock_nanoseconds,
                        False,
                        False,
                        1.0,
                        "missing_lidar",
                    )
                )
                self._publish_odometry(event.clock_nanoseconds, False, 0.0)
            case FaultKind.ODOM_REGRESSION:
                self._cloud_publisher.publish(
                    _cloud(event.clock_nanoseconds, False, False)
                )
                self._publish_odometry(
                    BASELINE_CLOCK_NANOSECONDS - 1,
                    False,
                    0.0,
                )
            case FaultKind.ODOM_JUMP:
                self._cloud_publisher.publish(
                    _cloud(event.clock_nanoseconds, False, False)
                )
                self._publish_odometry(event.clock_nanoseconds, False, 5.0)
            case FaultKind.ODOM_LOSS:
                self._cloud_publisher.publish(
                    _cloud(event.clock_nanoseconds, False, False)
                )
            case FaultKind.PROCESS_EXIT | FaultKind.LAUNCH_FAILURE:
                return

    def _publish_clock(self, nanoseconds: int) -> None:
        self._clock_publisher.publish(Clock(clock=_time(nanoseconds)))

    def _publish_odometry(self, nanoseconds: int, _unused: bool, position_x: float) -> None:
        message = Odometry()
        message.header.stamp = _time(nanoseconds)
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = position_x
        message.pose.pose.orientation.w = 1.0
        self._odometry_publisher.publish(message)


def _event_json(event: FixtureEvent) -> str:
    return json.dumps(
        {
            "phase": event.phase.value,
            "clock_nanoseconds": event.clock_nanoseconds,
            "reason_code": event.reason_code,
            "child_exit_code": event.child_exit_code,
        },
        separators=(",", ":"),
    )


def _cloud(
    nanoseconds: int,
    malformed: bool,
    empty: bool,
    point_x: float = 1.0,
    frame_id: str = "utlidar_lidar",
) -> PointCloud2:
    message = PointCloud2()
    message.header.stamp = _time(nanoseconds)
    message.header.frame_id = frame_id
    message.height = 1
    message.width = 0 if empty else 1
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.point_step = 0 if malformed else 12
    message.row_step = message.width * message.point_step
    message.data = b"" if empty or malformed else struct.pack("<fff", point_x, 0.0, 0.0)
    message.is_dense = point_x == point_x
    return message


def _time(nanoseconds: int) -> Time:
    return Time(
        sec=nanoseconds // NANOSECONDS_PER_SECOND,
        nanosec=nanoseconds % NANOSECONDS_PER_SECOND,
    )


def main(args: list[str] | None = None) -> None:
    """Run one deterministic owned fixture attempt and return its oracle exit code."""
    rclpy.init(args=args)
    node = FaultFixtureNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
