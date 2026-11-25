"""Motion command 계약의 ROS 비의존 경계."""

from dataclasses import dataclass
from enum import Enum
from math import isfinite


class DecisionKind(str, Enum):
    PREVIEW = "preview"
    READY = "ready"
    REJECTED = "rejected"
    STOP_PREVIEW = "stop_preview"
    STOP_READY = "stop_ready"


@dataclass(frozen=True, slots=True)
class MotionCommand:
    velocity_x: float
    velocity_y: float
    yaw_rate: float


@dataclass(frozen=True, slots=True)
class VelocityLimits:
    forward: float
    reverse: float
    lateral: float
    yaw: float


@dataclass(frozen=True, slots=True)
class AccelerationLimits:
    linear: float
    yaw: float


@dataclass(frozen=True, slots=True)
class MotionLimits:
    velocity: VelocityLimits
    acceleration: AccelerationLimits
    timeout_nanoseconds: int


@dataclass(frozen=True, slots=True)
class ActuationGate:
    output_enabled: bool
    physical_validation_approved: bool


@dataclass(frozen=True, slots=True)
class MotionInput:
    command: MotionCommand
    age_nanoseconds: int
    previous_command: MotionCommand
    elapsed_seconds: float


@dataclass(frozen=True, slots=True)
class CommandDecision:
    kind: DecisionKind
    command: MotionCommand | None
    errors: tuple[str, ...]
    should_publish: bool


def assess_motion_input(
    motion_input: MotionInput,
    limits: MotionLimits,
    gate: ActuationGate,
) -> CommandDecision:
    """Assess one motion command without ROS or Unitree side effects."""
    if not all(
        isfinite(value)
        for value in (
            motion_input.command.velocity_x,
            motion_input.command.velocity_y,
            motion_input.command.yaw_rate,
        )
    ):
        return CommandDecision(
            kind=DecisionKind.REJECTED,
            command=None,
            errors=("command_must_be_finite",),
            should_publish=False,
        )

    gate_open = gate.output_enabled and gate.physical_validation_approved
    if motion_input.age_nanoseconds > limits.timeout_nanoseconds:
        decision_kind = (
            DecisionKind.STOP_READY
            if gate_open
            else DecisionKind.STOP_PREVIEW
        )
        return CommandDecision(
            kind=decision_kind,
            command=MotionCommand(
                velocity_x=0.0,
                velocity_y=0.0,
                yaw_rate=0.0,
            ),
            errors=(),
            should_publish=gate_open,
        )

    bounded = _apply_velocity_limits(motion_input.command, limits.velocity)
    rate_limited = _apply_acceleration_limits(
        bounded,
        motion_input.previous_command,
        motion_input.elapsed_seconds,
        limits.acceleration,
    )
    return CommandDecision(
        kind=DecisionKind.READY if gate_open else DecisionKind.PREVIEW,
        command=rate_limited,
        errors=(),
        should_publish=gate_open,
    )


def _apply_velocity_limits(
    command: MotionCommand,
    limits: VelocityLimits,
) -> MotionCommand:
    return MotionCommand(
        velocity_x=min(
            max(command.velocity_x, -limits.reverse),
            limits.forward,
        ),
        velocity_y=min(
            max(command.velocity_y, -limits.lateral),
            limits.lateral,
        ),
        yaw_rate=min(max(command.yaw_rate, -limits.yaw), limits.yaw),
    )


def _apply_acceleration_limits(
    command: MotionCommand,
    previous: MotionCommand,
    elapsed_seconds: float,
    limits: AccelerationLimits,
) -> MotionCommand:
    linear_delta = max(elapsed_seconds, 0.0) * limits.linear
    yaw_delta = max(elapsed_seconds, 0.0) * limits.yaw
    return MotionCommand(
        velocity_x=_move_toward(
            previous.velocity_x,
            command.velocity_x,
            linear_delta,
        ),
        velocity_y=_move_toward(
            previous.velocity_y,
            command.velocity_y,
            linear_delta,
        ),
        yaw_rate=_move_toward(previous.yaw_rate, command.yaw_rate, yaw_delta),
    )


def _move_toward(current: float, target: float, maximum_delta: float) -> float:
    return min(max(target, current - maximum_delta), current + maximum_delta)
