from math import inf, nan

import pytest

from go2_control.motion_contract import (
    AccelerationLimits,
    ActuationGate,
    DecisionKind,
    MotionCommand,
    MotionInput,
    MotionLimits,
    VelocityLimits,
    assess_motion_input,
)


LIMITS = MotionLimits(
    velocity=VelocityLimits(
        forward=0.30,
        reverse=0.20,
        lateral=0.15,
        yaw=0.40,
    ),
    acceleration=AccelerationLimits(linear=0.50, yaw=1.00),
    timeout_nanoseconds=250_000_000,
)
CLOSED_GATE = ActuationGate(
    output_enabled=False,
    physical_validation_approved=False,
)
OPEN_GATE = ActuationGate(
    output_enabled=True,
    physical_validation_approved=True,
)
ZERO_COMMAND = MotionCommand(velocity_x=0.0, velocity_y=0.0, yaw_rate=0.0)


def _input(
    command: MotionCommand,
    *,
    age: int = 0,
    elapsed: float = 1.0,
) -> MotionInput:
    return MotionInput(
        command=command,
        age_nanoseconds=age,
        previous_command=ZERO_COMMAND,
        elapsed_seconds=elapsed,
    )


def test_closed_gate_returns_preview_for_valid_command() -> None:
    decision = assess_motion_input(
        _input(MotionCommand(velocity_x=0.1, velocity_y=0.0, yaw_rate=0.0)),
        LIMITS,
        CLOSED_GATE,
    )

    assert decision.kind is DecisionKind.PREVIEW
    assert decision.command == MotionCommand(
        velocity_x=0.1,
        velocity_y=0.0,
        yaw_rate=0.0,
    )
    assert not decision.should_publish


def test_two_approvals_allow_valid_command_publication() -> None:
    decision = assess_motion_input(
        _input(MotionCommand(velocity_x=0.1, velocity_y=0.0, yaw_rate=0.0)),
        LIMITS,
        OPEN_GATE,
    )

    assert decision.kind is DecisionKind.READY
    assert decision.should_publish


@pytest.mark.parametrize(
    "gate",
    (
        ActuationGate(output_enabled=True, physical_validation_approved=False),
        ActuationGate(output_enabled=False, physical_validation_approved=True),
    ),
)
def test_given_one_approval_when_command_is_valid_then_keeps_output_closed(
    gate: ActuationGate,
) -> None:
    decision = assess_motion_input(_input(ZERO_COMMAND), LIMITS, gate)

    assert decision.kind is DecisionKind.PREVIEW
    assert not decision.should_publish


def test_large_command_is_bounded_by_velocity_envelope() -> None:
    decision = assess_motion_input(
        _input(
            MotionCommand(velocity_x=-1.0, velocity_y=1.0, yaw_rate=-2.0),
            elapsed=10.0,
        ),
        LIMITS,
        CLOSED_GATE,
    )

    assert decision.command == MotionCommand(
        velocity_x=-0.2,
        velocity_y=0.15,
        yaw_rate=-0.4,
    )


def test_fast_change_is_bounded_by_acceleration_envelope() -> None:
    decision = assess_motion_input(
        _input(
            MotionCommand(velocity_x=0.3, velocity_y=-0.3, yaw_rate=0.4),
            elapsed=0.1,
        ),
        LIMITS,
        CLOSED_GATE,
    )

    assert decision.command == MotionCommand(
        velocity_x=0.05,
        velocity_y=-0.05,
        yaw_rate=0.1,
    )


@pytest.mark.parametrize("invalid_value", (nan, inf, -inf))
def test_nonfinite_command_is_rejected(invalid_value: float) -> None:
    decision = assess_motion_input(
        _input(
            MotionCommand(
                velocity_x=invalid_value,
                velocity_y=0.0,
                yaw_rate=0.0,
            )
        ),
        LIMITS,
        CLOSED_GATE,
    )

    assert decision.kind is DecisionKind.REJECTED
    assert decision.command is None
    assert decision.errors == ("command_must_be_finite",)
    assert not decision.should_publish


def test_closed_gate_previews_stop_for_stale_command() -> None:
    decision = assess_motion_input(
        _input(ZERO_COMMAND, age=250_000_001),
        LIMITS,
        CLOSED_GATE,
    )

    assert decision.kind is DecisionKind.STOP_PREVIEW
    assert decision.command == ZERO_COMMAND
    assert not decision.should_publish


def test_open_gate_requests_stop_for_stale_command() -> None:
    decision = assess_motion_input(
        _input(ZERO_COMMAND, age=250_000_001),
        LIMITS,
        OPEN_GATE,
    )

    assert decision.kind is DecisionKind.STOP_READY
    assert decision.command == ZERO_COMMAND
    assert decision.should_publish
