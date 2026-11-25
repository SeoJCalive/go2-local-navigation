
"""Deterministic, ROS-independent Stage 11 fault timelines."""
from dataclasses import dataclass
from enum import Enum
from typing import Final


class FaultKind(str, Enum):
    """Closed set of fault classes declared by the Stage 11 oracle."""

    MALFORMED_LAYOUT = "malformed_layout"
    EMPTY_CLOUD = "empty_cloud"
    NAN_CLOUD = "nan_cloud"
    STALE_CLOUD = "stale_cloud"
    TF_LOSS = "tf_loss"
    ODOM_REGRESSION = "odom_regression"
    ODOM_JUMP = "odom_jump"
    ODOM_LOSS = "odom_loss"
    PROCESS_EXIT = "process_exit"
    LAUNCH_FAILURE = "launch_failure"


class FixturePhase(str, Enum):
    """An observation boundary emitted by one fixture attempt."""

    BASELINE = "baseline"
    SUPPRESSED = "suppressed"
    RECOVERED = "recovered"
    OWNED_CHILD_EXIT = "owned_child_exit"


@dataclass(frozen=True, slots=True)
class FixtureConfigurationError(Exception):
    """A requested deterministic fixture timeline violates its oracle."""

    reason_code: str

    def __str__(self) -> str:
        return self.reason_code


@dataclass(frozen=True, slots=True)
class FaultScenario:
    """Fixture inputs reduced from the shared fault oracle."""

    scenario_id: str
    fault_kind: FaultKind
    reason_code: str
    recovery_deadline_nanoseconds: int


@dataclass(frozen=True, slots=True)
class OutputCounts:
    """One boundary's emitted validated-cloud, scan, odom, and TF counts."""

    validated_cloud: int
    scan: int
    odom: int
    tf: int

    @property
    def total(self) -> int:
        return self.validated_cloud + self.scan + self.odom + self.tf

    def captured_streams(self) -> tuple[str, ...]:
        streams = (
            ("validated_cloud", self.validated_cloud),
            ("scan", self.scan),
            ("odom", self.odom),
            ("tf", self.tf),
        )
        return tuple(name for name, count in streams if count > 0)


@dataclass(frozen=True, slots=True)
class FixtureEvent:
    """A logical-clock observation emitted by the isolated fixture."""

    phase: FixturePhase
    clock_nanoseconds: int
    output_counts: OutputCounts
    reason_code: str | None
    child_exit_code: int | None


BASELINE_CLOCK_NANOSECONDS: Final = 10_000_000_000
FAULT_OFFSET_NANOSECONDS: Final = 100_000_000
DEFAULT_RECOVERY_DELAY_NANOSECONDS: Final = 400_000_000
ODOMETRY_LOSS_RECOVERY_DELAY_NANOSECONDS: Final = 700_000_000
PROCESS_EXIT_CODE: Final = 75
LAUNCH_FAILURE_EXIT_CODE: Final = 23
ALL_OUTPUTS: Final = OutputCounts(1, 1, 1, 1)
NO_OUTPUTS: Final = OutputCounts(0, 0, 0, 0)


def build_attempt_events(
    scenario: FaultScenario,
    *,
    restart_attempt: bool = False,
    recovery_delay_nanoseconds: int | None = None,
) -> tuple[FixtureEvent, ...]:
    """Build one owned-child attempt with a deterministic logical clock."""
    delay = (
        ODOMETRY_LOSS_RECOVERY_DELAY_NANOSECONDS
        if recovery_delay_nanoseconds is None
        and scenario.fault_kind is FaultKind.ODOM_LOSS
        else recovery_delay_nanoseconds or DEFAULT_RECOVERY_DELAY_NANOSECONDS
    )
    if scenario.recovery_deadline_nanoseconds <= 0:
        raise FixtureConfigurationError("invalid_recovery_deadline")
    if delay > scenario.recovery_deadline_nanoseconds:
        raise FixtureConfigurationError("recovery_deadline_exceeded")
    match scenario.fault_kind:
        case FaultKind.LAUNCH_FAILURE:
            return ()
        case FaultKind.PROCESS_EXIT if restart_attempt:
            return (
                FixtureEvent(
                    FixturePhase.RECOVERED,
                    BASELINE_CLOCK_NANOSECONDS
                    + FAULT_OFFSET_NANOSECONDS
                    + ODOMETRY_LOSS_RECOVERY_DELAY_NANOSECONDS,
                    ALL_OUTPUTS,
                    None,
                    None,
                ),
            )
        case FaultKind.PROCESS_EXIT:
            return _process_exit_events(scenario)
        case (
            FaultKind.MALFORMED_LAYOUT
            | FaultKind.EMPTY_CLOUD
            | FaultKind.NAN_CLOUD
            | FaultKind.STALE_CLOUD
            | FaultKind.TF_LOSS
            | FaultKind.ODOM_REGRESSION
            | FaultKind.ODOM_JUMP
            | FaultKind.ODOM_LOSS
        ):
            return _recovering_events(scenario, delay)
        case unreachable:
            raise AssertionError(f"unsupported fault kind: {unreachable}")


def expected_child_exit_code(scenario: FaultScenario) -> int:
    """Return the documented nonzero child exit code for special scenarios."""
    match scenario.fault_kind:
        case FaultKind.LAUNCH_FAILURE:
            return LAUNCH_FAILURE_EXIT_CODE
        case FaultKind.PROCESS_EXIT:
            return PROCESS_EXIT_CODE
        case (
            FaultKind.MALFORMED_LAYOUT
            | FaultKind.EMPTY_CLOUD
            | FaultKind.NAN_CLOUD
            | FaultKind.STALE_CLOUD
            | FaultKind.TF_LOSS
            | FaultKind.ODOM_REGRESSION
            | FaultKind.ODOM_JUMP
            | FaultKind.ODOM_LOSS
        ):
            return 0
        case unreachable:
            raise AssertionError(f"unsupported fault kind: {unreachable}")


def _process_exit_events(scenario: FaultScenario) -> tuple[FixtureEvent, ...]:
    suppressed_at = BASELINE_CLOCK_NANOSECONDS + FAULT_OFFSET_NANOSECONDS
    return (
        FixtureEvent(
            FixturePhase.BASELINE,
            BASELINE_CLOCK_NANOSECONDS,
            ALL_OUTPUTS,
            None,
            None,
        ),
        FixtureEvent(
            FixturePhase.SUPPRESSED,
            suppressed_at,
            NO_OUTPUTS,
            scenario.reason_code,
            None,
        ),
        FixtureEvent(
            FixturePhase.OWNED_CHILD_EXIT,
            suppressed_at,
            NO_OUTPUTS,
            scenario.reason_code,
            PROCESS_EXIT_CODE,
        ),
    )


def _recovering_events(
    scenario: FaultScenario,
    recovery_delay_nanoseconds: int,
) -> tuple[FixtureEvent, ...]:
    suppressed_at = BASELINE_CLOCK_NANOSECONDS + FAULT_OFFSET_NANOSECONDS
    recovered_at = suppressed_at + recovery_delay_nanoseconds
    return (
        FixtureEvent(
            FixturePhase.BASELINE,
            BASELINE_CLOCK_NANOSECONDS,
            ALL_OUTPUTS,
            None,
            None,
        ),
        FixtureEvent(
            FixturePhase.SUPPRESSED,
            suppressed_at,
            _suppressed_outputs(scenario.fault_kind),
            scenario.reason_code,
            None,
        ),
        FixtureEvent(FixturePhase.RECOVERED, recovered_at, ALL_OUTPUTS, None, None),
    )


def _suppressed_outputs(kind: FaultKind) -> OutputCounts:
    match kind:
        case (
            FaultKind.MALFORMED_LAYOUT
            | FaultKind.EMPTY_CLOUD
            | FaultKind.NAN_CLOUD
            | FaultKind.STALE_CLOUD
        ):
            return OutputCounts(0, 0, 1, 1)
        case FaultKind.TF_LOSS:
            return OutputCounts(1, 0, 1, 0)
        case FaultKind.ODOM_REGRESSION | FaultKind.ODOM_JUMP | FaultKind.ODOM_LOSS:
            return OutputCounts(1, 1, 0, 0)
        case FaultKind.PROCESS_EXIT | FaultKind.LAUNCH_FAILURE:
            return NO_OUTPUTS
        case unreachable:
            raise AssertionError(f"unsupported fault kind: {unreachable}")
