
"""Pure Stage 11 oracle plus a narrow runtime entry-point boundary."""
from dataclasses import dataclass
from typing import Final

from go2_validation.fault_fixture_model import (
    FaultKind,
    FaultScenario,
    FixtureEvent,
    FixturePhase,
    expected_child_exit_code,
)


REQUIRED_STREAMS: Final = ("validated_cloud", "scan", "odom", "tf")


@dataclass(frozen=True, slots=True)
class FaultExpectation:
    """Runner view of one shared Todo 3 fault-oracle row."""

    scenario_id: str
    fault_kind: FaultKind
    reason_code: str
    recovery_deadline_nanoseconds: int

    @classmethod
    def from_scenario(cls, scenario: FaultScenario) -> "FaultExpectation":
        """Convert a pure fixture scenario into a runner expectation."""
        return cls(
            scenario_id=scenario.scenario_id,
            fault_kind=scenario.fault_kind,
            reason_code=scenario.reason_code,
            recovery_deadline_nanoseconds=scenario.recovery_deadline_nanoseconds,
        )


@dataclass(frozen=True, slots=True)
class AttemptOutcome:
    """All observable values captured while the runner owned its child launch."""

    first_exit_code: int
    restart_exit_code: int | None
    events: tuple[FixtureEvent, ...]
    global_tf_owner_count: int
    residual_nodes: tuple[str, ...]
    residual_processes: tuple[str, ...]
    sport_request_publishers: int
    lowcmd_publishers: int
    output_enabled: bool
    physical_validation_approved: bool


@dataclass(frozen=True, slots=True)
class FaultAcceptanceResult:
    """One scenario's terminal status and captured recovery surface."""

    scenario_id: str
    passed: bool
    reason_code: str | None
    expected_child_failure: bool
    captured_streams: tuple[str, ...]
    recovery_elapsed_nanoseconds: int | None


def evaluate_fault_acceptance(
    expectation: FaultExpectation,
    outcome: AttemptOutcome,
) -> FaultAcceptanceResult:
    """Evaluate safety, child lifecycle, suppression, and recovery in order."""
    common_failure = _common_failure(outcome)
    if common_failure is not None:
        return _failed(expectation, common_failure)
    if expectation.fault_kind is FaultKind.LAUNCH_FAILURE:
        return _evaluate_launch_failure(expectation, outcome)
    if expectation.fault_kind is FaultKind.PROCESS_EXIT:
        suppressed = _event(outcome.events, FixturePhase.SUPPRESSED)
        if suppressed is None or suppressed.reason_code != expectation.reason_code:
            return _failed(expectation, "fault_reason_mismatch")
        return _evaluate_restart(expectation, outcome, suppressed)
    if outcome.first_exit_code != 0:
        return _failed(expectation, "unexpected_child_exit")
    suppressed = _event(outcome.events, FixturePhase.SUPPRESSED)
    if suppressed is None or suppressed.reason_code != expectation.reason_code:
        return _failed(expectation, "fault_reason_mismatch")
    if suppressed.output_counts.total == 4:
        return _failed(expectation, "suppression_missing")
    recovered = _event(outcome.events, FixturePhase.RECOVERED)
    return _evaluate_recovery(expectation, suppressed, recovered)


def _common_failure(outcome: AttemptOutcome) -> str | None:
    if outcome.global_tf_owner_count != 0:
        return "global_tf_owner_present"
    if outcome.residual_nodes or outcome.residual_processes:
        return "residual_graph_or_process"
    if outcome.sport_request_publishers != 0 or outcome.lowcmd_publishers != 0:
        return "command_publisher_present"
    if outcome.output_enabled or outcome.physical_validation_approved:
        return "motion_gate_open"
    return None


def _evaluate_launch_failure(
    expectation: FaultExpectation,
    outcome: AttemptOutcome,
) -> FaultAcceptanceResult:
    scenario = FaultScenario(
        expectation.scenario_id,
        expectation.fault_kind,
        expectation.reason_code,
        expectation.recovery_deadline_nanoseconds,
    )
    if outcome.first_exit_code != expected_child_exit_code(scenario):
        return _failed(expectation, "launch_failure_oracle_mismatch")
    return FaultAcceptanceResult(
        expectation.scenario_id,
        True,
        None,
        True,
        (),
        None,
    )


def _evaluate_restart(
    expectation: FaultExpectation,
    outcome: AttemptOutcome,
    suppressed: FixtureEvent,
) -> FaultAcceptanceResult:
    scenario = FaultScenario(
        expectation.scenario_id,
        expectation.fault_kind,
        expectation.reason_code,
        expectation.recovery_deadline_nanoseconds,
    )
    exited = _event(outcome.events, FixturePhase.OWNED_CHILD_EXIT)
    if exited is None or exited.child_exit_code != expected_child_exit_code(scenario):
        return _failed(expectation, "owned_process_exit_missing")
    if outcome.first_exit_code != expected_child_exit_code(scenario):
        return _failed(expectation, "owned_process_exit_oracle_mismatch")
    if outcome.restart_exit_code != 0:
        return _failed(expectation, "owned_process_restart_failed")
    return _evaluate_recovery(expectation, suppressed, _event(outcome.events, FixturePhase.RECOVERED))


def _evaluate_recovery(
    expectation: FaultExpectation,
    suppressed: FixtureEvent,
    recovered: FixtureEvent | None,
) -> FaultAcceptanceResult:
    if recovered is None or recovered.output_counts.captured_streams() != REQUIRED_STREAMS:
        return _failed(expectation, "recovery_output_missing")
    elapsed = recovered.clock_nanoseconds - suppressed.clock_nanoseconds
    if elapsed >= expectation.recovery_deadline_nanoseconds:
        return _failed(expectation, "recovery_deadline_exceeded")
    return FaultAcceptanceResult(
        expectation.scenario_id,
        True,
        None,
        False,
        REQUIRED_STREAMS,
        elapsed,
    )


def _event(
    events: tuple[FixtureEvent, ...],
    phase: FixturePhase,
) -> FixtureEvent | None:
    return next((event for event in events if event.phase is phase), None)


def _failed(expectation: FaultExpectation, reason_code: str) -> FaultAcceptanceResult:
    return FaultAcceptanceResult(
        expectation.scenario_id,
        False,
        reason_code,
        False,
        (),
        None,
    )


def main(args: list[str] | None = None) -> None:
    """Delegate ROS process ownership while this module keeps the pure verdict API."""
    from go2_validation.fault_acceptance_runtime import main as runtime_main

    runtime_main(args)
