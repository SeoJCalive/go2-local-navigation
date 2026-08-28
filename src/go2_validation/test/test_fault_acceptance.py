from bringup.fault_contract import FaultScenario as ConfiguredFaultScenario
from pathlib import Path
import re

from go2_validation.fault_acceptance_runner import (
    AttemptOutcome,
    FaultExpectation,
    evaluate_fault_acceptance,
)
from go2_validation.fault_fixture_model import FaultKind, FaultScenario, build_attempt_events
from go2_validation.fault_runtime_execution import fault_launch_command


def test_given_recovered_cloud_fault_when_evaluated_then_capture_and_safety_oracle_pass() -> None:
    # Given: an on-time stale-cloud recovery fixture and matching oracle.
    scenario = FaultScenario(
        scenario_id="stale-cloud",
        fault_kind=FaultKind.STALE_CLOUD,
        reason_code="STALE_CLOUD",
        recovery_deadline_nanoseconds=1_000_000_000,
    )
    expectation = FaultExpectation.from_scenario(scenario)
    outcome = AttemptOutcome(
        first_exit_code=0,
        restart_exit_code=None,
        events=build_attempt_events(scenario),
        global_tf_owner_count=0,
        residual_nodes=(),
        residual_processes=(),
        sport_request_publishers=0,
        lowcmd_publishers=0,
        output_enabled=False,
        physical_validation_approved=False,
    )

    # When: the runner evaluates the captured observations.
    result = evaluate_fault_acceptance(expectation, outcome)

    # Then: suppression, recovery, safety gates, and teardown all pass.
    assert result.passed
    assert result.reason_code is None
    assert result.captured_streams == ("validated_cloud", "scan", "odom", "tf")


def test_given_late_recovery_when_evaluated_then_exact_deadline_reason_fails() -> None:
    # Given: a valid timeline whose recovery arrives one nanosecond late.
    scenario = FaultScenario(
        scenario_id="late-cloud",
        fault_kind=FaultKind.EMPTY_CLOUD,
        reason_code="EMPTY_CLOUD",
        recovery_deadline_nanoseconds=1_000_000_000,
    )
    outcome = AttemptOutcome(
        first_exit_code=0,
        restart_exit_code=None,
        events=build_attempt_events(scenario, recovery_delay_nanoseconds=1_000_000_000),
        global_tf_owner_count=0,
        residual_nodes=(),
        residual_processes=(),
        sport_request_publishers=0,
        lowcmd_publishers=0,
        output_enabled=False,
        physical_validation_approved=False,
    )

    # When: the oracle is stricter than the fixture construction boundary.
    result = evaluate_fault_acceptance(FaultExpectation.from_scenario(scenario), outcome)

    # Then: equal-to-deadline recovery cannot be promoted.
    assert not result.passed
    assert result.reason_code == "recovery_deadline_exceeded"


def test_given_expected_child_launch_failure_when_evaluated_then_runner_succeeds() -> None:
    # Given: the launch-failure scenario and its documented child exit code.
    scenario = FaultScenario(
        scenario_id="expected-launch-failure",
        fault_kind=FaultKind.LAUNCH_FAILURE,
        reason_code="EXPECTED_LAUNCH_FAILURE",
        recovery_deadline_nanoseconds=1_000_000_000,
    )
    outcome = AttemptOutcome(
        first_exit_code=23,
        restart_exit_code=None,
        events=build_attempt_events(scenario),
        global_tf_owner_count=0,
        residual_nodes=(),
        residual_processes=(),
        sport_request_publishers=0,
        lowcmd_publishers=0,
        output_enabled=False,
        physical_validation_approved=False,
    )

    # When: the expected failed child is checked with clean teardown.
    result = evaluate_fault_acceptance(FaultExpectation.from_scenario(scenario), outcome)

    # Then: the parent acceptance runner remains successful.
    assert result.passed
    assert result.expected_child_failure


def test_given_duplicate_tf_owner_when_evaluated_then_isolated_fault_run_fails() -> None:
    # Given: a normal TF-loss run with an unexpected global owner.
    scenario = FaultScenario(
        scenario_id="duplicate-tf",
        fault_kind=FaultKind.TF_LOSS,
        reason_code="TF_UNAVAILABLE",
        recovery_deadline_nanoseconds=1_000_000_000,
    )
    outcome = AttemptOutcome(
        first_exit_code=0,
        restart_exit_code=None,
        events=build_attempt_events(scenario),
        global_tf_owner_count=1,
        residual_nodes=(),
        residual_processes=(),
        sport_request_publishers=0,
        lowcmd_publishers=0,
        output_enabled=False,
        physical_validation_approved=False,
    )

    # When: the offline-domain owner cardinality is checked.
    result = evaluate_fault_acceptance(FaultExpectation.from_scenario(scenario), outcome)

    # Then: it fails before a recovery claim is accepted.
    assert not result.passed
    assert result.reason_code == "global_tf_owner_present"


def test_given_fault_scenario_when_launch_command_built_then_clock_and_restart_are_explicit() -> None:
    # Given: one configured process-exit scenario.
    scenario = ConfiguredFaultScenario(
        scenario_id="process-exit",
        fault_kind="process_exit",
        suppressed_outputs=("/scan",),
        reason_code="PROCESS_EXIT",
        recovery_trigger="process_restart",
        recovery_deadline_seconds=10,
        terminal_status="recovered",
    )

    # When: the owned restart command is built.
    command = fault_launch_command(scenario, restart_attempt=True)

    # Then: it starts only the isolated fault launch with explicit oracle values.
    assert command[:4] == ("ros2", "launch", "go2_validation", "go2_fault_acceptance.launch.py")
    assert "fault_kind:=process_exit" in command
    assert "restart_attempt:=true" in command
    assert "use_sim_time:=true" in command
    assert "execution_mode:=onboard" in command
    assert "continuity_profile:=replay_enforce" in command


def test_given_fault_launch_when_read_then_runtime_profile_arguments_are_consumed() -> None:
    launch_source = (
        Path(__file__).parents[1] / "launch/go2_fault_acceptance.launch.py"
    ).read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("execution_mode", default_value="onboard")' in launch_source
    assert re.search(
        r'DeclareLaunchArgument\(\s*"continuity_profile",\s*'
        r'default_value="replay_enforce"',
        launch_source,
    )
    assert 'LaunchConfiguration("execution_mode")' in launch_source
    assert 'LaunchConfiguration("continuity_profile")' in launch_source
