from math import isnan
from pathlib import Path
import struct
from types import SimpleNamespace

import pytest

from go2_validation.fault_fixture_model import (
    FaultKind,
    FaultScenario,
    FixtureConfigurationError,
    FixtureEvent,
    FixturePhase,
    OutputCounts,
    build_attempt_events,
)

pytest.importorskip("rclpy")

from go2_validation.fault_fixture_node import FaultFixtureNode, TICK_NANOSECONDS


class _CloudRecorder:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def test_given_each_fault_when_fixture_builds_then_suppression_and_recovery_are_deterministic() -> None:
    # Given: every software fault except the intentionally failed launch.
    scenarios = tuple(
        FaultScenario(
            scenario_id=f"fixture-{kind.value}",
            fault_kind=kind,
            reason_code=f"REASON_{kind.value.upper()}",
            recovery_deadline_nanoseconds=1_000_000_000,
        )
        for kind in FaultKind
        if kind not in {FaultKind.LAUNCH_FAILURE, FaultKind.PROCESS_EXIT}
    )

    # When: each deterministic fixture timeline is generated.
    timelines = tuple(build_attempt_events(scenario) for scenario in scenarios)

    # Then: it has an ordered fault boundary, preserves `/clock`, and recovers.
    for scenario, events in zip(scenarios, timelines, strict=True):
        suppressed = next(event for event in events if event.phase is FixturePhase.SUPPRESSED)
        recovered = next(event for event in events if event.phase is FixturePhase.RECOVERED)
        assert suppressed.reason_code == scenario.reason_code
        assert suppressed.output_counts.total < 4
        assert recovered.output_counts.total == 4
        assert recovered.clock_nanoseconds - suppressed.clock_nanoseconds <= (
            scenario.recovery_deadline_nanoseconds
        )


def test_given_process_exit_when_second_owned_attempt_runs_then_only_restart_recovers() -> None:
    # Given: the owned process-exit oracle.
    scenario = FaultScenario(
        scenario_id="owned-process-exit",
        fault_kind=FaultKind.PROCESS_EXIT,
        reason_code="OWNED_PROCESS_EXIT",
        recovery_deadline_nanoseconds=1_000_000_000,
    )

    # When: initial and restart attempts are generated independently.
    initial = build_attempt_events(scenario, restart_attempt=False)
    restarted = build_attempt_events(scenario, restart_attempt=True)

    # Then: only the restarted child produces a recovery sample.
    assert initial[-1].phase is FixturePhase.OWNED_CHILD_EXIT
    assert all(event.phase is not FixturePhase.RECOVERED for event in initial)
    assert restarted[-1].phase is FixturePhase.RECOVERED


def test_given_late_recovery_when_fixture_builds_then_the_invalid_timeline_is_rejected() -> None:
    # Given: a one-second recovery contract.
    scenario = FaultScenario(
        scenario_id="late-recovery",
        fault_kind=FaultKind.STALE_CLOUD,
        reason_code="STALE_CLOUD",
        recovery_deadline_nanoseconds=1_000_000_000,
    )

    # When / Then: a later requested recovery is refused at the boundary.
    try:
        build_attempt_events(scenario, recovery_delay_nanoseconds=1_000_000_001)
    except FixtureConfigurationError as error:
        assert error.reason_code == "recovery_deadline_exceeded"
    else:
        raise AssertionError("late recovery timeline was accepted")


def test_given_owned_fault_launch_when_read_then_parent_controls_teardown_and_sim_time() -> None:
    # Given: the source launch and shared odometry adapter launch.
    package_root = Path(__file__).parents[1]
    fault_launch = (
        package_root / "launch/go2_fault_acceptance.launch.py"
    ).read_text(encoding="utf-8")
    odometry_launch = (
        package_root.parent / "bringup/launch/go2_odometry_adapter.launch.py"
    ).read_text(encoding="utf-8")

    # When / Then: fixture exit is observed by the parent and all data uses ROS time.
    assert "OnProcessExit" not in fault_launch
    assert 'LaunchConfiguration("use_sim_time")' in fault_launch
    assert 'DeclareLaunchArgument("use_sim_time", default_value="true")' in fault_launch
    assert 'DeclareLaunchArgument("use_sim_time"' in odometry_launch


def test_given_nan_cloud_fault_when_published_then_fixture_serializes_nan() -> None:
    # Given: the runtime fixture is at the NaN-cloud suppression boundary.
    recorder = _CloudRecorder()
    fixture = SimpleNamespace(
        _scenario=SimpleNamespace(fault_kind=FaultKind.NAN_CLOUD),
        _cloud_publisher=recorder,
        _publish_odometry=lambda *_args: None,
    )
    event = FixtureEvent(
        FixturePhase.SUPPRESSED,
        10_100_000_000,
        OutputCounts(0, 0, 1, 1),
        "NAN_CLOUD",
        None,
    )

    # When: the real fixture branch creates its malformed numeric sample.
    FaultFixtureNode._publish_fault_input(fixture, event)

    # Then: a message is emitted with a NaN value instead of crashing the child.
    assert len(recorder.messages) == 1
    assert isnan(struct.unpack("<f", bytes(recorder.messages[0].data[:4]))[0])


def test_given_odom_fault_recovery_when_ticked_then_seed_precedes_terminal_marker() -> None:
    # Given: a recovered odometry event whose adapter requires two callbacks.
    event = FixtureEvent(
        FixturePhase.RECOVERED,
        10_500_000_000,
        OutputCounts(1, 1, 1, 1),
        None,
        None,
    )
    odometry_stamps: list[int] = []
    clock_stamps: list[int] = []
    cloud_recorder = _CloudRecorder()
    event_recorder = _CloudRecorder()
    fixture = SimpleNamespace(
        _done=False,
        _events=(event,),
        _event_index=0,
        _current_clock_nanoseconds=event.clock_nanoseconds - TICK_NANOSECONDS,
        _scenario=SimpleNamespace(fault_kind=FaultKind.ODOM_JUMP),
        _exit_code=0,
        _cloud_publisher=cloud_recorder,
        _event_publisher=event_recorder,
        _publish_clock=clock_stamps.append,
        _publish_odometry=lambda stamp, *_args: odometry_stamps.append(stamp),
    )
    fixture._publish_inputs = lambda current: FaultFixtureNode._publish_inputs(
        fixture,
        current,
    )

    # When: one pre-terminal tick and the terminal tick are processed.
    FaultFixtureNode._tick(fixture)
    first_tick = (
        tuple(odometry_stamps),
        fixture._done,
        len(event_recorder.messages),
    )
    FaultFixtureNode._tick(fixture)

    # Then: the seed is delivered first and only the second sample is terminal.
    assert first_tick == ((event.clock_nanoseconds - 1,), False, 0)
    assert odometry_stamps == [
        event.clock_nanoseconds - 1,
        event.clock_nanoseconds,
    ]
    assert clock_stamps == [event.clock_nanoseconds - 1, event.clock_nanoseconds]
    assert fixture._done
    assert len(event_recorder.messages) == 1
