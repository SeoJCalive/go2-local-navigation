"""Domain 65 관찰의 terminal·안전·teardown 합격 판정을 검증한다."""

from dataclasses import replace

from go2_validation.shadow_scenarios import ShadowTerminalStatus
from go2_validation.shadow_verdict import (
    ShadowObservation,
    ShadowStatus,
    assess_shadow_scenario,
)


def _observation(status: ShadowTerminalStatus, *, path: bool, candidate: bool) -> ShadowObservation:
    return ShadowObservation(
        action_terminal=status,
        path_count=int(path),
        shadow_candidate_count=int(candidate),
        feedback_count=1,
        lifecycle_states=(
            ("behavior_server", "active"),
            ("bt_navigator", "active"),
            ("controller_server", "active"),
            ("map_server", "active"),
            ("planner_server", "active"),
        ),
        global_costmap_count=1,
        local_costmap_count=1,
        clock_publisher_max=1,
        clock_progressed=True,
        map_to_odom_owners=("/synthetic_navigation_fixture",),
        odom_to_base_owners=("/synthetic_navigation_fixture",),
        physical_command_publisher_max=0,
        control_node_max=0,
        unitree_node_max=0,
        fixture_exit_code=0,
        launch_exit_code=0,
        residual_nodes=(),
        residual_processes=(),
        teardown_clock_publishers=0,
        teardown_tf_owners=(),
    )


def test_given_each_expected_terminal_when_assessed_then_six_shadow_scenarios_pass() -> None:
    # Given: observed terminal/path/candidate surfaces matching every manifest scenario
    observations = {
        "success": _observation(ShadowTerminalStatus.SUCCEEDED, path=True, candidate=True),
        "cancel": _observation(ShadowTerminalStatus.CANCELED, path=True, candidate=True),
        "blocked_goal": _observation(ShadowTerminalStatus.ABORTED, path=False, candidate=False),
        "outside_map_goal": _observation(ShadowTerminalStatus.ABORTED, path=False, candidate=False),
        "planner_failure": _observation(ShadowTerminalStatus.ABORTED, path=False, candidate=False),
        "no_progress": _observation(ShadowTerminalStatus.ABORTED, path=True, candidate=True),
    }

    # When: the Domain 65 oracle evaluates each named scenario.
    results = {
        scenario_id: assess_shadow_scenario(scenario_id, observation)
        for scenario_id, observation in observations.items()
    }

    # Then: all intended terminals pass their own observable contract.
    assert all(result.status is ShadowStatus.PASSED for result in results.values())
    assert all(result.failed_checks == () for result in results.values())


def test_given_physical_command_publisher_when_assessed_then_non_actuating_boundary_fails() -> None:
    # Given: otherwise valid success observation with an unsafe physical command publisher
    observation = replace(
        _observation(ShadowTerminalStatus.SUCCEEDED, path=True, candidate=True),
        physical_command_publisher_max=1,
    )

    # When: its verdict is calculated.
    result = assess_shadow_scenario("success", observation)

    # Then: the terminal result cannot mask a broken non-actuating boundary.
    assert result.status is ShadowStatus.FAILED
    assert "physical_command_boundary" in result.failed_checks


def test_given_control_node_when_assessed_then_non_actuating_boundary_fails() -> None:
    # Given: an otherwise valid observation with a Go2 control node present
    observation = replace(
        _observation(ShadowTerminalStatus.SUCCEEDED, path=True, candidate=True),
        control_node_max=1,
    )

    # When: the Domain 65 verdict is calculated.
    result = assess_shadow_scenario("success", observation)

    # Then: node ownership fails even when physical topics are still quiet.
    assert "control_node_boundary" in result.failed_checks


def test_given_immediate_planner_abort_when_feedback_is_absent_then_terminal_can_pass() -> None:
    # Given: planner failure가 path와 feedback을 만들기 전에 명시적으로 abort한 관찰
    observation = replace(
        _observation(ShadowTerminalStatus.ABORTED, path=False, candidate=False),
        feedback_count=0,
    )

    # When: planner failure oracle을 판정한다.
    result = assess_shadow_scenario("planner_failure", observation)

    # Then: action terminal·출력 부재가 맞으면 feedback 부재만으로 실패하지 않는다.
    assert result.status is ShadowStatus.PASSED


def test_given_cancel_without_feedback_when_assessed_then_cancel_gate_fails() -> None:
    # Given: cancel terminal은 반환됐지만 cancel 전 feedback 근거가 없는 관찰
    observation = replace(
        _observation(ShadowTerminalStatus.CANCELED, path=True, candidate=True),
        feedback_count=0,
    )

    # When: cancel oracle을 판정한다.
    result = assess_shadow_scenario("cancel", observation)

    # Then: accepted 직후 취소한 실행은 합격할 수 없다.
    assert "feedback" in result.failed_checks
