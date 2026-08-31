"""Navfn grid goal 변환, child argv와 feedback 기반 cancel 경계를 검증한다."""

from pathlib import Path

from go2_validation.shadow_action_runner import (
    _goal_pose,
    cancel_is_permitted,
    shadow_fixture_command,
    shadow_launch_command,
)
from go2_validation.shadow_scenarios import shadow_scenario

PACKAGE_ROOT = Path(__file__).parents[1]


def test_given_manifest_grid_goal_when_pose_built_then_navfn_index_is_preserved() -> None:
    # Given: an occupied grid cell selected by the blocked-goal scenario.
    scenario = shadow_scenario(
        PACKAGE_ROOT / "config/shadow_scenarios.yaml",
        "blocked_goal",
    )

    # When: the synthetic NavigateToPose goal is built.
    goal = _goal_pose(scenario)
    normalized = (
        (goal.pose.position.x + 3.0) / 0.5,
        (goal.pose.position.y + 3.0) / 0.5,
    )

    # Then: Navfn's round-based conversion receives the manifest indices exactly.
    assert normalized == (
        float(scenario.goal_cell.x),
        float(scenario.goal_cell.y),
    )


def test_given_manifest_scenario_when_shadow_process_commands_are_built_then_launch_and_fixture_stay_isolated() -> None:
    # Given: the success scenario and its synthetic map asset
    scenario = shadow_scenario(PACKAGE_ROOT / "config/shadow_scenarios.yaml", "success")
    map_path = Path("maps/shadow_open.yaml")

    # When: the runner builds owned child process argv.
    launch = shadow_launch_command(map_path)
    fixture = shadow_fixture_command(scenario)

    # Then: Nav2 only receives the map while fixture receives only its scenario selector.
    assert launch == (
        "ros2",
        "launch",
        "go2_nav2",
        "go2_nav2_shadow.launch.py",
        f"map:={map_path}",
    )
    assert fixture[:4] == (
        "ros2",
        "run",
        "go2_validation",
        "shadow_fixture",
    )
    assert fixture[-2:] == ("-p", "scenario_id:=success")
    assert "/api/sport/request" not in launch + fixture
    assert "/lowcmd" not in launch + fixture


def test_given_accepted_cancel_goal_when_feedback_path_or_candidate_is_missing_then_cancel_is_deferred() -> None:
    # Given: accepted cancel goals at each incomplete observation boundary
    incomplete = ((False, True, True), (True, False, True), (True, True, False))

    # When: the action layer evaluates cancel eligibility.
    permissions = tuple(cancel_is_permitted(*surface) for surface in incomplete)

    # Then: cancellation remains blocked until feedback, path, and candidate all exist.
    assert permissions == (False, False, False)
    assert cancel_is_permitted(True, True, True) is True
