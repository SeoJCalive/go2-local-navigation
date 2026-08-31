"""합성 fixture의 scenario별 진행 정책과 rclpy node 상태 격리를 검증한다."""

import pytest
import rclpy
from go2_validation.shadow_fixture import ShadowFixturePlan, fixture_plan_for
from go2_validation.shadow_fixture_node import ShadowFixtureNode
from go2_validation.shadow_scenarios import ShadowTerminalStatus


def test_given_each_shadow_terminal_when_fixture_plan_built_then_owners_and_progress_are_explicit() -> None:
    # Given: six Domain 65 terminal outcomes
    expected = {
        "success": (ShadowTerminalStatus.SUCCEEDED, True),
        "cancel": (ShadowTerminalStatus.CANCELED, True),
        "blocked_goal": (ShadowTerminalStatus.ABORTED, True),
        "outside_map_goal": (ShadowTerminalStatus.ABORTED, True),
        "planner_failure": (ShadowTerminalStatus.ABORTED, True),
        "no_progress": (ShadowTerminalStatus.ABORTED, False),
    }

    # When: each fixture plan is selected.
    plans = {scenario_id: fixture_plan_for(scenario_id) for scenario_id in expected}

    # Then: fixture alone owns simulated time and both required TF edges.
    assert all(isinstance(plan, ShadowFixturePlan) for plan in plans.values())
    assert {
        scenario_id: (plan.expected_terminal, plan.integrates_shadow_velocity)
        for scenario_id, plan in plans.items()
    } == expected
    assert all(plan.clock_topic == "/clock" for plan in plans.values())
    assert all(plan.map_to_odom_owner == "fixture" for plan in plans.values())
    assert all(plan.odom_to_base_owner == "fixture" for plan in plans.values())
    assert all(plan.shadow_velocity_topic == "/go2_nav2/shadow_cmd_vel" for plan in plans.values())


def test_given_domain65_fixture_when_constructed_then_node_clock_is_not_shadowed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Given: loopback DDS에서 실제 rclpy Node를 생성할 수 있는 격리 환경
    monkeypatch.setenv("ROS_DOMAIN_ID", "65")
    monkeypatch.setenv(
        "CYCLONEDDS_URI",
        "<CycloneDDS><Domain><General><Interfaces>"
        '<NetworkInterface name="lo" multicast="false" />'
        "</Interfaces></General></Domain></CycloneDDS>",
    )
    rclpy.init()

    try:
        # When: synthetic fixture가 publisher와 timer를 함께 생성한다.
        node = ShadowFixtureNode()
        try:
            # Then: Node 내부 clock을 publisher로 덮어쓰지 않고 timer 생성이 완료된다.
            assert node.get_name() == "synthetic_navigation_fixture"
        finally:
            node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
