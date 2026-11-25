"""Domain 0 live localization·no-goal Nav2의 순수 판정을 검증한다."""

from dataclasses import replace

from go2_validation.live_navigation_acceptance import (
    LiveNavigationObservation,
    LiveNavigationStatus,
    assess_live_navigation,
)


def _passing_observation() -> LiveNavigationObservation:
    return LiveNavigationObservation(
        scan_count=900,
        odom_count=9000,
        map_count=1,
        map_frames=("map",),
        map_has_cells=True,
        pose_count=10,
        finite_pose_count=10,
        global_costmap_count=10,
        local_costmap_count=10,
        lifecycle_states=(
            ("amcl", "active"),
            ("behavior_server", "active"),
            ("bt_navigator", "active"),
            ("controller_server", "active"),
            ("map_server", "active"),
            ("planner_server", "active"),
        ),
        global_edges=(("map", "odom"),),
        global_owner_nodes=("/amcl",),
        plan_count=0,
        nonempty_goal_status_count=0,
        inert_velocity_count=0,
        clock_publisher_max=0,
        sport_total_publisher_max=10,
        lowcmd_total_publisher_max=1,
        sport_ros_publisher_max=0,
        lowcmd_ros_publisher_max=0,
        cmd_vel_publisher_max=0,
        control_node_max=0,
        unitree_node_max=0,
        launch_exit_code=0,
        residual_nodes=(),
        residual_processes=(),
        teardown_global_owner_nodes=(),
    )


def test_given_complete_live_observation_when_assessed_then_it_passes() -> None:
    # Given: 실제 stream·AMCL·Nav2 lifecycle과 닫힌 command 경계
    observation = _passing_observation()

    # When: 최종 고정 전 no-goal 계약을 판정한다.
    result = assess_live_navigation(observation)

    # Then: bare DDS command endpoint는 허용하되 ROS command와 goal은 없어야 한다.
    assert result.status is LiveNavigationStatus.PASSED
    assert result.failed_checks == ()


def test_given_goal_or_inert_velocity_when_assessed_then_no_goal_boundary_fails() -> None:
    # Given: observer가 보내지 않은 goal status와 inert velocity가 관찰된 상태
    observation = replace(
        _passing_observation(),
        nonempty_goal_status_count=1,
        inert_velocity_count=1,
    )

    # When: no-goal 계약을 판정한다.
    result = assess_live_navigation(observation)

    # Then: 물리 topic이 아니어도 no-goal shadow 범위 이탈로 실패한다.
    assert result.status is LiveNavigationStatus.FAILED
    assert "no_goal_output" in result.failed_checks


def test_given_ros_command_publisher_when_assessed_then_safety_boundary_fails() -> None:
    # Given: Go2 bare endpoint가 아니라 ROS node가 command topic을 소유한 상태
    observation = replace(_passing_observation(), sport_ros_publisher_max=1)

    # When: command 경계를 판정한다.
    result = assess_live_navigation(observation)

    # Then: 실제 command payload가 없어도 publisher 존재만으로 실패한다.
    assert result.status is LiveNavigationStatus.FAILED
    assert "physical_command_boundary" in result.failed_checks


def test_given_wrong_global_edge_when_assessed_then_tf_contract_fails() -> None:
    # Given: AMCL owner가 있지만 canonical map→odom이 아닌 edge
    observation = replace(
        _passing_observation(),
        global_edges=(("go2_shadow_map", "odom"),),
    )

    # When: global TF 계약을 판정한다.
    result = assess_live_navigation(observation)

    # Then: owner 이름과 별개로 edge 불일치를 실패로 남긴다.
    assert result.status is LiveNavigationStatus.FAILED
    assert "global_edge" in result.failed_checks


def test_given_nonactive_lifecycle_when_assessed_then_runtime_fails() -> None:
    # Given: planner lifecycle만 inactive인 live graph
    observation = replace(
        _passing_observation(),
        lifecycle_states=(
            ("amcl", "active"),
            ("behavior_server", "active"),
            ("bt_navigator", "active"),
            ("controller_server", "active"),
            ("map_server", "active"),
            ("planner_server", "inactive"),
        ),
    )

    # When: runtime 계약을 판정한다.
    result = assess_live_navigation(observation)

    # Then: 전체 graph가 준비됐다고 보지 않는다.
    assert result.status is LiveNavigationStatus.FAILED
    assert "lifecycle" in result.failed_checks
