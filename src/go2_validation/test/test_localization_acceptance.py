"""Domain 64 저장 지도 localization의 순수 합격 판정을 검증한다."""

from dataclasses import replace

from go2_validation.localization_acceptance import (
    LocalizationObservation,
    LocalizationStatus,
    assess_localization,
)


def _passing_observation() -> LocalizationObservation:
    return LocalizationObservation(
        scan_count=20,
        odom_count=100,
        map_count=1,
        map_frames=("map",),
        map_has_cells=True,
        pose_count=2,
        finite_pose_count=2,
        lifecycle_states=(
            ("amcl", "active"),
            ("map_server", "active"),
        ),
        global_edges=(("map", "odom"),),
        global_owner_nodes=("/amcl",),
        clock_publisher_max=1,
        clock_progressed=True,
        command_publisher_max=0,
        control_node_max=0,
        player_exit_code=0,
        launch_exit_code=0,
        residual_nodes=(),
        residual_processes=(),
        teardown_clock_publishers=0,
        teardown_global_owner_nodes=(),
    )


def test_given_complete_localization_when_assessed_then_it_passes() -> None:
    # Given: map·pose·lifecycle·TF·안전·teardown이 모두 충족된 관찰
    observation = _passing_observation()

    # When: Domain 64 합격 판정을 수행한다.
    result = assess_localization(observation)

    # Then: 실패 check 없이 replay 범위가 통과한다.
    assert result.status is LocalizationStatus.PASSED
    assert result.failed_checks == ()


def test_given_duplicate_global_owner_when_assessed_then_it_fails() -> None:
    # Given: AMCL 외의 map→odom owner가 함께 관찰된 상태
    observation = replace(
        _passing_observation(),
        global_owner_nodes=("/amcl", "/slam_toolbox"),
    )

    # When: Domain 64 합격 판정을 수행한다.
    result = assess_localization(observation)

    # Then: 단일 AMCL owner 계약이 실패한다.
    assert result.status is LocalizationStatus.FAILED
    assert "global_owner" in result.failed_checks


def test_given_command_publisher_when_assessed_then_it_fails() -> None:
    # Given: localization과 무관한 실제 command publisher가 노출된 상태
    observation = replace(
        _passing_observation(),
        command_publisher_max=1,
    )

    # When: Domain 64 합격 판정을 수행한다.
    result = assess_localization(observation)

    # Then: software-only 안전 경계가 실패한다.
    assert result.status is LocalizationStatus.FAILED
    assert "command_boundary" in result.failed_checks


def test_given_residual_process_when_assessed_then_it_fails() -> None:
    # Given: launch 종료 뒤 소유 process가 남은 상태
    observation = replace(
        _passing_observation(),
        residual_processes=("amcl",),
    )

    # When: Domain 64 합격 판정을 수행한다.
    result = assess_localization(observation)

    # Then: clean teardown 계약이 실패한다.
    assert result.status is LocalizationStatus.FAILED
    assert "residual_processes" in result.failed_checks
