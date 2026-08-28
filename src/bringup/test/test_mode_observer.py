"""실행 mode별 환경 관찰 계약을 검증한다."""

from bringup.mode_observer import (
    ExecutionMode,
    ModeEnvironment,
    assess_mode_environment,
)
from bringup.preflight_types import CheckStatus


def test_offline_mode_does_not_require_live_domain_or_interface() -> None:
    # Given: domain 63의 mapping 실행에 AGX live 환경값이 없는 관찰값
    environment = ModeEnvironment(
        rmw_implementation="rmw_cyclonedds_cpp",
        ros_domain_id="63",
        go2_interface="",
        cyclonedds_uri="",
    )

    # When: offline mapping mode 환경을 판정한다.
    check = assess_mode_environment(ExecutionMode.MAPPING, environment)

    # Then: domain 0이나 eno1 없이도 통과한다.
    assert check.status is CheckStatus.PASS


def test_live_mode_requires_domain_zero_and_eno1() -> None:
    # Given: live가 아닌 domain과 interface를 가진 관찰값
    environment = ModeEnvironment(
        rmw_implementation="rmw_cyclonedds_cpp",
        ros_domain_id="65",
        go2_interface="lo",
        cyclonedds_uri='<NetworkInterface name="lo" />',
    )

    # When: live shadow mode 환경을 판정한다.
    check = assess_mode_environment(ExecutionMode.LIVE_SHADOW, environment)

    # Then: domain 0과 eno1 요구사항 위반이 실패로 남는다.
    assert check.status is CheckStatus.FAIL


def test_live_mode_accepts_agx_runtime_environment() -> None:
    # Given: AGX live observer가 수집한 domain 0과 eno1 환경값
    environment = ModeEnvironment(
        rmw_implementation="rmw_cyclonedds_cpp",
        ros_domain_id="0",
        go2_interface="eno1",
        cyclonedds_uri='<NetworkInterface name="eno1" />',
    )

    # When: live shadow mode 환경을 판정한다.
    check = assess_mode_environment(ExecutionMode.LIVE_SHADOW, environment)

    # Then: live 전용 요구사항을 충족한다.
    assert check.status is CheckStatus.PASS
