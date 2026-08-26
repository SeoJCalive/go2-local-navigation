"""AGX `tegrastats`와 kernel event의 정지 soak 판정 계약을 검증한다."""

from bringup.preflight_resources import (
    CheckStatus,
    KernelObservation,
    assess_resources,
    parse_tegrastats,
)


def test_tegrastats_parser_summarizes_memory_cpu_and_temperature() -> None:
    # Given: 두 시점의 실제 Jetson tegrastats 형식 표본
    lines = (
        "RAM 3116/62828MB CPU [1%@729,2%@729,off] GR3D_FREQ 0% "
        "cpu@41.781C soc0@39.468C tj@41.781C",
        "RAM 3200/62828MB CPU [11%@729,0%@729,3%@729] GR3D_FREQ 2% "
        "cpu@44.000C soc0@40.000C tj@45.500C",
    )

    # When: resource summary를 계산한다.
    summary = parse_tegrastats(lines)

    # Then: 최대 사용량·CPU·온도가 구조화된다.
    assert summary.sample_count == 2
    assert summary.maximum_ram_used_mb == 3200
    assert summary.total_ram_mb == 62828
    assert summary.maximum_cpu_percent == 11.0
    assert summary.maximum_temperature_c == 45.5


def test_resource_assessment_fails_when_passive_trip_or_oom_is_observed() -> None:
    # Given: passive thermal trip을 넘은 표본과 OOM kernel event
    summary = parse_tegrastats(
        (
            "RAM 4000/62828MB CPU [20%@729] GR3D_FREQ 0% "
            "cpu@71.000C tj@70.500C",
        )
    )

    # When: AGX의 최저 passive trip 70 C와 kernel event를 함께 판정한다.
    checks = assess_resources(
        summary,
        passive_trip_c=70.0,
        kernel=KernelObservation(
            available=True,
            forced_failure_events=("Out of memory: Killed process 123",),
        ),
    )

    # Then: thermal throttle 경계와 OOM이 모두 강제 실패다.
    by_id = {check.check_id: check for check in checks}
    assert by_id["resources.thermal"].status is CheckStatus.FAIL
    assert by_id["resources.kernel"].status is CheckStatus.FAIL


def test_resource_assessment_warns_when_kernel_log_is_unavailable() -> None:
    # Given: 온도는 낮지만 kernel log를 읽을 수 없는 실행
    summary = parse_tegrastats(
        (
            "RAM 3200/62828MB CPU [5%@729] GR3D_FREQ 0% "
            "cpu@45.000C tj@46.000C",
        )
    )

    # When: 자원 판정을 수행한다.
    checks = assess_resources(
        summary,
        passive_trip_c=70.0,
        kernel=KernelObservation(
            available=False,
            forced_failure_events=(),
        ),
    )

    # Then: 확인 불가를 통과로 위장하지 않고 WARN으로 보존한다.
    kernel = next(check for check in checks if check.check_id == "resources.kernel")
    assert kernel.status is CheckStatus.WARN
