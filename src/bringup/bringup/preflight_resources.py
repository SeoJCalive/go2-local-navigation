"""Jetson `tegrastats`와 kernel event를 soak 자원 판정으로 변환한다."""

from dataclasses import dataclass
import re
from typing import Final

from bringup.preflight_types import CheckResult, CheckStatus


RAM_PATTERN: Final = re.compile(r"RAM\s+(\d+)/(\d+)MB")
CPU_BLOCK_PATTERN: Final = re.compile(r"CPU\s+\[([^]]+)]")
CPU_VALUE_PATTERN: Final = re.compile(r"(\d+(?:\.\d+)?)%@")
TEMPERATURE_PATTERN: Final = re.compile(
    r"([A-Za-z0-9_-]+)@(\d+(?:\.\d+)?)C"
)


@dataclass(frozen=True, slots=True)
class ResourceSummary:
    """한 실행의 고정 크기 Jetson 자원 통계다."""

    sample_count: int
    maximum_ram_used_mb: int
    total_ram_mb: int
    maximum_cpu_percent: float
    maximum_temperature_c: float


@dataclass(frozen=True, slots=True)
class KernelObservation:
    """실행 구간 kernel log 접근 여부와 강제 실패 event다."""

    available: bool
    forced_failure_events: tuple[str, ...]


def parse_tegrastats(lines: tuple[str, ...]) -> ResourceSummary:
    """`tegrastats` 원문에서 최대 RAM·CPU·온도만 추출한다."""
    sample_count = 0
    maximum_ram_used_mb = 0
    total_ram_mb = 0
    maximum_cpu_percent = 0.0
    maximum_temperature_c = 0.0
    for line in lines:
        ram_match = RAM_PATTERN.search(line)
        if ram_match is None:
            continue
        sample_count += 1
        maximum_ram_used_mb = max(
            maximum_ram_used_mb,
            int(ram_match.group(1)),
        )
        total_ram_mb = max(total_ram_mb, int(ram_match.group(2)))
        cpu_match = CPU_BLOCK_PATTERN.search(line)
        if cpu_match is not None:
            cpu_values = tuple(
                float(value)
                for value in CPU_VALUE_PATTERN.findall(cpu_match.group(1))
            )
            if cpu_values:
                maximum_cpu_percent = max(
                    maximum_cpu_percent,
                    max(cpu_values),
                )
        temperatures = tuple(
            float(value) for _, value in TEMPERATURE_PATTERN.findall(line)
        )
        if temperatures:
            maximum_temperature_c = max(
                maximum_temperature_c,
                max(temperatures),
            )
    return ResourceSummary(
        sample_count=sample_count,
        maximum_ram_used_mb=maximum_ram_used_mb,
        total_ram_mb=total_ram_mb,
        maximum_cpu_percent=maximum_cpu_percent,
        maximum_temperature_c=maximum_temperature_c,
    )


def assess_resources(
    summary: ResourceSummary,
    passive_trip_c: float,
    kernel: KernelObservation,
) -> tuple[CheckResult, ...]:
    """telemetry, memory, thermal trip과 OOM/throttle event를 판정한다."""
    telemetry_status = (
        CheckStatus.PASS if summary.sample_count > 0 else CheckStatus.FAIL
    )
    memory_fraction = 0.0
    if summary.total_ram_mb > 0:
        memory_fraction = (
            summary.maximum_ram_used_mb / summary.total_ram_mb
        )
    memory_status = (
        CheckStatus.WARN if memory_fraction >= 0.90 else CheckStatus.PASS
    )
    thermal_status = (
        CheckStatus.FAIL
        if summary.maximum_temperature_c >= passive_trip_c
        else CheckStatus.PASS
    )
    if not kernel.available:
        kernel_status = CheckStatus.WARN
    elif kernel.forced_failure_events:
        kernel_status = CheckStatus.FAIL
    else:
        kernel_status = CheckStatus.PASS
    return (
        CheckResult(
            check_id="resources.telemetry",
            status=telemetry_status,
            detail=f"tegrastats_samples={summary.sample_count}",
        ),
        CheckResult(
            check_id="resources.memory",
            status=memory_status,
            detail=(
                f"maximum_ram_used_mb={summary.maximum_ram_used_mb}; "
                f"total_ram_mb={summary.total_ram_mb}; "
                f"maximum_fraction={memory_fraction:.6f}"
            ),
        ),
        CheckResult(
            check_id="resources.thermal",
            status=thermal_status,
            detail=(
                f"maximum_temperature_c={summary.maximum_temperature_c:.3f}; "
                f"passive_trip_c={passive_trip_c:.3f}"
            ),
        ),
        CheckResult(
            check_id="resources.kernel",
            status=kernel_status,
            detail=(
                f"kernel_log_available={kernel.available}; "
                f"forced_failure_events={kernel.forced_failure_events}"
            ),
        ),
    )
