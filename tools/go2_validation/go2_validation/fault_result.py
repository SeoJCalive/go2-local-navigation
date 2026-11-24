"""Fault acceptance runner가 기록하는 JSON 결과 schema를 정의한다."""

from dataclasses import dataclass
from typing import Literal, TypedDict


ResultStatus = Literal["passed", "failed"]


class FaultScenarioDocument(TypedDict):
    """JSON에 기록되는 scenario 한 행이다."""

    scenario_id: str
    status: ResultStatus
    reason_code: str
    suppressed_outputs: list[str]
    recovered_outputs: list[str]
    recovery_elapsed_nanoseconds: int | None
    child_exit_code: int


class FaultReportDocument(TypedDict):
    """Stage 11 fault acceptance JSON의 최상위 shape다."""

    schema_version: int
    record_kind: str
    overall: ResultStatus
    domain_id: int
    command_publisher_count: int
    motion_gates_closed: bool
    scenarios: list[FaultScenarioDocument]


@dataclass(frozen=True, slots=True)
class FaultScenarioResult:
    """실제 downstream 관찰로 판정한 scenario 결과다."""

    scenario_id: str
    status: ResultStatus
    reason_code: str
    suppressed_outputs: tuple[str, ...]
    recovered_outputs: tuple[str, ...]
    recovery_elapsed_nanoseconds: int | None
    child_exit_code: int


@dataclass(frozen=True, slots=True)
class FaultAcceptanceReport:
    """모든 scenario와 공통 안전 경계를 묶은 결과다."""

    overall: ResultStatus
    domain_id: int
    command_publisher_count: int
    motion_gates_closed: bool
    scenarios: tuple[FaultScenarioResult, ...]


def fault_report_document(report: FaultAcceptanceReport) -> FaultReportDocument:
    """불변 report를 JSON 호환 schema로 투영한다."""
    scenarios: list[FaultScenarioDocument] = [
        {
            "scenario_id": result.scenario_id,
            "status": result.status,
            "reason_code": result.reason_code,
            "suppressed_outputs": list(result.suppressed_outputs),
            "recovered_outputs": list(result.recovered_outputs),
            "recovery_elapsed_nanoseconds": result.recovery_elapsed_nanoseconds,
            "child_exit_code": result.child_exit_code,
        }
        for result in report.scenarios
    ]
    return {
        "schema_version": 1,
        "record_kind": "software_fault_acceptance_result",
        "overall": report.overall,
        "domain_id": report.domain_id,
        "command_publisher_count": report.command_publisher_count,
        "motion_gates_closed": report.motion_gates_closed,
        "scenarios": scenarios,
    }
