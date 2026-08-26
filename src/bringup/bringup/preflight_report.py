"""통합 preflight 관찰값의 JSON 문서 경계와 provenance를 정의한다."""

from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path

from bringup.preflight_types import (
    CheckResult,
    CheckStatus,
    TopicSummary,
)


@dataclass(frozen=True, slots=True)
class EnvironmentObservation:
    """wrapper 적용 여부를 판단할 수 있는 실제 환경값이다."""

    rmw_implementation: str
    ros_domain_id: str
    go2_interface: str
    cyclonedds_uri: str
    interface_operstate: str
    bringup_prefix: str
    runtime_wrapper_exists: bool


@dataclass(frozen=True, slots=True)
class TransformObservation:
    """필수 parent·child frame 연결의 runtime 조회 결과다."""

    parent_frame: str
    child_frame: str
    available: bool


@dataclass(frozen=True, slots=True)
class GraphObservation:
    """통합 stack node가 startup 뒤 유지됐는지 나타낸다."""

    expected_nodes: tuple[str, ...]
    seen_nodes: tuple[str, ...]
    lost_nodes: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class SafetyObservation:
    """motion gate와 프로젝트 command publisher의 최대 관찰값이다."""

    output_enabled: bool | None
    physical_validation_approved: bool | None
    sport_request_max_publishers: int
    lowcmd_max_publishers: int


@dataclass(frozen=True, slots=True)
class ObserverReport:
    """ROS 관찰기가 종료 직전에 저장하는 자동 실행 문서다."""

    schema_version: int
    record_kind: str
    run_id: str
    run_label: str
    target: str
    started_at: str
    completed_at: str
    requested_duration_seconds: int
    actual_duration_seconds: float
    physical_motion: bool
    command_publication: bool
    overall_status: CheckStatus
    checks: tuple[CheckResult, ...]
    environment: EnvironmentObservation
    graph: GraphObservation
    transforms: tuple[TransformObservation, ...]
    safety: SafetyObservation
    topics: tuple[TopicSummary, ...]


def write_observer_report(report: ObserverReport, path: Path) -> None:
    """Observer JSON을 같은 directory의 임시 파일을 거쳐 원자적으로 저장한다."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_suffix(f"{path.suffix}.tmp")
    temporary_path.write_text(
        json.dumps(asdict(report), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary_path, path)
