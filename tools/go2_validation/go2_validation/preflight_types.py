"""통합 preflight 모듈 사이에서 공유하는 불변 계약을 정의한다."""

from dataclasses import dataclass
from enum import Enum


class CheckStatus(str, Enum):
    """자동 판정 결과의 폐쇄된 상태 집합이다."""

    PASS = "PASS"
    FAIL = "FAIL"
    WARN = "WARN"
    DEFERRED = "DEFERRED"


@dataclass(frozen=True, slots=True)
class CheckResult:
    """하나의 재실행 가능한 검사 결과다."""

    check_id: str
    status: CheckStatus
    detail: str


@dataclass(frozen=True, slots=True)
class TopicContract:
    """필수 topic의 graph·frame·timing 합격 기준이다."""

    topic: str
    expected_type: str
    expected_frame: str | None
    expected_child_frame: str | None
    minimum_rate_hz: float
    maximum_gap_seconds: float


@dataclass(frozen=True, slots=True)
class Pose2D:
    """정지 odometry의 평면 drift 계산에 필요한 pose다."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True, slots=True)
class ObservedMessage:
    """ROS message에서 추출한 topic 독립 관찰값이다."""

    receive_nanoseconds: int
    stamp_nanoseconds: int | None
    frame_id: str | None
    child_frame_id: str | None
    is_valid: bool
    pose: Pose2D | None


@dataclass(frozen=True, slots=True)
class TopicSummary:
    """한 실행 구간의 topic 통계와 pose 변화를 보존한다."""

    contract: TopicContract
    received_messages: int
    invalid_messages: int
    timestamp_regressions: int
    observed_frames: tuple[str, ...]
    observed_child_frames: tuple[str, ...]
    observed_types: tuple[str, ...]
    maximum_publisher_count: int
    rate_hz: float
    maximum_gap_seconds: float
    drift_translation_m: float | None
    drift_yaw_rad: float | None
    maximum_step_translation_m: float | None
    maximum_step_yaw_rad: float | None
