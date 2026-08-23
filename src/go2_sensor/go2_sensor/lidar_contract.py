"""
LiDAR acceptance node가 ROS message 밖에서 사용하는 입력 계약이다.

이 모듈은 frame, PointCloud2 field, timestamp 순서만 검증하며 ROS API,
변환, 지각 처리, publisher를 포함하지 않는다.
"""

from dataclasses import dataclass
from typing import Final


EXPECTED_FRAME_ID: Final = "utlidar_lidar"
REQUIRED_FIELD_NAMES: Final = frozenset({"x", "y", "z"})


@dataclass(frozen=True, slots=True)
class CloudLayout:
    """PointCloud2의 최초 layout 로그와 field 검증에 필요한 정보다."""

    height: int
    width: int
    point_step: int
    field_names: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class CloudSample:
    """ROS 독립 LiDAR cloud header와 layout 입력값이다."""

    frame_id: str
    stamp_nanoseconds: int
    layout: CloudLayout


@dataclass(frozen=True, slots=True)
class ValidationResult:
    """LiDAR acceptance contract의 판정과 거부 사유다."""

    is_valid: bool
    reason: str | None


def validate_cloud_sample(
    sample: CloudSample,
    previous_stamp_nanoseconds: int | None,
) -> ValidationResult:
    """frame, required fields, positive stamp, and timestamp order를 검증한다."""
    if sample.frame_id != EXPECTED_FRAME_ID:
        return ValidationResult(is_valid=False, reason="unexpected frame_id")
    if not REQUIRED_FIELD_NAMES.issubset(sample.layout.field_names):
        return ValidationResult(is_valid=False, reason="missing required point fields")
    if sample.stamp_nanoseconds <= 0:
        return ValidationResult(is_valid=False, reason="timestamp must be positive")
    if (
        previous_stamp_nanoseconds is not None
        and sample.stamp_nanoseconds < previous_stamp_nanoseconds
    ):
        return ValidationResult(is_valid=False, reason="timestamp regressed")
    return ValidationResult(is_valid=True, reason=None)
