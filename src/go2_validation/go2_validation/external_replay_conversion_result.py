
"""External replay conversion의 passed·deferred·conflict 결과 schema를 정의한다."""
from dataclasses import dataclass
from typing import Literal

from go2_validation.external_replay_acquisition_runner import AcquisitionResult
from go2_validation.external_replay_window import ShortWindow


ConversionStatus = Literal["passed", "deferred", "conflict"]


@dataclass(frozen=True, slots=True)
class InventoryRow:
    topic: str
    schema: str
    message_count: int


@dataclass(frozen=True, slots=True)
class ConversionFailure:
    status: Literal["deferred", "conflict"]
    reason_code: str | None
    detail: str | None


@dataclass(frozen=True, slots=True)
class ConversionResult:
    """후속 mapping runner가 소비하는 fixture와 provenance 결과다."""

    status: ConversionStatus
    source_id: str
    provenance: str
    reason_code: str | None
    detail: str | None
    artifact_absent: bool
    source_path: str | None
    source_checksum: str | None
    interval_start_ns: int | None
    interval_end_ns: int | None
    inventory: tuple[InventoryRow, ...]
    cloud_frames: tuple[str, ...]
    odometry_frames: tuple[str, ...]
    odometry_child_frames: tuple[str, ...]
    candidate_windows: tuple[ShortWindow, ...]
    selected_window: ShortWindow | None
    short_bag_path: str | None
    short_checksum: str | None
    short_repeat_checksum: str | None
    short_size_bytes: int | None
    short_cloud_count: int | None
    short_odometry_count: int | None
    full_bag_path: str | None
    full_checksum: str | None
    full_size_bytes: int | None
    full_cloud_count: int | None
    full_odometry_count: int | None


def failed_conversion(
    acquisition: AcquisitionResult,
    failure: ConversionFailure,
) -> ConversionResult:
    """Artifact가 없는 상태를 checksum 발명 없이 명시한다."""
    return ConversionResult(
        status=failure.status,
        source_id=acquisition.source_id,
        provenance="external_dynamic",
        reason_code=failure.reason_code,
        detail=failure.detail,
        artifact_absent=True,
        source_path=None,
        source_checksum=None,
        interval_start_ns=None,
        interval_end_ns=None,
        inventory=(),
        cloud_frames=(),
        odometry_frames=(),
        odometry_child_frames=(),
        candidate_windows=(),
        selected_window=None,
        short_bag_path=None,
        short_checksum=None,
        short_repeat_checksum=None,
        short_size_bytes=None,
        short_cloud_count=None,
        short_odometry_count=None,
        full_bag_path=None,
        full_checksum=None,
        full_size_bytes=None,
        full_cloud_count=None,
        full_odometry_count=None,
    )
