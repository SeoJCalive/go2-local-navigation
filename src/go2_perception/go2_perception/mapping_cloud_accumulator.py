"""같은 target frame으로 변환된 PointCloud2의 짧은 sliding window를 결합한다.

TF 조회나 ROS graph는 다루지 않는다. node가 odometry 시각으로 변환한 cloud만 받아
동일 layout을 확인하고 최신 stamp의 한 cloud로 결합한다.
"""

from collections import deque
from dataclasses import asdict, dataclass, replace
import json
from typing import Final, Generic, TypeVar

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


CloudT = TypeVar("CloudT")
MAPPING_CLOUD_ACCOUNTING_PREFIX: Final = "MAPPING_CLOUD_ACCOUNTING "


@dataclass(frozen=True, slots=True)
class MappingCloudAccounting:
    received: int
    future_waited: int
    recovered_after_retry: int
    processed: int
    output_published: int
    dropped_unrecoverable: int
    dropped_overflow: int
    pending_at_shutdown: int
    partial_frames_not_emitted: int
    emit_every: int
    output_stamp_regression_count: int


@dataclass(frozen=True, slots=True)
class PendingMappingCloud(Generic[CloudT]):
    cloud: CloudT
    source_stamp_nanoseconds: int
    waited_for_future: bool = False


@dataclass(frozen=True, slots=True)
class MappingCloudEnqueueResult:
    drop_reason: str | None = None


class MappingCloudRetryQueue(Generic[CloudT]):
    """source stamp 순서를 강제하는 bounded mutable cloud retry queue다."""

    def __init__(self, capacity: int, emit_every: int = 1) -> None:
        if capacity <= 0:
            raise MappingCloudWindowError("mapping_cloud_retry_capacity_invalid")
        if emit_every <= 0:
            raise MappingCloudWindowError("mapping_cloud_emit_every_invalid")
        self._capacity = capacity
        self._emit_every = emit_every
        self._pending: deque[PendingMappingCloud[CloudT]] = deque()
        self._largest_source_stamp_nanoseconds: int | None = None
        self._last_output_stamp_nanoseconds: int | None = None
        self._received = 0
        self._future_waited = 0
        self._recovered_after_retry = 0
        self._processed = 0
        self._output_published = 0
        self._dropped_unrecoverable = 0
        self._dropped_overflow = 0
        self._output_stamp_regression_count = 0

    def enqueue(
        self,
        cloud: CloudT,
        source_stamp_nanoseconds: int,
    ) -> MappingCloudEnqueueResult:
        self._received += 1
        largest_stamp = self._largest_source_stamp_nanoseconds
        if largest_stamp is not None and source_stamp_nanoseconds <= largest_stamp:
            self._dropped_unrecoverable += 1
            return MappingCloudEnqueueResult("source_stamp_not_increasing")
        if len(self._pending) >= self._capacity:
            self._dropped_overflow += 1
            return MappingCloudEnqueueResult("queue_capacity_exceeded")
        self._pending.append(
            PendingMappingCloud(cloud, source_stamp_nanoseconds),
        )
        self._largest_source_stamp_nanoseconds = source_stamp_nanoseconds
        return MappingCloudEnqueueResult()

    def head(self) -> PendingMappingCloud[CloudT] | None:
        return self._pending[0] if self._pending else None

    def mark_head_future_waited(self) -> None:
        head = self.head()
        if head is not None and not head.waited_for_future:
            self._pending[0] = replace(head, waited_for_future=True)
            self._future_waited += 1

    def drop_head_unrecoverable(self) -> PendingMappingCloud[CloudT] | None:
        head = self.head()
        if head is None:
            return None
        self._pending.popleft()
        self._dropped_unrecoverable += 1
        return head

    def take_head_for_processing(self) -> PendingMappingCloud[CloudT] | None:
        head = self.head()
        if head is None:
            return None
        self._pending.popleft()
        self._processed += 1
        if head.waited_for_future:
            self._recovered_after_retry += 1
        return head

    def record_output_publish(self, output_stamp_nanoseconds: int) -> None:
        last_output_stamp = self._last_output_stamp_nanoseconds
        if (
            last_output_stamp is not None
            and output_stamp_nanoseconds <= last_output_stamp
        ):
            self._output_stamp_regression_count += 1
        self._last_output_stamp_nanoseconds = output_stamp_nanoseconds
        self._output_published += 1

    def accounting(self, partial_frames_not_emitted: int = 0) -> MappingCloudAccounting:
        return MappingCloudAccounting(
            received=self._received,
            future_waited=self._future_waited,
            recovered_after_retry=self._recovered_after_retry,
            processed=self._processed,
            output_published=self._output_published,
            dropped_unrecoverable=self._dropped_unrecoverable,
            dropped_overflow=self._dropped_overflow,
            pending_at_shutdown=len(self._pending),
            partial_frames_not_emitted=partial_frames_not_emitted,
            emit_every=self._emit_every,
            output_stamp_regression_count=self._output_stamp_regression_count,
        )


def format_mapping_cloud_accounting(accounting: MappingCloudAccounting) -> str:
    return MAPPING_CLOUD_ACCOUNTING_PREFIX + json.dumps(
        asdict(accounting),
        separators=(",", ":"),
    )


@dataclass(frozen=True, slots=True)
class MappingCloudWindowError(Exception):
    """누적 대상 cloud의 frame 또는 binary layout이 계약과 다르다."""

    reason_code: str
    detail: str = ""

    def __str__(self) -> str:
        return f"{self.reason_code}: {self.detail}" if self.detail else self.reason_code


class MappingCloudWindow:
    """Sliding 또는 non-overlapping batch PointCloud2 window를 유지한다."""

    def __init__(
        self,
        frame_limit: int,
        target_frame: str,
        emit_every: int = 1,
    ) -> None:
        if frame_limit <= 0:
            raise MappingCloudWindowError("cloud_window_frame_limit_invalid")
        if not target_frame:
            raise MappingCloudWindowError("cloud_window_target_frame_invalid")
        if emit_every not in (1, frame_limit):
            raise MappingCloudWindowError("cloud_window_emit_every_invalid")
        self._target_frame = target_frame
        self._emit_every = emit_every
        self._clouds: deque[PointCloud2] = deque(maxlen=frame_limit)

    def add(self, cloud: PointCloud2) -> PointCloud2 | None:
        """검증된 cloud를 추가하고 설정된 cadence에서만 결합 결과를 반환한다."""
        _validate_cloud(cloud, self._target_frame)
        if self._clouds:
            _validate_matching_layout(self._clouds[-1], cloud)
        self._clouds.append(cloud)
        if self._emit_every > 1 and len(self._clouds) < self._emit_every:
            return None
        output = _concatenate_clouds(tuple(self._clouds), self._target_frame)
        if self._emit_every > 1:
            self._clouds.clear()
        return output

    @property
    def partial_frame_count(self) -> int:
        return len(self._clouds) if self._emit_every > 1 else 0


def compact_xyz_cloud(cloud: PointCloud2) -> PointCloud2:
    field_names = {field.name for field in cloud.fields}
    if not {"x", "y", "z"}.issubset(field_names):
        raise MappingCloudWindowError("cloud_xyz_fields_missing")
    header = Header()
    header.stamp = cloud.header.stamp
    header.frame_id = cloud.header.frame_id
    points = point_cloud2.read_points_list(
        cloud,
        field_names=("x", "y", "z"),
        skip_nans=False,
    )
    return point_cloud2.create_cloud_xyz32(header, points)


def _validate_cloud(cloud: PointCloud2, target_frame: str) -> None:
    if cloud.header.frame_id != target_frame:
        raise MappingCloudWindowError(
            "cloud_window_frame_mismatch",
            cloud.header.frame_id,
        )
    expected_size = cloud.height * cloud.row_step
    if (
        cloud.height != 1
        or cloud.width <= 0
        or cloud.point_step <= 0
        or cloud.row_step != cloud.width * cloud.point_step
        or len(cloud.data) != expected_size
    ):
        raise MappingCloudWindowError("cloud_window_layout_invalid")


def _validate_matching_layout(previous: PointCloud2, current: PointCloud2) -> None:
    previous_fields = tuple(
        (field.name, field.offset, field.datatype, field.count)
        for field in previous.fields
    )
    current_fields = tuple(
        (field.name, field.offset, field.datatype, field.count)
        for field in current.fields
    )
    if (
        previous_fields != current_fields
        or previous.point_step != current.point_step
        or previous.is_bigendian != current.is_bigendian
    ):
        raise MappingCloudWindowError("cloud_window_layout_changed")


def _concatenate_clouds(
    clouds: tuple[PointCloud2, ...],
    target_frame: str,
) -> PointCloud2:
    latest = clouds[-1]
    output = PointCloud2()
    output.header = Header()
    output.header.stamp = latest.header.stamp
    output.header.frame_id = target_frame
    output.height = 1
    output.width = sum(cloud.width for cloud in clouds)
    output.fields = list(latest.fields)
    output.is_bigendian = latest.is_bigendian
    output.point_step = latest.point_step
    output.row_step = output.width * output.point_step
    output.data = b"".join(bytes(cloud.data) for cloud in clouds)
    output.is_dense = all(cloud.is_dense for cloud in clouds)
    return output
