
"""누적 mapping cloud node의 terminal accounting을 launch log에서 읽는다.
ROS 2 launch는 node stdout 앞에 process prefix를 붙이므로, 이 module은 terminal
marker 뒤 JSON만 단 한 번 파싱해 immutable accounting으로 만든다. marker 누락,
중복 또는 malformed 값은 launch log를 보존한 typed failure로 호출자에게 전달한다.
"""

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Final
from types import MappingProxyType


MARKER: Final = "MAPPING_CLOUD_ACCOUNTING "
ACCOUNTING_FIELDS: Final = frozenset(
    {
        "received",
        "future_waited",
        "recovered_after_retry",
        "processed",
        "output_published",
        "dropped_unrecoverable",
        "dropped_overflow",
        "pending_at_shutdown",
        "partial_frames_not_emitted",
        "emit_every",
        "output_stamp_regression_count",
    }
)
CLOUD_ACCOUNTING_EMIT_CADENCES: Final = MappingProxyType(
    {
        "dimos_odom_accumulated": 1,
        "dimos_odom_accumulated_emit3": 3,
        "dimos_odom_accumulated_emit10": 10,
    }
)


@dataclass(frozen=True, slots=True)
class MappingCloudAccounting:
    """누적 cloud node가 종료 시 출력하는 lossless count projection이다."""

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
class MappingCloudAccountingError(Exception):
    """Terminal accounting launch-log boundary가 계약을 충족하지 못했다."""

    reason_code: str
    detail: str = ""

    def __str__(self) -> str:
        return f"{self.reason_code}: {self.detail}" if self.detail else self.reason_code


def read_mapping_cloud_accounting(log_path: Path) -> MappingCloudAccounting:
    """닫힌 owned launch log에서 terminal accounting 한 건을 읽는다."""
    try:
        log_text = log_path.read_text(encoding="utf-8")
    except OSError as error:
        raise MappingCloudAccountingError(
            "mapping_cloud_accounting_log_unreadable",
            str(log_path),
        ) from error
    return parse_mapping_cloud_accounting_log(log_text)


def mapping_cloud_accounting_for_profile(
    log_path: Path,
    scan_projection_profile: str,
) -> MappingCloudAccounting | None:
    """누적 profile에만 terminal accounting을 요구하고 raw profile은 그대로 둔다."""
    if scan_projection_profile in CLOUD_ACCOUNTING_EMIT_CADENCES:
        return read_mapping_cloud_accounting(log_path)
    return None


def parse_mapping_cloud_accounting_log(log_text: str) -> MappingCloudAccounting:
    """ROS launch prefix를 허용하면서 marker line 하나를 immutable 값으로 파싱한다."""
    marker_lines = tuple(line for line in log_text.splitlines() if MARKER in line)
    if not marker_lines:
        raise MappingCloudAccountingError("mapping_cloud_accounting_missing")
    if len(marker_lines) != 1:
        raise MappingCloudAccountingError("mapping_cloud_accounting_multiple")
    payload = marker_lines[0].split(MARKER, maxsplit=1)[1]
    try:
        document = json.loads(payload)
    except json.JSONDecodeError as error:
        raise MappingCloudAccountingError(
            "mapping_cloud_accounting_json_invalid",
            str(error),
        ) from error
    if not isinstance(document, dict) or set(document) != ACCOUNTING_FIELDS:
        raise MappingCloudAccountingError("mapping_cloud_accounting_shape_invalid")
    return MappingCloudAccounting(
        received=_nonnegative_count(document.get("received"), "received"),
        future_waited=_nonnegative_count(document.get("future_waited"), "future_waited"),
        recovered_after_retry=_nonnegative_count(
            document.get("recovered_after_retry"),
            "recovered_after_retry",
        ),
        processed=_nonnegative_count(document.get("processed"), "processed"),
        output_published=_nonnegative_count(
            document.get("output_published"),
            "output_published",
        ),
        dropped_unrecoverable=_nonnegative_count(
            document.get("dropped_unrecoverable"),
            "dropped_unrecoverable",
        ),
        dropped_overflow=_nonnegative_count(
            document.get("dropped_overflow"),
            "dropped_overflow",
        ),
        pending_at_shutdown=_nonnegative_count(
            document.get("pending_at_shutdown"),
            "pending_at_shutdown",
        ),
        partial_frames_not_emitted=_nonnegative_count(
            document.get("partial_frames_not_emitted"),
            "partial_frames_not_emitted",
        ),
        emit_every=_positive_count(document.get("emit_every"), "emit_every"),
        output_stamp_regression_count=_nonnegative_count(
            document.get("output_stamp_regression_count"),
            "output_stamp_regression_count",
        ),
    )


def _nonnegative_count(value: int | None, field_name: str) -> int:
    """JSON count field가 bool이 아닌 non-negative integer인지 경계에서 확인한다."""
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise MappingCloudAccountingError(
            "mapping_cloud_accounting_count_invalid",
            field_name,
        )
    return value


def _positive_count(value: int | None, field_name: str) -> int:
    count = _nonnegative_count(value, field_name)
    if count == 0:
        raise MappingCloudAccountingError(
            "mapping_cloud_accounting_count_invalid",
            field_name,
        )
    return count
