
"""외부 replay YAML을 다운로드·변환에 사용할 불변 계약으로 파싱한다."""
from dataclasses import dataclass
from pathlib import Path

import yaml

from go2_validation.external_replay_contract import (
    CLOUD_SCHEMA,
    CLOUD_SOURCE,
    EXPECTED_CLOUD_COUNT,
    EXPECTED_ODOMETRY_COUNT,
    ODOMETRY_SCHEMA,
    ODOMETRY_SOURCE,
    AcquisitionConflict,
    SourceSpec,
)


@dataclass(frozen=True, slots=True)
class ConversionSpec:
    """Canonical short·full 변환의 count와 용량 경계다."""

    source: SourceSpec
    expected_cloud_count: int
    expected_odometry_count: int
    short_minimum_cloud_count: int
    short_minimum_odometry_count: int
    full_output_cap_bytes: int


def load_source_spec(path: Path) -> SourceSpec:
    """하나의 pinned source만 허용하고 필수 필드를 typed 계약으로 바꾼다."""
    raw = _source_row(path)
    return _parse_source_spec(raw)


def load_conversion_spec(path: Path) -> ConversionSpec:
    """Selected channel count와 short/full 제한을 함께 파싱한다."""
    raw = _source_row(path)
    channels = raw.get("selected_channels")
    if not isinstance(channels, list):
        raise AcquisitionConflict("selected_channels_invalid")
    counts: dict[str, int] = {}
    expected_channels = {
        CLOUD_SOURCE: (
            CLOUD_SCHEMA,
            "/utlidar/cloud",
            "sensor_msgs/msg/PointCloud2",
        ),
        ODOMETRY_SOURCE: (
            ODOMETRY_SCHEMA,
            "/utlidar/robot_odom",
            "nav_msgs/msg/Odometry",
        ),
    }
    for channel in channels:
        if not isinstance(channel, dict):
            raise AcquisitionConflict("selected_channel_row_invalid")
        topic = _required_string(channel, "source_topic")
        if topic in counts:
            raise AcquisitionConflict("duplicate_selected_channel", topic)
        expected = expected_channels.get(topic)
        if expected is None:
            raise AcquisitionConflict("selected_channel_set_invalid", topic)
        actual = (
            _required_string(channel, "source_schema"),
            _required_string(channel, "canonical_topic"),
            _required_string(channel, "canonical_type"),
        )
        if actual != expected:
            raise AcquisitionConflict("selected_channel_contract_mismatch", topic)
        counts[topic] = _required_positive_int(channel, "expected_count")
    if set(counts) != set(expected_channels):
        raise AcquisitionConflict("selected_channel_set_invalid")
    if (counts[CLOUD_SOURCE], counts[ODOMETRY_SOURCE]) != (
        EXPECTED_CLOUD_COUNT,
        EXPECTED_ODOMETRY_COUNT,
    ):
        raise AcquisitionConflict("selected_count_contract_mismatch")
    if _required_positive_int(raw, "short_duration_seconds") != 120:
        raise AcquisitionConflict("short_duration_contract_mismatch")
    return ConversionSpec(
        source=_parse_source_spec(raw),
        expected_cloud_count=counts[CLOUD_SOURCE],
        expected_odometry_count=counts[ODOMETRY_SOURCE],
        short_minimum_cloud_count=_required_positive_int(
            raw,
            "short_minimum_cloud_count",
        ),
        short_minimum_odometry_count=_required_positive_int(
            raw,
            "short_minimum_odometry_count",
        ),
        full_output_cap_bytes=_required_positive_int(raw, "full_output_cap_bytes"),
    )


def _source_row(path: Path) -> dict:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as error:
        raise AcquisitionConflict("source_manifest_unreadable", str(error)) from error
    except yaml.YAMLError as error:
        raise AcquisitionConflict("source_manifest_yaml_invalid", str(error)) from error
    if not isinstance(document, dict):
        raise AcquisitionConflict("source_manifest_root_invalid")
    sources = document.get("sources")
    if not isinstance(sources, list) or len(sources) != 1:
        raise AcquisitionConflict("source_manifest_cardinality_invalid")
    raw = sources[0]
    if not isinstance(raw, dict):
        raise AcquisitionConflict("source_manifest_row_invalid")
    return raw


def _parse_source_spec(raw: dict) -> SourceSpec:
    try:
        return SourceSpec(
            source_id=_required_string(raw, "source_id"),
            download_url=_required_string(raw, "download_url"),
            archive_filename=_required_string(raw, "archive_filename"),
            extracted_filename=_required_string(raw, "extracted_filename"),
            archive_sha256=_required_string(raw, "archive_sha256"),
            archive_size_bytes=_required_positive_int(raw, "archive_size_bytes"),
            extracted_size_bytes=_required_positive_int(raw, "extracted_size_bytes"),
            minimum_free_bytes=_required_positive_int(raw, "minimum_free_bytes"),
            connect_timeout_seconds=_required_positive_int(
                raw,
                "connect_timeout_seconds",
            ),
            total_timeout_seconds=_required_positive_int(raw, "total_timeout_seconds"),
            max_attempts=_required_positive_int(raw, "max_attempts"),
        )
    except KeyError as error:
        raise AcquisitionConflict("source_manifest_field_missing", str(error)) from error


def _required_string(raw: dict, key: str) -> str:
    value = raw[key]
    if not isinstance(value, str) or not value:
        raise AcquisitionConflict("source_manifest_string_invalid", key)
    return value


def _required_positive_int(raw: dict, key: str) -> int:
    value = raw[key]
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise AcquisitionConflict("source_manifest_integer_invalid", key)
    return value
