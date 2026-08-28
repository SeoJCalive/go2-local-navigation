
"""Pure contracts for selecting and validating canonical external replay data."""
from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Final, TypeAlias


EXPECTED_CLOUD_COUNT: Final = 17_776
EXPECTED_ODOMETRY_COUNT: Final = 173_616
CLOUD_SOURCE: Final = "rt/utlidar/cloud"
ODOMETRY_SOURCE: Final = "rt/utlidar/robot_odom"
CLOUD_SCHEMA: Final = "sensor_msgs::msg::dds_::PointCloud2_"
ODOMETRY_SCHEMA: Final = "nav_msgs::msg::dds_::Odometry_"
SHA256_PATTERN: Final = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True, slots=True)
class AcquisitionConflict(Exception):
    reason: str
    detail: str = ""

    def __str__(self) -> str:
        return f"{self.reason}: {self.detail}" if self.detail else self.reason


@dataclass(frozen=True, slots=True)
class AcquisitionDeferred(Exception):
    reason: str
    detail: str = ""

    def __str__(self) -> str:
        return f"{self.reason}: {self.detail}" if self.detail else self.reason


@dataclass(frozen=True, slots=True)
class SourceSpec:
    source_id: str
    download_url: str
    archive_filename: str
    extracted_filename: str
    archive_sha256: str
    archive_size_bytes: int
    extracted_size_bytes: int
    minimum_free_bytes: int
    connect_timeout_seconds: int
    total_timeout_seconds: int
    max_attempts: int

    def __post_init__(self) -> None:
        if SHA256_PATTERN.fullmatch(self.archive_sha256) is None:
            raise AcquisitionConflict("invalid_archive_sha256")
        if not self.download_url.startswith("https://"):
            raise AcquisitionConflict("download_url_must_be_https")
        filenames = (self.archive_filename, self.extracted_filename)
        if any(
            not value or "/" in value or "\\" in value or value in {".", ".."}
            for value in filenames
        ):
            raise AcquisitionConflict("source_filename_invalid")
        limits = (
            self.archive_size_bytes,
            self.extracted_size_bytes,
            self.minimum_free_bytes,
            self.connect_timeout_seconds,
            self.total_timeout_seconds,
            self.max_attempts,
        )
        if min(limits) <= 0:
            raise AcquisitionConflict("source_limit_must_be_positive")


@dataclass(frozen=True, slots=True)
class ContractConflict(Exception):
    reason: str
    detail: str = ""

    def __str__(self) -> str:
        return f"{self.reason}: {self.detail}" if self.detail else self.reason


@dataclass(frozen=True, slots=True)
class Channel:
    channel_id: int
    topic: str
    schema: str


@dataclass(frozen=True, slots=True)
class CanonicalChannel:
    source_id: int
    source_topic: str
    output_topic: str
    output_type: str


@dataclass(frozen=True, slots=True)
class PointCloudSemantic:
    header: tuple[int, str]
    height: int
    width: int
    fields: tuple[tuple[str, int, int, int], ...]
    is_bigendian: bool
    point_step: int
    row_step: int
    data: bytes
    is_dense: bool


@dataclass(frozen=True, slots=True)
class OdometrySemantic:
    header: tuple[int, str]
    child_frame_id: str
    pose: tuple[float, ...]
    pose_covariance: tuple[float, ...]
    twist: tuple[float, ...]
    twist_covariance: tuple[float, ...]


ReplaySemantic: TypeAlias = PointCloudSemantic | OdometrySemantic


def canonical_channels(channels: tuple[Channel, ...]) -> tuple[CanonicalChannel, ...]:
    """Require one exact source channel for each canonical output topic."""
    contracts = (
        (CLOUD_SOURCE, CLOUD_SCHEMA, "/utlidar/cloud", "sensor_msgs/msg/PointCloud2"),
        (
            ODOMETRY_SOURCE,
            ODOMETRY_SCHEMA,
            "/utlidar/robot_odom",
            "nav_msgs/msg/Odometry",
        ),
    )
    selected: list[CanonicalChannel] = []
    for source_topic, schema, output_topic, output_type in contracts:
        matches = tuple(channel for channel in channels if channel.topic == source_topic)
        if len(matches) != 1:
            reason = "selected_channel_missing" if not matches else "duplicate_selected_channel"
            raise ContractConflict(reason, source_topic)
        channel = matches[0]
        if channel.schema != schema:
            raise ContractConflict("selected_schema_mismatch", source_topic)
        selected.append(
            CanonicalChannel(
                source_id=channel.channel_id,
                source_topic=source_topic,
                output_topic=output_topic,
                output_type=output_type,
            )
        )
    return tuple(selected)


def validate_semantic_round_trip(
    source: ReplaySemantic,
    round_trip: ReplaySemantic,
) -> None:
    if source != round_trip or type(source) is not type(round_trip):
        raise ContractConflict("cdr_semantic_mismatch")


def validate_counts(cloud_count: int, odometry_count: int) -> None:
    if (cloud_count, odometry_count) != (
        EXPECTED_CLOUD_COUNT,
        EXPECTED_ODOMETRY_COUNT,
    ):
        detail = f"cloud={cloud_count},odometry={odometry_count}"
        raise ContractConflict("selected_count_mismatch", detail)
