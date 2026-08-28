
"""Atomic conversion boundary for validated raw DDS replay messages."""
from __future__ import annotations

import hashlib
import os
import shutil
from dataclasses import dataclass
from pathlib import Path
from collections.abc import Iterable
from typing import TYPE_CHECKING, Protocol

from .external_replay_contract import (
    Channel,
    ContractConflict,
    OdometrySemantic,
    PointCloudSemantic,
    validate_semantic_round_trip,
)

if TYPE_CHECKING:
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2


@dataclass(frozen=True, slots=True)
class RawMessage:
    source_topic: str
    log_time_ns: int
    sequence: int
    payload: bytes


@dataclass(frozen=True, slots=True)
class CanonicalMessage:
    output_topic: str
    output_type: str
    log_time_ns: int
    sequence: int
    payload: bytes


class MessageCanonicalizer(Protocol):
    def canonicalize(self, message: RawMessage) -> CanonicalMessage:
        ...


class CanonicalWriter(Protocol):
    def write(self, message: CanonicalMessage) -> None:
        ...

    def finish(self) -> None:
        ...


@dataclass(frozen=True, slots=True)
class SourceInventory:
    channels: tuple[Channel, ...]
    channel_counts: tuple[tuple[str, int], ...]


class RosCdrCanonicalizer:
    """Use target ROS typesupport and reject every semantic round-trip mutation."""

    def canonicalize(self, message: RawMessage) -> CanonicalMessage:
        try:
            from rclpy.serialization import deserialize_message, serialize_message
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import PointCloud2
        except ImportError as error:
            raise ContractConflict("ros_typesupport_unavailable", str(error)) from error
        match message.source_topic:
            case "rt/utlidar/cloud":
                output_topic = "/utlidar/cloud"
                output_type = "sensor_msgs/msg/PointCloud2"
                message_type = PointCloud2
                semantic = _cloud_semantic
            case "rt/utlidar/robot_odom":
                output_topic = "/utlidar/robot_odom"
                output_type = "nav_msgs/msg/Odometry"
                message_type = Odometry
                semantic = _odometry_semantic
            case _:
                raise ContractConflict("unselected_source_topic", message.source_topic)
        try:
            source = deserialize_message(message.payload, message_type)
            payload = serialize_message(source)
            round_trip = deserialize_message(payload, message_type)
        except (RuntimeError, ValueError, TypeError) as error:
            raise ContractConflict("cdr_decode_or_encode_failure", str(error)) from error
        validate_semantic_round_trip(
            semantic(source),
            semantic(round_trip),
        )
        return CanonicalMessage(
            output_topic=output_topic,
            output_type=output_type,
            log_time_ns=message.log_time_ns,
            sequence=message.sequence,
            payload=payload,
        )


def convert_messages(
    messages: Iterable[RawMessage],
    canonicalizer: MessageCanonicalizer,
    writer: CanonicalWriter,
) -> tuple[str, ...]:
    """Canonicalize in source order while preserving log timestamps and sequence."""
    manifest: list[str] = []
    previous_key: tuple[int, int] | None = None
    try:
        for source in messages:
            source_key = (source.log_time_ns, source.sequence)
            if previous_key is not None and source_key < previous_key:
                raise ContractConflict("source_message_order_nondeterministic")
            previous_key = source_key
            canonical = canonicalizer.canonicalize(source)
            if canonical.log_time_ns != source.log_time_ns:
                raise ContractConflict("source_log_timestamp_mutated")
            if canonical.sequence != source.sequence:
                raise ContractConflict("source_sequence_mutated")
            writer.write(canonical)
            payload_hash = hashlib.sha256(canonical.payload).hexdigest()
            manifest.append(
                f"{canonical.log_time_ns}:{canonical.sequence}:"
                f"{canonical.output_topic}:{payload_hash}"
            )
    except ContractConflict:
        writer.finish()
        raise
    writer.finish()
    return tuple(manifest)


def enforce_determinism(
    first_manifest: tuple[str, ...],
    first_checksum: str,
    second_manifest: tuple[str, ...],
    second_checksum: str,
) -> None:
    if first_manifest != second_manifest or first_checksum != second_checksum:
        raise ContractConflict("short_conversion_nondeterministic")


def enforce_output_cap(output_size: int, byte_cap: int) -> None:
    if output_size > byte_cap:
        raise ContractConflict("full_output_byte_cap_exceeded", str(output_size))


def promote_output(staging: Path, destination: Path, byte_cap: int) -> str:
    """Promote only a bounded directory tree after hashing every regular file."""
    size = output_tree_size(staging)
    enforce_output_cap(size, byte_cap)
    digest = output_tree_checksum(staging)
    if destination.exists():
        raise ContractConflict("derived_destination_already_exists", str(destination))
    destination.parent.mkdir(parents=True, exist_ok=True)
    os.replace(staging, destination)
    descriptor = os.open(destination.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return digest


def output_tree_size(path: Path) -> int:
    """Count every regular derived file after rejecting links."""
    entries = tuple(path.rglob("*"))
    if any(entry.is_symlink() for entry in entries):
        raise ContractConflict("derived_output_link_forbidden")
    return sum(entry.stat().st_size for entry in entries if entry.is_file())


def output_tree_checksum(path: Path) -> str:
    """Hash relative names and bytes so two conversion roots are comparable."""
    files = tuple(sorted(entry for entry in path.rglob("*") if entry.is_file()))
    if any(entry.is_symlink() for entry in path.rglob("*")):
        raise ContractConflict("derived_output_link_forbidden")
    digest = hashlib.sha256()
    for file_path in files:
        digest.update(file_path.relative_to(path).as_posix().encode("utf-8"))
        with file_path.open("rb") as source:
            while chunk := source.read(1024 * 1024):
                digest.update(chunk)
    return digest.hexdigest()


def remove_staging(staging: Path) -> None:
    if staging.exists():
        shutil.rmtree(staging)


def _cloud_semantic(message: PointCloud2) -> PointCloudSemantic:
    stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
    header = (stamp_ns, message.header.frame_id)
    fields = tuple(
        (field.name, field.offset, field.datatype, field.count)
        for field in message.fields
    )
    return PointCloudSemantic(
        header, message.height, message.width, fields, message.is_bigendian,
        message.point_step, message.row_step, bytes(message.data), message.is_dense,
    )


def _odometry_semantic(message: Odometry) -> OdometrySemantic:
    stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
    pose = message.pose.pose
    twist = message.twist.twist
    return OdometrySemantic(
        header=(stamp_ns, message.header.frame_id),
        child_frame_id=message.child_frame_id,
        pose=(pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
              pose.orientation.y, pose.orientation.z, pose.orientation.w),
        pose_covariance=tuple(message.pose.covariance),
        twist=(twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x,
               twist.angular.y, twist.angular.z),
        twist_covariance=tuple(message.twist.covariance),
    )
