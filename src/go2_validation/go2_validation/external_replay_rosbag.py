
"""Mixed-format MCAP reader와 Humble rosbag2 canonical writer를 연결한다."""
from collections.abc import Iterator
from pathlib import Path
from typing import Final

from go2_validation.external_replay_contract import (
    CLOUD_SOURCE,
    ODOMETRY_SOURCE,
    Channel,
    ContractConflict,
    canonical_channels,
)
from go2_validation.external_replay_converter import (
    CanonicalMessage,
    RawMessage,
    SourceInventory,
)


QOS_PROFILES: Final = """- history: 1
  depth: 1
  reliability: 1
  durability: 2
  deadline:
    sec: 9223372036
    nsec: 854775807
  lifespan:
    sec: 9223372036
    nsec: 854775807
  liveliness: 1
  liveliness_lease_duration:
    sec: 9223372036
    nsec: 854775807
  avoid_ros_namespace_conventions: false
"""
SELECTED_TOPICS: Final = frozenset({CLOUD_SOURCE, ODOMETRY_SOURCE})


def inspect_source_inventory(path: Path) -> SourceInventory:
    """CRC를 켜고 전체 source를 읽어 schema와 channel count를 검증한다."""
    try:
        from mcap.exceptions import McapError
        from mcap.reader import make_reader
        from mcap.stream_reader import CRCValidationError
    except ImportError as error:
        raise ContractConflict("mcap_dependency_unavailable", str(error)) from error
    channels: dict[int, Channel] = {}
    counts: dict[int, int] = {}
    try:
        with _mcap_path(path).open("rb") as source:
            reader = make_reader(source, validate_crcs=True)
            summary = reader.get_summary()
            if summary is None:
                raise ContractConflict("mcap_summary_missing")
            for channel_id, source_channel in summary.channels.items():
                schema = summary.schemas.get(source_channel.schema_id)
                if schema is None:
                    raise ContractConflict("mcap_schema_missing", str(channel_id))
                channels[channel_id] = Channel(
                    channel_id=channel_id,
                    topic=source_channel.topic,
                    schema=schema.name,
                )
            canonical_channels(tuple(channels.values()))
            for _schema, channel, _message in reader.iter_messages():
                counts[channel.id] = counts.get(channel.id, 0) + 1
    except (OSError, McapError, CRCValidationError, EOFError) as error:
        raise ContractConflict("mcap_crc_or_format_failure", str(error)) from error
    return SourceInventory(
        channels=tuple(channels.values()),
        channel_counts=tuple(
            sorted(
                (channels[channel_id].topic, count)
                for channel_id, count in counts.items()
            )
        ),
    )


def iter_selected_messages(
    path: Path,
    start_ns: int | None,
    end_ns: int | None,
) -> Iterator[RawMessage]:
    """Mixed source에서 선택한 CDR channel의 원래 sequence와 time만 반환한다."""
    try:
        from mcap.exceptions import McapError
        from mcap.reader import make_reader
        from mcap.stream_reader import CRCValidationError
    except ImportError as error:
        raise ContractConflict("mcap_dependency_unavailable", str(error)) from error
    try:
        with _mcap_path(path).open("rb") as source:
            reader = make_reader(source, validate_crcs=True)
            for _schema, channel, message in reader.iter_messages(
                topics=list(SELECTED_TOPICS),
                start_time=start_ns,
                end_time=end_ns,
            ):
                yield RawMessage(
                    channel.topic,
                    message.log_time,
                    message.sequence,
                    bytes(message.data),
                )
    except (OSError, McapError, CRCValidationError, EOFError) as error:
        raise ContractConflict("mcap_crc_or_format_failure", str(error)) from error


class Rosbag2CanonicalWriter:
    """두 canonical topic과 native writer lifetime을 소유한다."""

    def __init__(self, path: Path) -> None:
        try:
            import rosbag2_py
        except ImportError as error:
            raise ContractConflict("rosbag2_py_unavailable", str(error)) from error
        self._writer = rosbag2_py.SequentialWriter()
        try:
            self._writer.open(
                rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""),
            )
            for topic, message_type in (
                ("/utlidar/cloud", "sensor_msgs/msg/PointCloud2"),
                ("/utlidar/robot_odom", "nav_msgs/msg/Odometry"),
            ):
                self._writer.create_topic(
                    rosbag2_py.TopicMetadata(
                        name=topic,
                        type=message_type,
                        serialization_format="cdr",
                        offered_qos_profiles=QOS_PROFILES,
                    )
                )
        except RuntimeError as error:
            raise ContractConflict("canonical_writer_open_failure", str(error)) from error

    def write(self, message: CanonicalMessage) -> None:
        """검증된 canonical message만 원래 log timestamp로 기록한다."""
        expected_types = {
            "/utlidar/cloud": "sensor_msgs/msg/PointCloud2",
            "/utlidar/robot_odom": "nav_msgs/msg/Odometry",
        }
        if expected_types.get(message.output_topic) != message.output_type:
            raise ContractConflict("canonical_output_contract_mismatch")
        try:
            self._writer.write(
                message.output_topic,
                message.payload,
                message.log_time_ns,
            )
        except RuntimeError as error:
            raise ContractConflict("canonical_writer_write_failure", str(error)) from error

    def finish(self) -> None:
        """Native writer를 닫아 metadata와 MCAP footer를 확정한다."""
        try:
            self._writer.close()
        except RuntimeError as error:
            raise ContractConflict("canonical_writer_close_failure", str(error)) from error


def _mcap_path(path: Path) -> Path:
    if path.is_file() and not path.is_symlink():
        return path
    if path.is_dir() and not path.is_symlink():
        candidates = tuple(sorted(path.glob("*.mcap")))
        if len(candidates) == 1 and not candidates[0].is_symlink():
            return candidates[0]
    raise ContractConflict("mcap_source_file_invalid", str(path))
