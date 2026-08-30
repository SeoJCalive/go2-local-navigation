
"""Todo 12가 소비하는 rosbag metadata와 external-full custody를 파싱한다.
파일 경계에서 cloud·odometry count, 재생 시간, source checksum과 derived bag
checksum을 불변 값으로 바꾼다. ROS 실행 코드는 이 값을 다시 해석하지 않는다.
"""

from dataclasses import dataclass
from enum import Enum
import json
from pathlib import Path

import yaml

from bringup.preflight_result import JsonDocument, JsonValue
from go2_validation.typing_compat import assert_never


@dataclass(frozen=True, slots=True)
class MappingRuntimeDataError(Exception):
    """Bag metadata 또는 conversion result가 실행 계약과 다르다."""

    reason_code: str
    detail: str

    def __str__(self) -> str:
        return f"{self.reason_code}: {self.detail}"


@dataclass(frozen=True, slots=True)
class BagExpectation:
    """한 canonical bag의 selected message 수와 log-time 구간이다."""

    cloud_count: int
    odometry_count: int
    start_nanoseconds: int
    duration_nanoseconds: int

    @property
    def end_nanoseconds(self) -> int:
        """Rosbag metadata의 exclusive 종료 log time을 반환한다."""
        return self.start_nanoseconds + self.duration_nanoseconds

    @property
    def playback_timeout_seconds(self) -> float:
        """1.0배속 duration에 20%와 120초 teardown 여유를 둔다."""
        duration_seconds = self.duration_nanoseconds / 1_000_000_000
        return max(90.0, duration_seconds * 1.2 + 120.0)


class ExternalFullStatus(str, Enum):
    """Todo 9 full fixture의 허용된 custody 상태다."""

    PASSED = "passed"
    DEFERRED = "deferred"
    CONFLICT = "conflict"


@dataclass(frozen=True, slots=True)
class ExternalFullReplay:
    """Raw source와 full derived bag identity를 분리한 실행 입력이다."""

    status: ExternalFullStatus
    provenance: str
    bag_path: Path | None
    source_checksum: str | None
    replay_checksum: str | None
    cloud_count: int
    odometry_count: int


def read_bag_expectation(bag_root: Path) -> BagExpectation:
    """Rosbag2 metadata에서 두 selected topic과 시간 구간을 파싱한다."""
    metadata_path = bag_root / "metadata.yaml"
    try:
        document = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise MappingRuntimeDataError("bag_metadata_unreadable", str(error)) from error
    root = _required_mapping(document, "metadata_root")
    information = _required_mapping(
        root.get("rosbag2_bagfile_information"),
        "bagfile_information",
    )
    duration = _required_mapping(information.get("duration"), "duration")
    starting_time = _required_mapping(
        information.get("starting_time"),
        "starting_time",
    )
    topic_rows = information.get("topics_with_message_count")
    if not isinstance(topic_rows, list):
        raise MappingRuntimeDataError("bag_topics_invalid", str(topic_rows))
    topics: dict[str, tuple[str, int]] = {}
    for raw_row in topic_rows:
        row = _required_mapping(raw_row, "topic_row")
        metadata = _required_mapping(row.get("topic_metadata"), "topic_metadata")
        name = _required_string(metadata.get("name"), "topic_name")
        message_type = _required_string(metadata.get("type"), "topic_type")
        count = _required_nonnegative_integer(row.get("message_count"), "message_count")
        topics[name] = (message_type, count)
    expected_topics = {
        "/utlidar/cloud": "sensor_msgs/msg/PointCloud2",
        "/utlidar/robot_odom": "nav_msgs/msg/Odometry",
    }
    if set(topics) != set(expected_topics):
        raise MappingRuntimeDataError("bag_topic_set_mismatch", str(tuple(sorted(topics))))
    for name, expected_type in expected_topics.items():
        if topics[name][0] != expected_type:
            raise MappingRuntimeDataError("bag_topic_type_mismatch", name)
    return BagExpectation(
        cloud_count=topics["/utlidar/cloud"][1],
        odometry_count=topics["/utlidar/robot_odom"][1],
        start_nanoseconds=_required_nonnegative_integer(
            starting_time.get("nanoseconds_since_epoch"),
            "starting_nanoseconds",
        ),
        duration_nanoseconds=_required_positive_integer(
            duration.get("nanoseconds"),
            "duration_nanoseconds",
        ),
    )


def read_external_full_replay(result_path: Path) -> ExternalFullReplay:
    """Todo 9 conversion JSON을 full replay custody로 파싱한다."""
    try:
        document = json.loads(result_path.read_text(encoding="utf-8"))
    except OSError as error:
        raise MappingRuntimeDataError("conversion_result_unreadable", str(error)) from error
    except json.JSONDecodeError as error:
        raise MappingRuntimeDataError("conversion_result_invalid", str(error)) from error
    root = _required_mapping(document, "conversion_root")
    raw_status = _required_string(root.get("status"), "conversion_status")
    try:
        status = ExternalFullStatus(raw_status)
    except ValueError as error:
        raise MappingRuntimeDataError("conversion_status_invalid", raw_status) from error
    provenance = _required_string(root.get("provenance"), "provenance")
    if provenance != "external_dynamic":
        raise MappingRuntimeDataError("conversion_provenance_invalid", provenance)
    match status:
        case ExternalFullStatus.PASSED:
            return ExternalFullReplay(
                status=status,
                provenance=provenance,
                bag_path=Path(_required_string(root.get("full_bag_path"), "full_bag_path")),
                source_checksum=_required_string(
                    root.get("source_checksum"),
                    "source_checksum",
                ),
                replay_checksum=_required_string(
                    root.get("full_checksum"),
                    "full_checksum",
                ),
                cloud_count=_required_positive_integer(
                    root.get("full_cloud_count"),
                    "full_cloud_count",
                ),
                odometry_count=_required_positive_integer(
                    root.get("full_odometry_count"),
                    "full_odometry_count",
                ),
            )
        case ExternalFullStatus.DEFERRED | ExternalFullStatus.CONFLICT:
            return ExternalFullReplay(status, provenance, None, None, None, 0, 0)
        case unreachable:
            assert_never(unreachable)


def assert_external_metadata_matches(
    replay: ExternalFullReplay,
    expectation: BagExpectation,
) -> None:
    """Passed manifest count와 actual full bag metadata가 같은지 확인한다."""
    if replay.status is not ExternalFullStatus.PASSED:
        raise MappingRuntimeDataError("external_fixture_not_passed", replay.status.value)
    if replay.cloud_count != expectation.cloud_count:
        raise MappingRuntimeDataError("external_cloud_count_mismatch", str(expectation.cloud_count))
    if replay.odometry_count != expectation.odometry_count:
        raise MappingRuntimeDataError(
            "external_odometry_count_mismatch",
            str(expectation.odometry_count),
        )


def _required_mapping(value: JsonValue, field_name: str) -> JsonDocument:
    if not isinstance(value, dict):
        raise MappingRuntimeDataError("mapping_field_invalid", field_name)
    return value


def _required_string(value, field_name: str) -> str:
    if not isinstance(value, str) or not value:
        raise MappingRuntimeDataError("string_field_invalid", field_name)
    return value


def _required_nonnegative_integer(value, field_name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise MappingRuntimeDataError("integer_field_invalid", field_name)
    return value


def _required_positive_integer(value, field_name: str) -> int:
    parsed = _required_nonnegative_integer(value, field_name)
    if parsed == 0:
        raise MappingRuntimeDataError("positive_integer_required", field_name)
    return parsed
