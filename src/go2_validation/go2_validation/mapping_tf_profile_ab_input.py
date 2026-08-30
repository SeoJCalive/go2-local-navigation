
"""TF profile A/B가 소비하는 external replay manifest 입력을 파싱한다.
이 모듈은 conversion manifest와 canonical short bag의 identity를 교차 검증해
실행 runner가 그대로 소비할 불변 입력으로 만든다. ROS node를 생성하거나 replay를
시작하지 않으며, manifest 경계의 오류는 MappingRuntimeDataError로 보존한다.
"""

from dataclasses import dataclass
import json
from pathlib import Path

from bringup.preflight_result import JsonDocument, JsonValue
from go2_validation.external_replay_converter import output_tree_checksum
from go2_validation.mapping_runtime_data import (
    BagExpectation,
    MappingRuntimeDataError,
    read_bag_expectation,
)


@dataclass(frozen=True, slots=True)
class MappingTfProfileAbInput:
    """A/B가 공유하는 short bag identity와 전체 소비 기대값이다."""

    bag_path: Path
    provenance: str
    source_checksum: str
    replay_checksum: str
    expectation: BagExpectation


def load_mapping_tf_profile_ab_input(
    project_root: Path,
    manifest_path: Path,
) -> MappingTfProfileAbInput:
    """Conversion JSON과 actual short bag의 path·count·checksum을 교차 검증한다."""
    try:
        document = json.loads(manifest_path.read_text(encoding="utf-8"))
    except OSError as error:
        raise MappingRuntimeDataError("tf_ab_manifest_unreadable", str(error)) from error
    except json.JSONDecodeError as error:
        raise MappingRuntimeDataError("tf_ab_manifest_invalid", str(error)) from error
    root = _required_mapping(document, "tf_ab_manifest_root")
    if _required_string(root.get("status"), "status") != "passed":
        raise MappingRuntimeDataError("tf_ab_fixture_not_passed", str(manifest_path))
    bag_path = Path(_required_string(root.get("short_bag_path"), "short_bag_path"))
    if not bag_path.is_absolute():
        bag_path = project_root / bag_path
    expectation = read_bag_expectation(bag_path)
    expected_counts = (
        _required_positive_integer(root.get("short_cloud_count"), "short_cloud_count"),
        _required_positive_integer(
            root.get("short_odometry_count"),
            "short_odometry_count",
        ),
    )
    if expected_counts != (expectation.cloud_count, expectation.odometry_count):
        raise MappingRuntimeDataError("tf_ab_short_count_mismatch", str(expected_counts))
    replay_checksum = output_tree_checksum(bag_path)
    manifest_checksum = _required_string(root.get("short_checksum"), "short_checksum")
    if replay_checksum != manifest_checksum:
        raise MappingRuntimeDataError("tf_ab_short_checksum_mismatch", replay_checksum)
    return MappingTfProfileAbInput(
        bag_path=bag_path,
        provenance=_required_string(root.get("provenance"), "provenance"),
        source_checksum=_required_string(root.get("source_checksum"), "source_checksum"),
        replay_checksum=replay_checksum,
        expectation=expectation,
    )


def _required_mapping(value: JsonValue, field_name: str) -> JsonDocument:
    if not isinstance(value, dict):
        raise MappingRuntimeDataError("tf_ab_mapping_invalid", field_name)
    return value


def _required_string(value: JsonValue, field_name: str) -> str:
    if not isinstance(value, str) or not value:
        raise MappingRuntimeDataError("tf_ab_string_invalid", field_name)
    return value


def _required_positive_integer(value: JsonValue, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise MappingRuntimeDataError("tf_ab_integer_invalid", field_name)
    return value
