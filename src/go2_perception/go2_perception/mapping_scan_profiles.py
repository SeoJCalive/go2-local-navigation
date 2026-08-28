"""Mapping 전용 2D scan projection profile을 YAML에서 파싱한다.

단일 cloud 기본 경로와 외부 DimOS replay 전용 odometry 보정 누적 경로를 분리해,
실물 기본값이 외부 자료 실험값으로 암묵적으로 바뀌지 않게 한다.
"""

from dataclasses import dataclass
from enum import Enum
import math
from pathlib import Path
from typing import TypeAlias

import yaml


JsonScalar: TypeAlias = str | int | float | bool | None
JsonValue: TypeAlias = JsonScalar | list["JsonValue"] | dict[str, "JsonValue"]


@dataclass(frozen=True, slots=True)
class MappingScanProfile:
    """하나의 converter 입력과 선택적 cloud 누적 계약이다."""

    profile_id: str
    scope: str
    accumulator_enabled: bool
    frame_limit: int
    emit_every: int
    input_qos_depth: int
    retry_queue_capacity: int
    accumulator_target_frame: str
    accumulator_output_topic: str
    converter_input_topic: str
    converter_min_height: float | None
    converter_queue_size: int | None
    source_commit: str | None


@dataclass(frozen=True, slots=True)
class MappingScanProfileError(Exception):
    """Profile registry가 실행 가능한 폐쇄된 값이 아닐 때 발생한다."""

    reason_code: str
    detail: str = ""

    def __str__(self) -> str:
        return f"{self.reason_code}: {self.detail}" if self.detail else self.reason_code


class ExecutionMode(str, Enum):
    ONBOARD = "onboard"
    EXTERNAL_REPLAY = "external_replay"


def load_mapping_scan_profile(
    path: Path,
    profile_id: str,
    execution_mode: str,
) -> MappingScanProfile:
    """지정 profile을 검증해 launch에서 사용할 불변 값으로 반환한다."""
    try:
        document: JsonValue = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise MappingScanProfileError(
            "mapping_scan_profile_registry_unreadable",
            str(error),
        ) from error
    mode = _parse_execution_mode(execution_mode)
    root = _required_mapping(document, "registry_root")
    mapping_scan = _required_mapping(root.get("mapping_scan"), "mapping_scan")
    registry = _required_mapping(
        mapping_scan.get("projection_profiles"),
        "projection_profiles",
    )
    profiles = _required_mapping(registry.get("profiles"), "profiles")
    raw_profile = profiles.get(profile_id)
    if raw_profile is None:
        raise MappingScanProfileError("unknown_mapping_scan_profile", profile_id)
    profile = _required_mapping(raw_profile, profile_id)
    _require_scope(_required_string(profile.get("scope"), "scope"), mode)
    accumulator = _required_mapping(profile.get("accumulator"), "accumulator")
    converter_override = _optional_mapping(
        profile.get("converter_override"),
        "converter_override",
    )
    source = _optional_mapping(profile.get("source"), "source")
    source_commit = None if source is None else _required_string(
        source.get("commit"),
        "source_commit",
    )
    enabled = _required_boolean(accumulator.get("enabled"), "enabled")
    frame_limit = _required_positive_integer(
        accumulator.get("frame_limit"),
        "frame_limit",
    )
    emit_every = _required_positive_integer(
        accumulator.get("emit_every"),
        "emit_every",
    )
    input_qos_depth = _required_positive_integer(
        accumulator.get("input_qos_depth"),
        "input_qos_depth",
    )
    retry_queue_capacity = _required_positive_integer(
        accumulator.get("retry_queue_capacity"),
        "retry_queue_capacity",
    )
    if not enabled and frame_limit != 1:
        raise MappingScanProfileError(
            "disabled_accumulator_requires_single_frame",
            profile_id,
        )
    if emit_every not in (1, frame_limit):
        raise MappingScanProfileError(
            "accumulator_emit_every_invalid",
            profile_id,
        )
    converter_min_height = (
        None
        if converter_override is None or "min_height" not in converter_override
        else _required_finite_number(
            converter_override["min_height"],
            "converter_min_height",
        )
    )
    converter_queue_size = (
        None
        if converter_override is None or "queue_size" not in converter_override
        else _required_positive_integer(
            converter_override["queue_size"],
            "converter_queue_size",
        )
    )
    return MappingScanProfile(
        profile_id=profile_id,
        scope=_required_string(profile.get("scope"), "scope"),
        accumulator_enabled=enabled,
        frame_limit=frame_limit,
        emit_every=emit_every,
        input_qos_depth=input_qos_depth,
        retry_queue_capacity=retry_queue_capacity,
        accumulator_target_frame=_required_string(
            accumulator.get("target_frame"),
            "target_frame",
        ),
        accumulator_output_topic=_required_string(
            accumulator.get("output_topic"),
            "output_topic",
        ),
        converter_input_topic=_required_string(
            profile.get("converter_input_topic"),
            "converter_input_topic",
        ),
        converter_min_height=converter_min_height,
        converter_queue_size=converter_queue_size,
        source_commit=source_commit,
    )


def _parse_execution_mode(value: str) -> ExecutionMode:
    try:
        return ExecutionMode(value)
    except ValueError as error:
        raise MappingScanProfileError(
            "unknown_mapping_scan_execution_mode",
            value,
        ) from error


def _require_scope(scope: str, mode: ExecutionMode) -> None:
    match scope:
        case "onboard_and_replay_default":
            return
        case "dimos_external_replay_only":
            if mode is not ExecutionMode.EXTERNAL_REPLAY:
                raise MappingScanProfileError(
                    "mapping_scan_profile_external_replay_required",
                    mode.value,
                )
        case _:
            raise MappingScanProfileError("unknown_mapping_scan_profile_scope", scope)


def _required_mapping(value: JsonValue, field_name: str) -> dict[str, JsonValue]:
    if not isinstance(value, dict):
        raise MappingScanProfileError("mapping_scan_profile_mapping_invalid", field_name)
    return value


def _optional_mapping(
    value: JsonValue,
    field_name: str,
) -> dict[str, JsonValue] | None:
    if value is None:
        return None
    return _required_mapping(value, field_name)


def _required_string(value: JsonValue, field_name: str) -> str:
    if not isinstance(value, str) or not value:
        raise MappingScanProfileError("mapping_scan_profile_string_invalid", field_name)
    return value


def _required_boolean(value: JsonValue, field_name: str) -> bool:
    if not isinstance(value, bool):
        raise MappingScanProfileError("mapping_scan_profile_boolean_invalid", field_name)
    return value


def _required_positive_integer(value: JsonValue, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise MappingScanProfileError("mapping_scan_profile_integer_invalid", field_name)
    return value


def _required_finite_number(value: JsonValue, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MappingScanProfileError("mapping_scan_profile_numeric_invalid", field_name)
    numeric_value = float(value)
    if not math.isfinite(numeric_value):
        raise MappingScanProfileError("mapping_scan_profile_numeric_invalid", field_name)
    return numeric_value
