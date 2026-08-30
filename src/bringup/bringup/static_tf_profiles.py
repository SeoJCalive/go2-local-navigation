"""
Static sensor TF YAML을 launch용 불변 값으로 파싱한다.

프로필 ID는 실물 기본 구성과 외부 replay를 분리하며, 등록되지 않은 ID나 유효하지
않은 frame·vector·quaternion을 launch 전에 거부한다. 이 모듈은 ROS graph를 열거나
TF를 publish하지 않는다.
"""

from dataclasses import dataclass
from enum import Enum
from math import isclose, isfinite, sqrt
from pathlib import Path

import yaml

from bringup.preflight_result import JsonDocument, JsonValue


@dataclass(frozen=True, slots=True)
class StaticTfProfile:
    """하나의 parent→child static transform과 사용 범위 ID다."""

    profile_id: str
    parent_frame: str
    child_frame: str
    translation_xyz_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class StaticTfProfileError(Exception):
    """Profile registry가 launch 경계에서 사용할 수 없음을 나타낸다."""

    reason_code: str
    detail: str = ""

    def __str__(self) -> str:
        return f"{self.reason_code}: {self.detail}" if self.detail else self.reason_code


class ExecutionMode(str, Enum):
    ONBOARD = "onboard"
    EXTERNAL_REPLAY = "external_replay"


def load_static_tf_profile(
    path: Path,
    profile_id: str,
    execution_mode: str,
) -> StaticTfProfile:
    """YAML의 지정 profile을 검증된 transform으로 변환한다."""
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise StaticTfProfileError("static_tf_profile_registry_unreadable", str(error)) from error
    mode = _parse_execution_mode(execution_mode)
    root = _required_mapping(document, "registry_root")
    registry = _required_mapping(root.get("static_tf_profiles"), "static_tf_profiles")
    profiles = _required_mapping(registry.get("profiles"), "profiles")
    raw_profile = profiles.get(profile_id)
    if raw_profile is None:
        raise StaticTfProfileError("unknown_static_tf_profile", profile_id)
    profile = _required_mapping(raw_profile, profile_id)
    _require_scope(_required_string(profile.get("scope"), "scope"), mode)
    translation = _required_vector(profile.get("translation_xyz_m"), 3, "translation")
    quaternion = _required_vector(profile.get("quaternion_xyzw"), 4, "quaternion")
    quaternion_norm = sqrt(sum(value * value for value in quaternion))
    if not isclose(quaternion_norm, 1.0, abs_tol=1e-6):
        raise StaticTfProfileError("static_tf_quaternion_not_normalized", profile_id)
    return StaticTfProfile(
        profile_id=profile_id,
        parent_frame=_required_string(profile.get("parent_frame"), "parent_frame"),
        child_frame=_required_string(profile.get("child_frame"), "child_frame"),
        translation_xyz_m=(translation[0], translation[1], translation[2]),
        quaternion_xyzw=(quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
    )


def _parse_execution_mode(value: str) -> ExecutionMode:
    try:
        return ExecutionMode(value)
    except ValueError as error:
        raise StaticTfProfileError(
            "unknown_static_tf_execution_mode",
            value,
        ) from error


def _require_scope(scope: str, mode: ExecutionMode) -> None:
    match scope:
        case "onboard_and_replay_default":
            return
        case "external_replay_only":
            if mode is not ExecutionMode.EXTERNAL_REPLAY:
                raise StaticTfProfileError(
                    "static_tf_profile_external_replay_required",
                    mode.value,
                )
        case _:
            raise StaticTfProfileError("unknown_static_tf_profile_scope", scope)


def _required_mapping(value: JsonValue, field_name: str) -> JsonDocument:
    if not isinstance(value, dict):
        raise StaticTfProfileError("static_tf_profile_mapping_invalid", field_name)
    return value


def _required_string(value: JsonValue, field_name: str) -> str:
    if not isinstance(value, str) or not value:
        raise StaticTfProfileError("static_tf_profile_string_invalid", field_name)
    return value


def _required_vector(value: JsonValue, length: int, field_name: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise StaticTfProfileError("static_tf_profile_vector_invalid", field_name)
    parsed: list[float] = []
    for component in value:
        if isinstance(component, bool) or not isinstance(component, (int, float)):
            raise StaticTfProfileError("static_tf_profile_component_invalid", field_name)
        numeric = float(component)
        if not isfinite(numeric):
            raise StaticTfProfileError("static_tf_profile_component_nonfinite", field_name)
        parsed.append(numeric)
    return tuple(parsed)
