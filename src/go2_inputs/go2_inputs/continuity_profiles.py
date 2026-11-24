"""Parse immutable odometry continuity profiles from the installed YAML contract."""

from dataclasses import dataclass
from enum import Enum
from math import isfinite
from pathlib import Path
from typing import TypeAlias

import yaml


YamlValue: TypeAlias = None | bool | int | float | str | list["YamlValue"] | dict[str, "YamlValue"]


class ContinuityAction(str, Enum):
    """Specify whether a continuity failure is observed or suppresses output."""

    OBSERVE_ONLY = "observe_only"
    ENFORCE = "enforce"


@dataclass(frozen=True, slots=True)
class ContinuityProfile:
    """Validated limits and output action for one continuity evaluation profile."""

    profile_id: str
    action: ContinuityAction
    max_timestamp_gap_nanoseconds: int
    max_translation_delta_m: float
    max_yaw_delta_rad: float
    recovery_consecutive_valid_samples: int

    def suppresses_continuity_failure(self) -> bool:
        """Return whether this profile blocks continuity-failing source-valid samples."""
        return self.action is ContinuityAction.ENFORCE


@dataclass(frozen=True, slots=True)
class ContinuityProfileError(Exception):
    """Report a profile registry condition that prevents safe profile selection."""

    reason_code: str
    detail: str = ""

    def __str__(self) -> str:
        return f"{self.reason_code}: {self.detail}" if self.detail else self.reason_code


def load_continuity_profile(path: Path, profile_id: str) -> ContinuityProfile:
    """Load one fully validated continuity profile without fallback selection."""
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ContinuityProfileError("continuity_profiles_unreadable", str(error)) from error
    root = _required_mapping(document, "registry_root")
    project = _required_mapping(root.get("project"), "project")
    registry = _required_mapping(
        project.get("continuity_profiles"),
        "continuity_profiles",
    )
    profiles = _required_mapping(registry.get("profiles"), "profiles")
    raw_profile = profiles.get(profile_id)
    if raw_profile is None:
        raise ContinuityProfileError("unknown_continuity_profile", profile_id)
    profile = _required_mapping(raw_profile, profile_id)
    return ContinuityProfile(
        profile_id=profile_id,
        action=_required_action(profile.get("action")),
        max_timestamp_gap_nanoseconds=_required_positive_int(
            profile.get("max_timestamp_gap_nanoseconds"),
            "max_timestamp_gap_nanoseconds",
        ),
        max_translation_delta_m=_required_positive_float(
            profile.get("max_translation_delta_m"),
            "max_translation_delta_m",
        ),
        max_yaw_delta_rad=_required_positive_float(
            profile.get("max_yaw_delta_rad"),
            "max_yaw_delta_rad",
        ),
        recovery_consecutive_valid_samples=_required_positive_int(
            profile.get("recovery_consecutive_valid_samples"),
            "recovery_consecutive_valid_samples",
        ),
    )


def _required_mapping(value: YamlValue, field_name: str) -> dict[str, YamlValue]:
    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise ContinuityProfileError("continuity_profiles_mapping_invalid", field_name)
    return value


def _required_action(value: YamlValue) -> ContinuityAction:
    if not isinstance(value, str):
        raise ContinuityProfileError("continuity_profile_action_invalid")
    try:
        return ContinuityAction(value)
    except ValueError as error:
        raise ContinuityProfileError("continuity_profile_action_invalid", value) from error


def _required_positive_int(value: YamlValue, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ContinuityProfileError("continuity_profile_integer_invalid", field_name)
    return value


def _required_positive_float(value: YamlValue, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ContinuityProfileError("continuity_profile_number_invalid", field_name)
    numeric = float(value)
    if not isfinite(numeric) or numeric <= 0.0:
        raise ContinuityProfileError("continuity_profile_number_invalid", field_name)
    return numeric
