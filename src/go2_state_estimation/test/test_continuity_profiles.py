from dataclasses import replace
from pathlib import Path
from typing import Final

import pytest
import yaml

from go2_state_estimation.odometry_contract import (
    ContinuityState,
    OdometrySample,
    advance_continuity,
)


PACKAGE_ROOT: Final = Path(__file__).parents[1]
PROFILE_PATH: Final = PACKAGE_ROOT / "config" / "odometry_contract.yaml"
LAUNCH_PATH: Final = PACKAGE_ROOT.parent / "bringup" / "launch" / "go2_odometry_adapter.launch.py"
OBSERVED_SAMPLE: Final = OdometrySample(
    timestamp_nanoseconds=1_000_000_000,
    header_frame_id="odom",
    child_frame_id="base_link",
    position_xyz=(0.0, 0.0, 0.0),
    orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    linear_velocity_xyz=(0.0, 0.0, 0.0),
    angular_velocity_xyz=(0.0, 0.0, 0.0),
    pose_covariance=(0.0,) * 36,
    twist_covariance=(0.0,) * 36,
)


def test_given_contract_yaml_when_profiles_are_read_then_default_actions_and_limits_are_exact() -> None:
    # Given: the installed source contract YAML
    document = yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8"))

    # When: continuity profile values are read at their machine-consumed location
    registry = document["project"]["continuity_profiles"]
    observe = registry["profiles"]["onboard_observe"]
    enforce = registry["profiles"]["replay_enforce"]

    # Then: onboard use observes and replay use enforces the same candidate limits.
    assert registry["default_profile"] == "onboard_observe"
    assert observe["action"] == "observe_only"
    assert enforce["action"] == "enforce"
    assert observe["max_timestamp_gap_nanoseconds"] == 500_000_000
    assert observe["max_translation_delta_m"] == 0.5
    assert observe["max_yaw_delta_rad"] == 0.5
    assert observe["recovery_consecutive_valid_samples"] == 2
    assert enforce == observe | {"action": "enforce"}


def test_given_unknown_or_malformed_profile_when_loaded_then_rejects_without_fallback(
    tmp_path: Path,
) -> None:
    from go2_state_estimation.continuity_profiles import (
        ContinuityProfileError,
        load_continuity_profile,
    )

    # Given: a known registry and a malformed replacement registry.
    malformed_path = tmp_path / "malformed.yaml"
    malformed_path.write_text(
        "project:\n  continuity_profiles:\n    default_profile: onboard_observe\n",
        encoding="utf-8",
    )

    # When/Then: neither case silently selects an unsafe fallback.
    with pytest.raises(ContinuityProfileError, match="unknown_continuity_profile"):
        load_continuity_profile(PROFILE_PATH, "missing_profile")
    with pytest.raises(ContinuityProfileError, match="continuity_profiles_mapping_invalid"):
        load_continuity_profile(malformed_path, "onboard_observe")


def test_given_continuity_faults_when_actions_differ_then_transitions_match_and_only_enforce_suppresses() -> None:
    from go2_state_estimation.continuity_profiles import load_continuity_profile

    # Given: the same source-valid translation jump under both profiles.
    observe = load_continuity_profile(PROFILE_PATH, "onboard_observe")
    enforce = load_continuity_profile(PROFILE_PATH, "replay_enforce")
    jump = replace(
        OBSERVED_SAMPLE,
        timestamp_nanoseconds=1_020_000_000,
        position_xyz=(1.0, 0.0, 0.0),
    )

    # When: one shared evaluator advances each profile from the same state.
    observed = advance_continuity(
        ContinuityState(previous_sample=OBSERVED_SAMPLE),
        jump,
        observed_at_nanoseconds=1_020_000_000,
        profile=observe,
    )
    enforced = advance_continuity(
        ContinuityState(previous_sample=OBSERVED_SAMPLE),
        jump,
        observed_at_nanoseconds=1_020_000_000,
        profile=enforce,
    )

    # Then: violation metrics and recovery state agree, but only enforce blocks output.
    assert observed.assessment.reason_code == "translation_jump"
    assert enforced.assessment.reason_code == "translation_jump"
    assert observed.assessment.continuity_valid is False
    assert enforced.assessment.continuity_valid is False
    assert observed.state == enforced.state
    assert observed.assessment.publish is True
    assert enforced.assessment.publish is False


def test_given_odometry_launch_when_read_then_default_profile_is_passed_as_a_typed_parameter() -> None:
    launch_source = LAUNCH_PATH.read_text(encoding="utf-8")

    assert '"continuity_profile"' in launch_source
    assert 'default_value="onboard_observe"' in launch_source
    assert "value_type=str" in launch_source
