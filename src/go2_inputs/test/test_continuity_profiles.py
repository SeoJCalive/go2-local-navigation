from dataclasses import replace
from pathlib import Path
from typing import Final

import pytest
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from launch_test_support import evaluate



from go2_inputs.odometry_contract import (
    ContinuityState,
    OdometrySample,
    advance_continuity,
)


PACKAGE_ROOT: Final = Path(__file__).parents[1]
PROFILE_PATH: Final = PACKAGE_ROOT / "config" / "odometry_contract.yaml"
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


def test_hand_computed_three_four_five_translation_boundary() -> None:
    from go2_inputs.continuity_profiles import ContinuityAction, ContinuityProfile
    # 0.3^2 + 0.4^2 = 0.5^2. Exercise the caller's supplied limit either side.
    sample = replace(OBSERVED_SAMPLE, timestamp_nanoseconds=1_010_000_000,
                     position_xyz=(0.3, 0.4, 0.0))
    for limit, valid in [(0.49, False), (0.51, True)]:
        profile = ContinuityProfile("geometry", ContinuityAction.ENFORCE,
                                    100_000_000, limit, 1.0, 1)
        result = advance_continuity(ContinuityState(previous_sample=OBSERVED_SAMPLE),
                                    sample, observed_at_nanoseconds=1_010_000_000,
                                    profile=profile)
        assert result.assessment.continuity_valid is valid
        assert result.assessment.publish is valid


def test_given_unknown_or_malformed_profile_when_loaded_then_rejects_without_fallback(
    tmp_path: Path,
) -> None:
    from go2_inputs.continuity_profiles import (
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
    from go2_inputs.continuity_profiles import load_continuity_profile

    # Given: the same source-valid translation jump under both profiles.
    observe = load_continuity_profile(PROFILE_PATH, "onboard_observe")
    enforce = load_continuity_profile(PROFILE_PATH, "validation_enforce")
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


def test_odometry_launch_evaluates_caller_profile() -> None:
    _, nodes, _ = evaluate("go2_inputs", "go2_odometry_adapter.launch.py",
                           {"continuity_profile": "validation_enforce"})
    assert len(nodes) == 1
    parameters = {key: value for group in nodes[0]["parameters"]
                  for key, value in group.items()}
    assert parameters["continuity_profile"] == "validation_enforce"
