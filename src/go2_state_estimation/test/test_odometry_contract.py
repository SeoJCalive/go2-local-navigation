from dataclasses import replace
from pathlib import Path
from typing import Final

from go2_state_estimation.continuity_profiles import load_continuity_profile
from go2_state_estimation.odometry_contract import (
    ADAPTER_STATUS_BLOCKED,
    ADAPTER_STATUS_PROJECT_ACCEPTED,
    ContinuityState,
    OdometrySample,
    advance_continuity,
    assess_odometry_adapter,
    assess_odometry_loss,
    validate_odometry_sample,
)


OBSERVED_SAMPLE = OdometrySample(
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
PROFILE_PATH: Final = Path(__file__).parents[1] / "config" / "odometry_contract.yaml"
ENFORCE_PROFILE = load_continuity_profile(PROFILE_PATH, "replay_enforce")


def test_given_observed_source_when_validated_then_keeps_child_and_blocks_adapter() -> None:
    assessment = validate_odometry_sample(OBSERVED_SAMPLE, previous_timestamp_nanoseconds=None)

    assert assessment.errors == ()
    assert assessment.source_child_frame_id == "base_link"
    assert assessment.adapter_status == ADAPTER_STATUS_BLOCKED
    assert "pose_covariance_all_zero" in assessment.warnings
    assert "twist_covariance_all_zero" in assessment.warnings


def test_given_wrong_header_or_nonpositive_timestamp_when_validated_then_reports_errors() -> None:
    assessment = validate_odometry_sample(
        replace(OBSERVED_SAMPLE, header_frame_id="map", timestamp_nanoseconds=0),
        previous_timestamp_nanoseconds=None,
    )

    assert "header_frame_id_must_be_odom" in assessment.errors
    assert "timestamp_must_be_positive" in assessment.errors


def test_given_decreasing_timestamp_when_validated_then_reports_error() -> None:
    assessment = validate_odometry_sample(
        replace(OBSERVED_SAMPLE, timestamp_nanoseconds=9),
        previous_timestamp_nanoseconds=10,
    )

    assert "timestamp_must_be_nondecreasing" in assessment.errors


def test_given_nonfinite_pose_or_twist_when_validated_then_reports_error() -> None:
    assessment = validate_odometry_sample(
        replace(OBSERVED_SAMPLE, linear_velocity_xyz=(float("inf"), 0.0, 0.0)),
        previous_timestamp_nanoseconds=None,
    )

    assert "pose_or_twist_must_be_finite" in assessment.errors


def test_given_accepted_source_frame_when_adapted_then_targets_project_base() -> None:
    assessment = assess_odometry_adapter(OBSERVED_SAMPLE, previous_timestamp_nanoseconds=None)

    assert assessment.errors == ()
    assert assessment.adapter_status == ADAPTER_STATUS_PROJECT_ACCEPTED
    assert assessment.target_child_frame_id == "base"
    assert assessment.timestamp_accepted


def test_given_unexpected_source_child_when_adapted_then_rejects_output() -> None:
    assessment = assess_odometry_adapter(
        replace(OBSERVED_SAMPLE, child_frame_id="other_frame"),
        previous_timestamp_nanoseconds=None,
    )

    assert "child_frame_id_must_be_base_link" in assessment.errors
    assert assessment.adapter_status == ADAPTER_STATUS_BLOCKED
    assert assessment.target_child_frame_id is None


def test_given_regression_jump_and_loss_when_continuity_assessed_then_suppresses_with_reasons() -> None:
    """Continuity rejects source discontinuities before the adapter publishes."""
    # Given: a previously accepted source sample
    previous = OBSERVED_SAMPLE
    regression = replace(OBSERVED_SAMPLE, timestamp_nanoseconds=999_999_999)
    jump = replace(OBSERVED_SAMPLE, timestamp_nanoseconds=1_020_000_000, position_xyz=(5.0, 0.0, 0.0))

    # When: discontinuities and a stale stream are assessed
    results = (
        advance_continuity(
            ContinuityState(previous_sample=previous),
            regression,
            1_000_000_000,
            ENFORCE_PROFILE,
        ).assessment,
        advance_continuity(
            ContinuityState(previous_sample=previous),
            jump,
            1_020_000_000,
            ENFORCE_PROFILE,
        ).assessment,
        assess_odometry_loss(
            ContinuityState(
                previous_sample=previous,
                last_received_at_nanoseconds=1_000_000_000,
            ),
            2_000_000_000,
            ENFORCE_PROFILE,
        ),
    )

    # Then: the adapter must suppress them with distinct reason codes
    assert [(result.publish, result.reason_code) for result in results] == [
        (False, "timestamp_regression"),
        (False, "translation_jump"),
        (False, "stale_odometry"),
    ]


def test_given_consecutive_valid_recovery_when_continuity_assessed_then_releases_output() -> None:
    """A recovered stream requires consecutive valid samples before release."""
    # Given: two continuous samples after a loss
    first = replace(OBSERVED_SAMPLE, timestamp_nanoseconds=1_020_000_000)
    second = replace(OBSERVED_SAMPLE, timestamp_nanoseconds=1_040_000_000, position_xyz=(0.01, 0.0, 0.0))

    # When: the recovery sequence is assessed
    faulted = ContinuityState(previous_sample=OBSERVED_SAMPLE, recovery_required=True)
    first_result = advance_continuity(faulted, first, 1_020_000_000, ENFORCE_PROFILE)
    second_result = advance_continuity(
        first_result.state,
        second,
        1_040_000_000,
        ENFORCE_PROFILE,
    )

    # Then: only the required consecutive recovery releases output
    assert not first_result.assessment.publish
    assert first_result.assessment.reason_code == "recovery_pending"
    assert second_result.assessment.publish
    assert second_result.assessment.reason_code is None


def test_given_single_sample_yaw_jump_when_advanced_then_output_is_suppressed() -> None:
    state = ContinuityState(previous_sample=OBSERVED_SAMPLE)
    yaw_jump = replace(
        OBSERVED_SAMPLE,
        timestamp_nanoseconds=1_020_000_000,
        orientation_xyzw=(0.0, 0.0, 0.70710678, 0.70710678),
    )

    transition = advance_continuity(
        state,
        yaw_jump,
        observed_at_nanoseconds=1_020_000_000,
        profile=ENFORCE_PROFILE,
    )

    assert not transition.assessment.publish
    assert transition.assessment.reason_code == "yaw_jump"
    assert transition.state.recovery_required


def test_given_faulted_stream_when_two_valid_samples_arrive_then_second_recovers() -> None:
    faulted = ContinuityState(
        previous_sample=OBSERVED_SAMPLE,
        recovery_required=True,
    )
    first = replace(OBSERVED_SAMPLE, timestamp_nanoseconds=1_020_000_000)
    second = replace(
        OBSERVED_SAMPLE,
        timestamp_nanoseconds=1_040_000_000,
        position_xyz=(0.01, 0.0, 0.0),
    )

    first_transition = advance_continuity(
        faulted,
        first,
        observed_at_nanoseconds=1_020_000_000,
        profile=ENFORCE_PROFILE,
    )
    second_transition = advance_continuity(
        first_transition.state,
        second,
        observed_at_nanoseconds=1_040_000_000,
        profile=ENFORCE_PROFILE,
    )

    assert first_transition.assessment.reason_code == "recovery_pending"
    assert second_transition.assessment.publish
    assert not second_transition.state.recovery_required


def test_given_host_and_source_clock_epochs_differ_when_sample_arrives_then_it_is_not_stale() -> None:
    source_epoch_sample = replace(
        OBSERVED_SAMPLE,
        timestamp_nanoseconds=1_020_000_000,
    )

    transition = advance_continuity(
        ContinuityState(previous_sample=OBSERVED_SAMPLE),
        source_epoch_sample,
        observed_at_nanoseconds=1_900_000_000_000_000_000,
        profile=ENFORCE_PROFILE,
    )

    assert transition.assessment.publish


def test_given_source_timestamp_gap_when_sample_arrives_then_it_is_suppressed() -> None:
    delayed = replace(
        OBSERVED_SAMPLE,
        timestamp_nanoseconds=1_600_000_001,
    )

    transition = advance_continuity(
        ContinuityState(previous_sample=OBSERVED_SAMPLE),
        delayed,
        observed_at_nanoseconds=2_000_000_000,
        profile=ENFORCE_PROFILE,
    )

    assert not transition.assessment.publish
    assert transition.assessment.reason_code == "stale_odometry"
    assert transition.state.previous_sample == delayed
