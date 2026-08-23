from dataclasses import replace

from go2_state_estimation.odometry_contract import (
    ADAPTER_STATUS_BLOCKED,
    ADAPTER_STATUS_PROJECT_ACCEPTED,
    OdometrySample,
    assess_odometry_adapter,
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
