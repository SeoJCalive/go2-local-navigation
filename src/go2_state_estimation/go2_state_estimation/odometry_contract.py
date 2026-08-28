"""Pure validation for an observed odometry source; it never maps frames or publishes data."""

from dataclasses import dataclass, replace
from math import atan2, cos, dist, isfinite, sin
from typing import Final

from go2_state_estimation.continuity_profiles import ContinuityProfile


ADAPTER_STATUS_BLOCKED: Final = "blocked_until_frame_mapping_verified"
ADAPTER_STATUS_PROJECT_ACCEPTED: Final = "project_accepted_base_link_to_base"
EXPECTED_HEADER_FRAME_ID: Final = "odom"
EXPECTED_SOURCE_CHILD_FRAME_ID: Final = "base_link"
PROJECT_TARGET_FRAME_ID: Final = "base"


@dataclass(frozen=True, slots=True)
class OdometrySample:
    """The ROS-independent odometry fields required by the source contract."""

    timestamp_nanoseconds: int
    header_frame_id: str
    child_frame_id: str
    position_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    linear_velocity_xyz: tuple[float, float, float]
    angular_velocity_xyz: tuple[float, float, float]
    pose_covariance: tuple[float, ...]
    twist_covariance: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class ContractAssessment:
    """Validation output that preserves the source child frame without adapting it."""

    source_child_frame_id: str
    adapter_status: str
    timestamp_accepted: bool
    errors: tuple[str, ...]
    warnings: tuple[str, ...]
    target_child_frame_id: str | None = None


@dataclass(frozen=True, slots=True)
class ContinuityAssessment:
    """A non-publishing decision for one odometry continuity sample."""

    publish: bool
    reason_code: str | None
    continuity_valid: bool


@dataclass(frozen=True, slots=True)
class ContinuityState:
    """Odometry fault 이후 연속 recovery를 판정하는 불변 상태다."""

    previous_sample: OdometrySample | None = None
    recovery_required: bool = False
    consecutive_valid_samples: int = 0
    last_received_at_nanoseconds: int | None = None


@dataclass(frozen=True, slots=True)
class ContinuityTransition:
    """한 sample의 publish 판정과 다음 continuity 상태다."""

    state: ContinuityState
    assessment: ContinuityAssessment


def validate_odometry_sample(
    sample: OdometrySample,
    previous_timestamp_nanoseconds: int | None,
) -> ContractAssessment:
    """Validate one source sample without treating its child frame as project ``base``."""
    errors: list[str] = []
    warnings: list[str] = []

    if sample.header_frame_id != EXPECTED_HEADER_FRAME_ID:
        errors.append("header_frame_id_must_be_odom")
    if sample.child_frame_id != EXPECTED_SOURCE_CHILD_FRAME_ID:
        errors.append("child_frame_id_must_be_base_link")
    if sample.timestamp_nanoseconds <= 0:
        errors.append("timestamp_must_be_positive")
    if (
        previous_timestamp_nanoseconds is not None
        and sample.timestamp_nanoseconds < previous_timestamp_nanoseconds
    ):
        errors.append("timestamp_must_be_nondecreasing")
    if not _motion_is_finite(sample):
        errors.append("pose_or_twist_must_be_finite")
    if _all_zero(sample.pose_covariance):
        warnings.append("pose_covariance_all_zero")
    if _all_zero(sample.twist_covariance):
        warnings.append("twist_covariance_all_zero")

    timestamp_accepted = (
        sample.timestamp_nanoseconds > 0
        and (
            previous_timestamp_nanoseconds is None
            or sample.timestamp_nanoseconds >= previous_timestamp_nanoseconds
        )
    )
    return ContractAssessment(
        source_child_frame_id=sample.child_frame_id,
        adapter_status=ADAPTER_STATUS_BLOCKED,
        timestamp_accepted=timestamp_accepted,
        errors=tuple(errors),
        warnings=tuple(warnings),
    )


def assess_odometry_adapter(
    sample: OdometrySample,
    previous_timestamp_nanoseconds: int | None,
) -> ContractAssessment:
    """Apply the project-accepted ``base_link`` to ``base`` mapping gate."""
    assessment = validate_odometry_sample(sample, previous_timestamp_nanoseconds)
    if assessment.errors:
        return assessment
    return replace(
        assessment,
        adapter_status=ADAPTER_STATUS_PROJECT_ACCEPTED,
        target_child_frame_id=PROJECT_TARGET_FRAME_ID,
    )


def advance_continuity(
    state: ContinuityState,
    sample: OdometrySample,
    observed_at_nanoseconds: int,
    profile: ContinuityProfile,
) -> ContinuityTransition:
    """한 sample을 판정하고 fault recovery 상태를 전진시킨다."""
    assessment = _assess_sample_continuity(
        sample,
        state.previous_sample,
        profile,
    )
    if assessment.reason_code is not None:
        recovery_baseline = (
            sample
            if assessment.reason_code == "stale_odometry"
            else state.previous_sample
        )
        return ContinuityTransition(
            state=ContinuityState(
                previous_sample=recovery_baseline,
                recovery_required=True,
                consecutive_valid_samples=0,
                last_received_at_nanoseconds=observed_at_nanoseconds,
            ),
            assessment=_output_assessment(profile, False, assessment.reason_code),
        )
    if not state.recovery_required:
        return ContinuityTransition(
            state=ContinuityState(
                previous_sample=sample,
                last_received_at_nanoseconds=observed_at_nanoseconds,
            ),
            assessment=_output_assessment(profile, True, None),
        )
    valid_samples = state.consecutive_valid_samples + 1
    recovered = valid_samples >= profile.recovery_consecutive_valid_samples
    return ContinuityTransition(
        state=ContinuityState(
            previous_sample=sample,
            recovery_required=not recovered,
            consecutive_valid_samples=0 if recovered else valid_samples,
            last_received_at_nanoseconds=observed_at_nanoseconds,
        ),
        assessment=(
            _output_assessment(profile, True, None)
            if recovered
            else _output_assessment(profile, False, "recovery_pending")
        ),
    )


def assess_odometry_loss(
    state: ContinuityState,
    observed_at_nanoseconds: int,
    profile: ContinuityProfile,
) -> ContinuityAssessment:
    """마지막 수신 시각이 timeout을 넘겼는지 판정한다."""
    if state.last_received_at_nanoseconds is None:
        return _output_assessment(profile, True, None)
    age = observed_at_nanoseconds - state.last_received_at_nanoseconds
    if age > profile.max_timestamp_gap_nanoseconds:
        return _output_assessment(profile, False, "stale_odometry")
    return _output_assessment(profile, True, None)


def mark_odometry_loss(state: ContinuityState) -> ContinuityState:
    """Loss가 발생한 stream을 연속 recovery 대기 상태로 바꾼다."""
    return replace(
        state,
        previous_sample=None,
        recovery_required=True,
        consecutive_valid_samples=0,
    )


def _assess_sample_continuity(
    sample: OdometrySample,
    previous: OdometrySample | None,
    profile: ContinuityProfile,
) -> ContinuityAssessment:
    if not _motion_is_finite(sample):
        return _raw_assessment(False, "nonfinite_odometry")
    if previous is None:
        return _raw_assessment(True, None)
    if sample.timestamp_nanoseconds < previous.timestamp_nanoseconds:
        return _raw_assessment(False, "timestamp_regression")
    if (
        sample.timestamp_nanoseconds - previous.timestamp_nanoseconds
        > profile.max_timestamp_gap_nanoseconds
    ):
        return _raw_assessment(False, "stale_odometry")
    if dist(sample.position_xyz, previous.position_xyz) > profile.max_translation_delta_m:
        return _raw_assessment(False, "translation_jump")
    if _yaw_delta(sample.orientation_xyzw, previous.orientation_xyzw) > profile.max_yaw_delta_rad:
        return _raw_assessment(False, "yaw_jump")
    return _raw_assessment(True, None)


def _raw_assessment(
    continuity_valid: bool,
    reason_code: str | None,
) -> ContinuityAssessment:
    return ContinuityAssessment(
        publish=continuity_valid,
        reason_code=reason_code,
        continuity_valid=continuity_valid,
    )


def _output_assessment(
    profile: ContinuityProfile,
    continuity_valid: bool,
    reason_code: str | None,
) -> ContinuityAssessment:
    return ContinuityAssessment(
        publish=continuity_valid or not profile.suppresses_continuity_failure(),
        reason_code=reason_code,
        continuity_valid=continuity_valid,
    )


def _yaw_delta(
    current_xyzw: tuple[float, float, float, float],
    previous_xyzw: tuple[float, float, float, float],
) -> float:
    current = _yaw(current_xyzw)
    previous = _yaw(previous_xyzw)
    return abs(atan2(sin(current - previous), cos(current - previous)))


def _yaw(orientation_xyzw: tuple[float, float, float, float]) -> float:
    x, y, z, w = orientation_xyzw
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _motion_is_finite(sample: OdometrySample) -> bool:
    return all(
        isfinite(value)
        for value in (
            sample.position_xyz
            + sample.orientation_xyzw
            + sample.linear_velocity_xyz
            + sample.angular_velocity_xyz
        )
    )


def _all_zero(covariance: tuple[float, ...]) -> bool:
    return bool(covariance) and all(value == 0.0 for value in covariance)
