"""Pure validation for an observed odometry source; it never maps frames or publishes data."""

from dataclasses import dataclass, replace
from math import isfinite
from typing import Final


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
