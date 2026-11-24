"""Pure suppression rules for the mapping cloud ingress boundary."""

from dataclasses import dataclass
from typing import Final


MAX_CLOUD_AGE_NANOSECONDS: Final = 1_000_000_000
REQUIRED_FIELD_NAMES: Final = frozenset({"x", "y", "z"})


@dataclass(frozen=True, slots=True)
class MappingCloudSample:
    stamp_nanoseconds: int
    width: int
    height: int
    point_step: int
    row_step: int
    data_length: int
    finite_point_count: int
    field_names: tuple[str, ...] = ("x", "y", "z")


@dataclass(frozen=True, slots=True)
class MappingCloudAssessment:
    publish: bool
    reason_code: str | None


def assess_mapping_cloud(
    sample: MappingCloudSample,
    now_nanoseconds: int,
    previous_stamp_nanoseconds: int | None = None,
) -> MappingCloudAssessment:
    """Suppress malformed, empty, NaN-only, and stale mapping input."""
    if sample.width == 0 or sample.height == 0:
        return MappingCloudAssessment(False, "empty_cloud")
    minimum_row_step = sample.width * sample.point_step
    expected_length = sample.height * sample.row_step
    if (
        sample.width < 0
        or sample.height < 0
        or sample.point_step <= 0
        or sample.row_step < minimum_row_step
        or sample.data_length != expected_length
        or not REQUIRED_FIELD_NAMES.issubset(sample.field_names)
    ):
        return MappingCloudAssessment(False, "malformed_layout")
    if sample.stamp_nanoseconds <= 0:
        return MappingCloudAssessment(False, "nonpositive_timestamp")
    if (
        previous_stamp_nanoseconds is not None
        and sample.stamp_nanoseconds < previous_stamp_nanoseconds
    ):
        return MappingCloudAssessment(False, "timestamp_regression")
    if sample.finite_point_count == 0:
        return MappingCloudAssessment(False, "nan_cloud")
    if now_nanoseconds - sample.stamp_nanoseconds > MAX_CLOUD_AGE_NANOSECONDS:
        return MappingCloudAssessment(False, "stale_cloud")
    return MappingCloudAssessment(True, None)
