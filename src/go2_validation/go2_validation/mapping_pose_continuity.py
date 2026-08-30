
"""공통 현재 `odom → base` pose에서 `map → odom` 보정량을 측정한다."""
from bisect import bisect_left
from dataclasses import dataclass
from math import atan2, cos, inf, isfinite, sin, sqrt
from typing import Final

from go2_validation.mapping_tf_continuity import (
    MAXIMUM_TRANSLATION_STEP_M,
    MAXIMUM_YAW_STEP_RAD,
)


SLAM_TRANSFORM_TIMEOUT_NS: Final = 200_000_000
ODOMETRY_ALIGNMENT_TOLERANCE_NS: Final = 50_000_000


@dataclass(frozen=True, slots=True)
class MappingCorrectionContinuityObservation:
    """공통 current odometry pose에서 측정한 map 보정량의 bounded 통계다."""

    sample_count: int
    maximum_translation_step_m: float
    maximum_yaw_step_rad: float
    translation_exceedance_count: int = 0
    yaw_exceedance_count: int = 0
    unaligned_sample_count: int = 0
    duplicate_stamp_count: int = 0
    regressive_stamp_count: int = 0
    maximum_translation_step_published_stamp_ns: int | None = None
    maximum_translation_step_effective_stamp_ns: int | None = None
    maximum_yaw_step_published_stamp_ns: int | None = None
    maximum_yaw_step_effective_stamp_ns: int | None = None


@dataclass(frozen=True, slots=True)
class _PlanarPose:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True, slots=True)
class _TimedPlanarPose:
    stamp_ns: int
    pose: _PlanarPose


def empty_mapping_correction_continuity_observation() -> MappingCorrectionContinuityObservation:
    """아직 map 보정을 측정하지 않은 기본 observation을 반환한다."""
    return MappingCorrectionContinuityObservation(0, 0.0, 0.0)


class MapCorrectionContinuityAccumulator:
    """각 map 보정 시각의 공통 odometry pose에서 이전·현재 map transform을 비교한다."""

    def __init__(self) -> None:
        self._odometry: list[_TimedPlanarPose] = []
        self._previous_map_to_odom: _PlanarPose | None = None
        self._last_received_effective_stamp_ns: int | None = None
        self._sample_count = 0
        self._maximum_translation_step_m = 0.0
        self._maximum_yaw_step_rad = 0.0
        self._translation_exceedance_count = 0
        self._yaw_exceedance_count = 0
        self._unaligned_sample_count = 0
        self._duplicate_stamp_count = 0
        self._regressive_stamp_count = 0
        self._maximum_translation_step_published_stamp_ns: int | None = None
        self._maximum_translation_step_effective_stamp_ns: int | None = None
        self._maximum_yaw_step_published_stamp_ns: int | None = None
        self._maximum_yaw_step_effective_stamp_ns: int | None = None

    def observe_odometry(
        self,
        translation_xyz_m: tuple[float, float, float],
        quaternion_xyzw: tuple[float, float, float, float],
        stamp_nanoseconds: int,
    ) -> None:
        """High-rate `odom → base` pose를 stamp order로 보관한다."""
        stamps = [sample.stamp_ns for sample in self._odometry]
        index = bisect_left(stamps, stamp_nanoseconds)
        self._odometry.insert(
            index,
            _TimedPlanarPose(
                stamp_nanoseconds,
                _planar_pose(translation_xyz_m, quaternion_xyzw),
            ),
        )

    def observe_map_to_odom(
        self,
        translation_xyz_m: tuple[float, float, float],
        quaternion_xyzw: tuple[float, float, float, float],
        published_stamp_nanoseconds: int,
    ) -> None:
        """SLAM published transform을 effective correction 시각에서 비교한다."""
        effective_stamp = published_stamp_nanoseconds - SLAM_TRANSFORM_TIMEOUT_NS
        current_map_to_odom = _planar_pose(translation_xyz_m, quaternion_xyzw)
        if (
            self._last_received_effective_stamp_ns == effective_stamp
            and self._previous_map_to_odom == current_map_to_odom
        ):
            self._duplicate_stamp_count += 1
            return
        if (
            self._last_received_effective_stamp_ns is not None
            and effective_stamp < self._last_received_effective_stamp_ns
        ):
            self._regressive_stamp_count += 1
            return
        self._last_received_effective_stamp_ns = effective_stamp
        if self._previous_map_to_odom is None:
            self._previous_map_to_odom = current_map_to_odom
            return
        odom_to_base = self._interpolated_odometry(effective_stamp)
        if odom_to_base is None:
            self._unaligned_sample_count += 1
            return
        self._observe_correction(
            self._previous_map_to_odom,
            current_map_to_odom,
            odom_to_base,
            published_stamp_nanoseconds,
            effective_stamp,
        )
        self._previous_map_to_odom = current_map_to_odom

    def observation(self) -> MappingCorrectionContinuityObservation:
        """현재 map correction bounded 통계를 immutable projection으로 반환한다."""
        return MappingCorrectionContinuityObservation(
            sample_count=self._sample_count,
            maximum_translation_step_m=self._maximum_translation_step_m,
            maximum_yaw_step_rad=self._maximum_yaw_step_rad,
            translation_exceedance_count=self._translation_exceedance_count,
            yaw_exceedance_count=self._yaw_exceedance_count,
            unaligned_sample_count=self._unaligned_sample_count,
            duplicate_stamp_count=self._duplicate_stamp_count,
            regressive_stamp_count=self._regressive_stamp_count,
            maximum_translation_step_published_stamp_ns=self._maximum_translation_step_published_stamp_ns,
            maximum_translation_step_effective_stamp_ns=self._maximum_translation_step_effective_stamp_ns,
            maximum_yaw_step_published_stamp_ns=self._maximum_yaw_step_published_stamp_ns,
            maximum_yaw_step_effective_stamp_ns=self._maximum_yaw_step_effective_stamp_ns,
        )

    def _interpolated_odometry(self, effective_stamp_ns: int) -> _PlanarPose | None:
        stamps = [sample.stamp_ns for sample in self._odometry]
        index = bisect_left(stamps, effective_stamp_ns)
        if index < len(self._odometry) and stamps[index] == effective_stamp_ns:
            return self._odometry[index].pose
        before = self._odometry[index - 1] if index > 0 else None
        after = self._odometry[index] if index < len(self._odometry) else None
        if before is None or after is None:
            return None
        if (
            effective_stamp_ns - before.stamp_ns > ODOMETRY_ALIGNMENT_TOLERANCE_NS
            or after.stamp_ns - effective_stamp_ns > ODOMETRY_ALIGNMENT_TOLERANCE_NS
        ):
            return None
        return _interpolate(before, after, effective_stamp_ns)

    def _observe_correction(
        self,
        previous_map_to_odom: _PlanarPose,
        current_map_to_odom: _PlanarPose,
        odom_to_base: _PlanarPose,
        published_stamp_ns: int,
        effective_stamp_ns: int,
    ) -> None:
        before_at_current_odom = _compose(previous_map_to_odom, odom_to_base)
        after_at_current_odom = _compose(current_map_to_odom, odom_to_base)
        translation_step = sqrt(
            (after_at_current_odom.x_m - before_at_current_odom.x_m) ** 2
            + (after_at_current_odom.y_m - before_at_current_odom.y_m) ** 2
        )
        yaw_step = _shortest_yaw_step(
            after_at_current_odom.yaw_rad,
            before_at_current_odom.yaw_rad,
        )
        self._sample_count += 1
        if translation_step > self._maximum_translation_step_m:
            self._maximum_translation_step_m = translation_step
            self._maximum_translation_step_published_stamp_ns = published_stamp_ns
            self._maximum_translation_step_effective_stamp_ns = effective_stamp_ns
        if yaw_step > self._maximum_yaw_step_rad:
            self._maximum_yaw_step_rad = yaw_step
            self._maximum_yaw_step_published_stamp_ns = published_stamp_ns
            self._maximum_yaw_step_effective_stamp_ns = effective_stamp_ns
        if translation_step > MAXIMUM_TRANSLATION_STEP_M:
            self._translation_exceedance_count += 1
        if yaw_step > MAXIMUM_YAW_STEP_RAD:
            self._yaw_exceedance_count += 1


def _planar_pose(
    translation_xyz_m: tuple[float, float, float],
    quaternion_xyzw: tuple[float, float, float, float],
) -> _PlanarPose:
    values = (*translation_xyz_m, *quaternion_xyzw)
    if not all(isfinite(value) for value in values):
        return _PlanarPose(inf, inf, inf)
    x, y, z, w = quaternion_xyzw
    if sqrt(sum(value * value for value in quaternion_xyzw)) == 0.0:
        return _PlanarPose(inf, inf, inf)
    return _PlanarPose(
        translation_xyz_m[0],
        translation_xyz_m[1],
        atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
    )


def _compose(map_to_odom: _PlanarPose, odom_to_base: _PlanarPose) -> _PlanarPose:
    cosine = cos(map_to_odom.yaw_rad)
    sine = sin(map_to_odom.yaw_rad)
    return _PlanarPose(
        map_to_odom.x_m + cosine * odom_to_base.x_m - sine * odom_to_base.y_m,
        map_to_odom.y_m + sine * odom_to_base.x_m + cosine * odom_to_base.y_m,
        map_to_odom.yaw_rad + odom_to_base.yaw_rad,
    )


def _interpolate(
    before: _TimedPlanarPose,
    after: _TimedPlanarPose,
    stamp_ns: int,
) -> _PlanarPose:
    ratio = (stamp_ns - before.stamp_ns) / (after.stamp_ns - before.stamp_ns)
    yaw_delta = atan2(
        sin(after.pose.yaw_rad - before.pose.yaw_rad),
        cos(after.pose.yaw_rad - before.pose.yaw_rad),
    )
    return _PlanarPose(
        before.pose.x_m + ratio * (after.pose.x_m - before.pose.x_m),
        before.pose.y_m + ratio * (after.pose.y_m - before.pose.y_m),
        before.pose.yaw_rad + ratio * yaw_delta,
    )


def _shortest_yaw_step(current: float, previous: float) -> float:
    return abs(atan2(sin(current - previous), cos(current - previous)))
