"""
`map → odom` transform의 연속성을 bounded 통계로 누적한다.

TF message payload나 전체 trajectory를 보관하지 않고 sample 수와 연속 두 sample의
최대 3D translation·최단 yaw step만 남긴다. 이 값은 gross jump screening용이며
지도 정확도나 물리 위치 ground truth를 판정하지 않는다.

"""
from dataclasses import dataclass
from math import atan2, inf, isfinite, sin, cos, sqrt

from geometry_msgs.msg import TransformStamped


MAXIMUM_TRANSLATION_STEP_M = 0.5
MAXIMUM_YAW_STEP_RAD = 0.2


@dataclass(frozen=True, slots=True)
class MappingTfContinuityObservation:
    """한 mapping run에서 관찰한 global TF 연속성 최대값이다."""

    sample_count: int
    maximum_translation_step_m: float
    maximum_yaw_step_rad: float
    translation_exceedance_count: int = 0
    yaw_exceedance_count: int = 0
    maximum_translation_step_stamp_ns: int | None = None
    maximum_yaw_step_stamp_ns: int | None = None


def empty_mapping_tf_continuity_observation() -> MappingTfContinuityObservation:
    """아직 pose를 받지 않은 기본 연속성 관찰값을 반환한다."""
    return MappingTfContinuityObservation(0, 0.0, 0.0)


class MappingTfContinuityAccumulator:
    """ROS callback 순서대로 들어오는 transform의 이전 값과 최대 step을 갱신한다."""

    def __init__(self) -> None:
        self._sample_count = 0
        self._previous_translation: tuple[float, float, float] | None = None
        self._previous_yaw: float | None = None
        self._maximum_translation_step_m = 0.0
        self._maximum_yaw_step_rad = 0.0
        self._translation_exceedance_count = 0
        self._yaw_exceedance_count = 0
        self._maximum_translation_step_stamp_ns: int | None = None
        self._maximum_yaw_step_stamp_ns: int | None = None

    def observe(
        self,
        translation_xyz_m: tuple[float, float, float],
        quaternion_xyzw: tuple[float, float, float, float],
        stamp_nanoseconds: int | None = None,
    ) -> None:
        """하나의 transform을 받아 이전 sample과의 step을 반영한다."""
        values = (*translation_xyz_m, *quaternion_xyzw)
        quaternion_norm = sqrt(sum(value * value for value in quaternion_xyzw))
        self._sample_count += 1
        if not all(isfinite(value) for value in values) or quaternion_norm == 0.0:
            self._maximum_translation_step_m = inf
            self._maximum_yaw_step_rad = inf
            return
        x, y, z, w = quaternion_xyzw
        yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        if self._previous_translation is not None and self._previous_yaw is not None:
            delta = tuple(
                current - previous
                for current, previous in zip(
                    translation_xyz_m,
                    self._previous_translation,
                )
            )
            translation_step = sqrt(sum(component * component for component in delta))
            yaw_delta = yaw - self._previous_yaw
            yaw_step = abs(atan2(sin(yaw_delta), cos(yaw_delta)))
            if translation_step > self._maximum_translation_step_m:
                self._maximum_translation_step_m = translation_step
                self._maximum_translation_step_stamp_ns = stamp_nanoseconds
            if yaw_step > self._maximum_yaw_step_rad:
                self._maximum_yaw_step_rad = yaw_step
                self._maximum_yaw_step_stamp_ns = stamp_nanoseconds
            if translation_step > MAXIMUM_TRANSLATION_STEP_M:
                self._translation_exceedance_count += 1
            if yaw_step > MAXIMUM_YAW_STEP_RAD:
                self._yaw_exceedance_count += 1
        self._previous_translation = translation_xyz_m
        self._previous_yaw = yaw

    def observe_transform(self, transform: TransformStamped) -> None:
        """ROS TransformStamped의 translation과 quaternion만 누적한다."""
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.observe(
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
            stamp_nanoseconds=(
                transform.header.stamp.sec * 1_000_000_000
                + transform.header.stamp.nanosec
            ),
        )

    def observation(self) -> MappingTfContinuityObservation:
        """현재 bounded 통계를 불변 observation으로 반환한다."""
        return MappingTfContinuityObservation(
            sample_count=self._sample_count,
            maximum_translation_step_m=self._maximum_translation_step_m,
            maximum_yaw_step_rad=self._maximum_yaw_step_rad,
            translation_exceedance_count=self._translation_exceedance_count,
            yaw_exceedance_count=self._yaw_exceedance_count,
            maximum_translation_step_stamp_ns=(
                self._maximum_translation_step_stamp_ns
            ),
            maximum_yaw_step_stamp_ns=self._maximum_yaw_step_stamp_ns,
        )
