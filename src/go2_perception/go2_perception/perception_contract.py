"""Pure geometry and candidate bounds for the stationary perception boundary.

This module does not access ROS or TF. Its transform argument is populated only
from the runtime TF lookup performed by the perception node.
"""

from collections.abc import Iterable
from dataclasses import dataclass
from math import hypot, isfinite
from typing import Final


INPUT_TOPIC: Final = "/utlidar/cloud"
SOURCE_FRAME_ID: Final = "utlidar_lidar"
OUTPUT_TOPIC: Final = "/perception/obstacle_candidates"
OUTPUT_FRAME_ID: Final = "base"


@dataclass(frozen=True, slots=True)
class CandidateBounds:
    """Inclusive base-frame limits for obstacle candidates, not final obstacles."""

    planar_range_m: tuple[float, float]
    z_m: tuple[float, float]


@dataclass(frozen=True, slots=True)
class PointXYZ:
    """One XYZ point expressed in the frame named by its containing contract."""

    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True)
class RigidTransform:
    """Translation and unit quaternion returned by a TF lookup."""

    translation_xyz: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]

    @classmethod
    def identity(cls) -> "RigidTransform":
        """Return the identity transform for pure geometry tests."""
        return cls(
            translation_xyz=(0.0, 0.0, 0.0),
            rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
        )


CANDIDATE_BOUNDS: Final = CandidateBounds(
    planar_range_m=(0.25, 5.0),
    z_m=(-0.25, 1.0),
)


def transform_and_filter_points(
    points: Iterable[PointXYZ],
    transform: RigidTransform,
) -> tuple[PointXYZ, ...]:
    """Transform source XYZ points and retain finite points inside candidate bounds."""
    candidates: list[PointXYZ] = []
    for point in points:
        transformed = _apply_transform(point, transform)
        if _is_obstacle_candidate(transformed):
            candidates.append(transformed)
    return tuple(candidates)


def _apply_transform(point: PointXYZ, transform: RigidTransform) -> PointXYZ:
    translation_x, translation_y, translation_z = transform.translation_xyz
    quaternion_x, quaternion_y, quaternion_z, quaternion_w = transform.rotation_xyzw
    xx = quaternion_x * quaternion_x
    yy = quaternion_y * quaternion_y
    zz = quaternion_z * quaternion_z
    xy = quaternion_x * quaternion_y
    xz = quaternion_x * quaternion_z
    yz = quaternion_y * quaternion_z
    wx = quaternion_w * quaternion_x
    wy = quaternion_w * quaternion_y
    wz = quaternion_w * quaternion_z
    return PointXYZ(
        x=((1.0 - (2.0 * (yy + zz))) * point.x)
        + ((2.0 * (xy - wz)) * point.y)
        + ((2.0 * (xz + wy)) * point.z)
        + translation_x,
        y=((2.0 * (xy + wz)) * point.x)
        + ((1.0 - (2.0 * (xx + zz))) * point.y)
        + ((2.0 * (yz - wx)) * point.z)
        + translation_y,
        z=((2.0 * (xz - wy)) * point.x)
        + ((2.0 * (yz + wx)) * point.y)
        + ((1.0 - (2.0 * (xx + yy))) * point.z)
        + translation_z,
    )


def _is_obstacle_candidate(point: PointXYZ) -> bool:
    if not all(isfinite(value) for value in (point.x, point.y, point.z)):
        return False
    minimum_range, maximum_range = CANDIDATE_BOUNDS.planar_range_m
    minimum_z, maximum_z = CANDIDATE_BOUNDS.z_m
    planar_range = hypot(point.x, point.y)
    return minimum_range <= planar_range <= maximum_range and minimum_z <= point.z <= maximum_z
