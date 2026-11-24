from math import sqrt

import pytest

from go2_perception.perception_contract import (
    CANDIDATE_BOUNDS,
    INPUT_TOPIC,
    OUTPUT_FRAME_ID,
    OUTPUT_TOPIC,
    SOURCE_FRAME_ID,
    PointXYZ,
    RigidTransform,
    transform_and_filter_points,
)


def test_given_stationary_contract_when_inspected_then_uses_observed_input_and_project_output() -> None:
    assert INPUT_TOPIC == "/utlidar/cloud"
    assert SOURCE_FRAME_ID == "utlidar_lidar"
    assert OUTPUT_TOPIC == "/perception/obstacle_candidates"
    assert OUTPUT_FRAME_ID == "base"
    assert CANDIDATE_BOUNDS.planar_range_m == (0.25, 5.0)
    assert CANDIDATE_BOUNDS.z_m == (-0.25, 1.0)


def test_given_lidar_point_when_transformed_then_applies_lookup_transform_geometry() -> None:
    transformed = transform_and_filter_points(
        points=(PointXYZ(1.0, 0.0, 0.0),),
        transform=RigidTransform(
            translation_xyz=(0.5, -0.5, 0.25),
            rotation_xyzw=(0.0, 0.0, sqrt(0.5), sqrt(0.5)),
        ),
    )

    assert len(transformed) == 1
    assert transformed[0].x == pytest.approx(0.5)
    assert transformed[0].y == pytest.approx(0.5)
    assert transformed[0].z == pytest.approx(0.25)


def test_given_points_on_or_outside_candidate_limits_when_filtered_then_keeps_only_finite_inclusive_candidates() -> None:
    transformed = transform_and_filter_points(
        points=(
            PointXYZ(0.25, 0.0, -0.25),
            PointXYZ(5.0, 0.0, 1.0),
            PointXYZ(0.249, 0.0, 0.0),
            PointXYZ(5.001, 0.0, 0.0),
            PointXYZ(1.0, 0.0, -0.251),
            PointXYZ(1.0, 0.0, 1.001),
            PointXYZ(float("nan"), 0.0, 0.0),
        ),
        transform=RigidTransform.identity(),
    )

    assert transformed == (
        PointXYZ(0.25, 0.0, -0.25),
        PointXYZ(5.0, 0.0, 1.0),
    )
