import unittest

from go2_sensor.lidar_contract import (
    CloudLayout,
    CloudSample,
    validate_cloud_sample,
)


def _sample(
    *,
    frame_id: str = "utlidar_lidar",
    field_names: tuple[str, ...] = ("x", "y", "z", "intensity"),
    stamp_nanoseconds: int = 1,
) -> CloudSample:
    return CloudSample(
        frame_id=frame_id,
        stamp_nanoseconds=stamp_nanoseconds,
        layout=CloudLayout(
            height=1,
            width=10,
            point_step=16,
            field_names=field_names,
        ),
    )


class LidarContractTests(unittest.TestCase):
    def test_validate_cloud_sample_accepts_matching_contract(self) -> None:
        # Given: expected frame, fields, and positive timestamp
        sample = _sample()

        # When: the acceptance boundary validates the sample
        result = validate_cloud_sample(sample, previous_stamp_nanoseconds=None)

        # Then: it is accepted
        self.assertTrue(result.is_valid)
        self.assertIsNone(result.reason)

    def test_validate_cloud_sample_rejects_unexpected_frame(self) -> None:
        # Given: a cloud in another frame
        sample = _sample(frame_id="base_link")

        # When: the acceptance boundary validates the sample
        result = validate_cloud_sample(sample, previous_stamp_nanoseconds=None)

        # Then: it is rejected
        self.assertFalse(result.is_valid)
        self.assertEqual(result.reason, "unexpected frame_id")

    def test_validate_cloud_sample_rejects_missing_xyz_field(self) -> None:
        # Given: a cloud layout without z
        sample = _sample(field_names=("x", "y", "intensity"))

        # When: the acceptance boundary validates the sample
        result = validate_cloud_sample(sample, previous_stamp_nanoseconds=None)

        # Then: it is rejected
        self.assertFalse(result.is_valid)
        self.assertEqual(result.reason, "missing required point fields")

    def test_validate_cloud_sample_rejects_nonpositive_timestamp(self) -> None:
        # Given: a cloud with an unset ROS timestamp
        sample = _sample(stamp_nanoseconds=0)

        # When: the acceptance boundary validates the sample
        result = validate_cloud_sample(sample, previous_stamp_nanoseconds=None)

        # Then: it is rejected
        self.assertFalse(result.is_valid)
        self.assertEqual(result.reason, "timestamp must be positive")

    def test_validate_cloud_sample_rejects_timestamp_regression(self) -> None:
        # Given: a cloud older than the preceding sample
        sample = _sample(stamp_nanoseconds=9)

        # When: the acceptance boundary validates the sample
        result = validate_cloud_sample(sample, previous_stamp_nanoseconds=10)

        # Then: it is rejected
        self.assertFalse(result.is_valid)
        self.assertEqual(result.reason, "timestamp regressed")

    def test_validate_cloud_sample_accepts_equal_timestamp(self) -> None:
        # Given: a cloud with the same timestamp as the preceding sample
        sample = _sample(stamp_nanoseconds=10)

        # When: the acceptance boundary validates the sample
        result = validate_cloud_sample(sample, previous_stamp_nanoseconds=10)

        # Then: nondecreasing order accepts it
        self.assertTrue(result.is_valid)
