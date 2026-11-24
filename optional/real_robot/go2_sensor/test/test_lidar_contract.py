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
            row_step=160,
            data_length=160,
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

    def test_validate_cloud_sample_rejects_zero_layout_and_short_data(self) -> None:
        zero_layout = _sample()
        zero_layout = CloudSample(
            frame_id=zero_layout.frame_id,
            stamp_nanoseconds=zero_layout.stamp_nanoseconds,
            layout=CloudLayout(1, 1, 0, 0, 0, ("x", "y", "z")),
        )
        short_data = CloudSample(
            frame_id="utlidar_lidar",
            stamp_nanoseconds=1,
            layout=CloudLayout(1, 2, 16, 32, 16, ("x", "y", "z")),
        )

        zero_result = validate_cloud_sample(zero_layout, None)
        short_result = validate_cloud_sample(short_data, None)

        self.assertEqual(zero_result.reason, "cloud layout must be nonzero")
        self.assertEqual(short_result.reason, "cloud data length mismatch")
