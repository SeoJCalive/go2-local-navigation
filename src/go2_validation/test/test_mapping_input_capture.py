from math import nan

from go2_validation.mapping_input_acceptance_runner import MappingInputVariant
from go2_validation.mapping_input_capture import MappingStreamCapture, build_observation


def test_given_valid_scan_and_odom_capture_when_built_then_runtime_contract_is_derived() -> None:
    # Given: a 2 Hz scan stream overlapping external odometry and one clock owner.
    capture = MappingStreamCapture(
        scan_type="sensor_msgs/msg/LaserScan",
        scan_frames=("base", "base", "base"),
        scan_stamps_ns=(1_000_000_000, 1_500_000_000, 2_000_000_000),
        scan_ranges=(0.4, float("inf"), 1.2),
        odom_stamps_ns=(1_100_000_000, 1_900_000_000),
        clock_publisher_max=1,
        global_tf_owner_count=0,
        command_publisher_max=0,
    )

    # When: the observer projection is built.
    observation = build_observation(
        MappingInputVariant.EXTERNAL_DYNAMIC_SHORT,
        capture,
        "source-sha256",
        domain_id=62,
        loopback_only=True,
        minimum_rate_hz=1.0,
    )

    # Then: type, frame, monotonicity, rate, ranges, and overlap all pass.
    assert observation.scan_stamps_monotonic
    assert observation.scan_minimum_rate_met
    assert observation.scan_ranges_finite_or_infinite
    assert observation.odom_overlaps_scan_clock


def test_given_nan_and_regressing_scan_when_built_then_observation_exposes_both_failures() -> None:
    # Given: invalid scan ranges and a timestamp regression.
    capture = MappingStreamCapture(
        scan_type="sensor_msgs/msg/LaserScan",
        scan_frames=("base", "base"),
        scan_stamps_ns=(2, 1),
        scan_ranges=(nan,),
        odom_stamps_ns=(),
        clock_publisher_max=1,
        global_tf_owner_count=0,
        command_publisher_max=0,
    )

    # When: the stationary observation is built.
    observation = build_observation(
        MappingInputVariant.PROJECT_STATIONARY,
        capture,
        "source-sha256",
        domain_id=62,
        loopback_only=True,
        minimum_rate_hz=1.0,
    )

    # Then: neither defect is hidden by the projection.
    assert not observation.scan_stamps_monotonic
    assert not observation.scan_ranges_finite_or_infinite
