from math import cos, isclose, pi, sin, sqrt


def _yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, sin(yaw / 2.0), cos(yaw / 2.0))


def test_given_unchanged_map_transform_and_moving_odometry_when_corrected_then_jump_is_zero() -> None:
    # Given: map→odom은 고정됐지만 odom→base가 두 correction 시각 사이에서 이동한다.
    from go2_validation.mapping_pose_continuity import MapCorrectionContinuityAccumulator

    accumulator = MapCorrectionContinuityAccumulator()
    accumulator.observe_odometry((0.0, 0.0, 0.0), _yaw_quaternion(0.0), 0)
    accumulator.observe_odometry((2.0, 0.0, 0.0), _yaw_quaternion(0.0), 200_000_000)
    accumulator.observe_odometry((4.0, 0.0, 0.0), _yaw_quaternion(0.0), 1_000_000_000)
    accumulator.observe_map_to_odom((0.0, 0.0, 0.0), _yaw_quaternion(0.0), 200_000_000)

    # When: 같은 map→odom을 다음 effective correction 시각에 다시 관찰한다.
    accumulator.observe_map_to_odom((0.0, 0.0, 0.0), _yaw_quaternion(0.0), 1_200_000_000)

    # Then: robot 이동을 보정량으로 만들지 않는다.
    observation = accumulator.observation()
    assert observation.sample_count == 1
    assert observation.maximum_translation_step_m == 0.0
    assert observation.maximum_yaw_step_rad == 0.0


def test_given_pure_yaw_map_correction_at_radius_when_corrected_then_chord_and_yaw_are_measured() -> None:
    # Given: 원점에서 r만큼 떨어진 공통 current odom pose와 순수 map yaw 보정이다.
    from go2_validation.mapping_pose_continuity import MapCorrectionContinuityAccumulator

    radius_m = 23.0
    correction_yaw = 0.358
    accumulator = MapCorrectionContinuityAccumulator()
    accumulator.observe_odometry((radius_m, 0.0, 0.0), _yaw_quaternion(0.0), 0)
    accumulator.observe_odometry((radius_m, 0.0, 0.0), _yaw_quaternion(0.0), 200_000_000)
    accumulator.observe_map_to_odom((0.0, 0.0, 0.0), _yaw_quaternion(0.0), 200_000_000)

    # When: translation 없는 map yaw correction을 적용한다.
    accumulator.observe_map_to_odom(
        (0.0, 0.0, 0.0),
        _yaw_quaternion(correction_yaw),
        400_000_000,
    )

    # Then: current radius에서의 chord와 shortest yaw delta를 측정한다.
    observation = accumulator.observation()
    assert observation.sample_count == 1
    assert isclose(
        observation.maximum_translation_step_m,
        2.0 * radius_m * sin(correction_yaw / 2.0),
    )
    assert isclose(observation.maximum_yaw_step_rad, correction_yaw)
    assert observation.maximum_translation_step_published_stamp_ns == 400_000_000
    assert observation.maximum_translation_step_effective_stamp_ns == 200_000_000


def test_given_map_yaw_correction_cancelled_at_current_pose_when_corrected_then_translation_is_zero() -> None:
    # Given: map yaw correction을 current odom pose에서 정확히 상쇄하는 map translation이다.
    from go2_validation.mapping_pose_continuity import MapCorrectionContinuityAccumulator

    radius_m = 23.0
    correction_yaw = 0.358
    accumulator = MapCorrectionContinuityAccumulator()
    accumulator.observe_odometry((radius_m, 0.0, 0.0), _yaw_quaternion(0.0), 0)
    accumulator.observe_odometry((radius_m, 0.0, 0.0), _yaw_quaternion(0.0), 200_000_000)
    accumulator.observe_map_to_odom((0.0, 0.0, 0.0), _yaw_quaternion(0.0), 200_000_000)

    # When: M_current * O_current == M_previous * O_current가 되도록 보정한다.
    accumulator.observe_map_to_odom(
        (radius_m - radius_m * cos(correction_yaw), -radius_m * sin(correction_yaw), 0.0),
        _yaw_quaternion(correction_yaw),
        400_000_000,
    )

    # Then: translation은 상쇄되고 yaw correction만 남는다.
    observation = accumulator.observation()
    assert observation.sample_count == 1
    assert observation.maximum_translation_step_m < 1e-9
    assert isclose(observation.maximum_yaw_step_rad, correction_yaw)


def test_given_one_sided_odometry_when_corrected_then_sample_is_explicitly_unaligned() -> None:
    # Given: effective correction 시각 뒤에만 허용 범위 odometry가 있다.
    from go2_validation.mapping_pose_continuity import MapCorrectionContinuityAccumulator

    accumulator = MapCorrectionContinuityAccumulator()
    accumulator.observe_map_to_odom((0.0, 0.0, 0.0), _yaw_quaternion(0.0), 200_000_000)
    accumulator.observe_odometry((5.0, 0.0, 0.0), _yaw_quaternion(0.0), 190_000_000)

    # When: 두 번째 correction이 one-sided nearest 표본만 가진 시각에 도착한다.
    accumulator.observe_map_to_odom((1.0, 0.0, 0.0), _yaw_quaternion(0.0), 400_000_000)

    # Then: pose를 만들어내지 않고 unaligned correction으로 남긴다.
    observation = accumulator.observation()
    assert observation.sample_count == 0
    assert observation.unaligned_sample_count == 1


def test_given_same_effective_stamp_and_same_planar_pose_when_corrected_then_it_is_a_duplicate() -> None:
    # Given: 같은 planar pose의 20 Hz rebroadcast다.
    from go2_validation.mapping_pose_continuity import MapCorrectionContinuityAccumulator

    accumulator = MapCorrectionContinuityAccumulator()
    accumulator.observe_odometry((0.0, 0.0, 0.0), _yaw_quaternion(0.0), 0)
    accumulator.observe_odometry((3.0, 0.0, 0.0), _yaw_quaternion(0.0), 1_000_000_000)
    accumulator.observe_map_to_odom((0.0, 0.0, 0.0), _yaw_quaternion(0.0), 200_000_000)

    # When: effective stamp와 planar pose가 같은 TF를 다시 받는다.
    accumulator.observe_map_to_odom((0.0, 0.0, 0.0), _yaw_quaternion(0.0), 200_000_000)
    accumulator.observe_map_to_odom((0.0, 0.0, 0.0), _yaw_quaternion(0.0), 1_200_000_000)

    # Then: rebroadcast 하나만 duplicate이고 monotonic correction 하나만 측정한다.
    observation = accumulator.observation()
    assert observation.sample_count == 1
    assert observation.duplicate_stamp_count == 1
    assert observation.regressive_stamp_count == 0
    assert observation.maximum_translation_step_m == 0.0


def test_given_same_effective_stamp_and_changed_planar_pose_when_corrected_then_it_is_measured() -> None:
    # Given: 같은 effective correction 시각의 current odom pose와 최초 map→odom이다.
    from go2_validation.mapping_pose_continuity import MapCorrectionContinuityAccumulator

    accumulator = MapCorrectionContinuityAccumulator()
    accumulator.observe_odometry((2.0, 0.0, 0.0), _yaw_quaternion(0.0), 0)
    accumulator.observe_map_to_odom((0.0, 0.0, 0.0), _yaw_quaternion(0.0), 200_000_000)

    # When: 같은 effective stamp에서 translation과 yaw가 바뀐 TF를 받는다.
    accumulator.observe_map_to_odom((1.0, 0.0, 0.0), _yaw_quaternion(pi / 2.0), 200_000_000)

    # Then: 같은 current odom pose에서 이전 TF와 비교한 correction이 측정된다.
    observation = accumulator.observation()
    assert observation.sample_count == 1
    assert observation.duplicate_stamp_count == 0
    assert isclose(observation.maximum_translation_step_m, sqrt(5.0))
    assert isclose(observation.maximum_yaw_step_rad, pi / 2.0)
    assert observation.maximum_translation_step_published_stamp_ns == 200_000_000
    assert observation.maximum_translation_step_effective_stamp_ns == 0


def test_given_regressive_effective_stamp_when_corrected_then_it_is_rejected() -> None:
    # Given: monotonic correction 뒤에 더 이른 effective stamp가 온다.
    from go2_validation.mapping_pose_continuity import MapCorrectionContinuityAccumulator

    accumulator = MapCorrectionContinuityAccumulator()
    accumulator.observe_odometry((0.0, 0.0, 0.0), _yaw_quaternion(0.0), 0)
    accumulator.observe_odometry((3.0, 0.0, 0.0), _yaw_quaternion(0.0), 1_000_000_000)
    accumulator.observe_map_to_odom((0.0, 0.0, 0.0), _yaw_quaternion(0.0), 200_000_000)
    accumulator.observe_map_to_odom((0.0, 0.0, 0.0), _yaw_quaternion(0.0), 1_200_000_000)

    # When: 이미 받은 correction보다 앞선 effective stamp를 받는다.
    accumulator.observe_map_to_odom((20.0, 0.0, 0.0), _yaw_quaternion(1.0), 100_000_000)

    # Then: 새로운 sample을 만들지 않고 regression만 기록한다.
    observation = accumulator.observation()
    assert observation.sample_count == 1
    assert observation.duplicate_stamp_count == 0
    assert observation.regressive_stamp_count == 1


def test_given_slam_mapping_config_when_read_then_effective_time_offset_is_locked() -> None:
    # Given: the project SLAM timeout configuration.
    from pathlib import Path

    import yaml

    from go2_validation.mapping_pose_continuity import SLAM_TRANSFORM_TIMEOUT_NS

    # When: the machine-consumed timeout is loaded.
    document = yaml.safe_load(
        (
            Path(__file__).parents[2]
            / "go2_nav2/config/slam_mapping.yaml"
        ).read_text(encoding="utf-8")
    )

    # Then: transform published time and composition effective time differ by 0.2 s.
    assert document["slam_toolbox"]["ros__parameters"]["transform_timeout"] == 0.2
    assert SLAM_TRANSFORM_TIMEOUT_NS == 200_000_000
