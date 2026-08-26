"""
통합 preflight의 topic·정지 pose 판정 계약을 검증한다.

ROS graph나 실제 Go2에 연결하지 않고, 관찰값이 PASS·WARN·FAIL로 바뀌는
순수 판정 경계를 고정한다. 실제 topic 수신 E2E는 AGX 정지 실행에서 별도로 확인한다.
"""

from bringup.preflight_accumulator import TopicAccumulator
from bringup.preflight_metrics import (
    assess_stationary_pose,
    assess_topic,
    overall_status,
)
from bringup.preflight_types import (
    CheckStatus,
    ObservedMessage,
    Pose2D,
    TopicContract,
)


def _odom_contract() -> TopicContract:
    return TopicContract(
        topic="/odom",
        expected_type="nav_msgs/msg/Odometry",
        expected_frame="odom",
        expected_child_frame="base",
        minimum_rate_hz=10.0,
        maximum_gap_seconds=0.5,
    )


def test_topic_summary_passes_when_contract_is_continuous() -> None:
    # Given: 올바른 type·frame과 일정한 20 Hz odometry 표본
    accumulator = TopicAccumulator(_odom_contract())
    accumulator.observe_graph(("nav_msgs/msg/Odometry",), 1)
    for index in range(3):
        accumulator.observe(
            ObservedMessage(
                receive_nanoseconds=index * 50_000_000,
                stamp_nanoseconds=1_000_000_000 + index * 50_000_000,
                frame_id="odom",
                child_frame_id="base",
                is_valid=True,
                pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
            )
        )

    # When: topic 계약을 판정한다.
    checks = assess_topic(accumulator.summary())

    # Then: graph·수신·schema·timing이 모두 통과한다.
    assert {check.status for check in checks} == {CheckStatus.PASS}


def test_topic_timing_fails_when_timestamp_regresses() -> None:
    # Given: 두 번째 message timestamp가 과거로 돌아간 odometry 표본
    accumulator = TopicAccumulator(_odom_contract())
    accumulator.observe_graph(("nav_msgs/msg/Odometry",), 1)
    accumulator.observe(
        ObservedMessage(
            receive_nanoseconds=0,
            stamp_nanoseconds=2_000_000_000,
            frame_id="odom",
            child_frame_id="base",
            is_valid=True,
            pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
        )
    )
    accumulator.observe(
        ObservedMessage(
            receive_nanoseconds=50_000_000,
            stamp_nanoseconds=1_000_000_000,
            frame_id="odom",
            child_frame_id="base",
            is_valid=True,
            pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
        )
    )

    # When: topic 계약을 판정한다.
    checks = assess_topic(accumulator.summary())

    # Then: timing 항목만 강제 실패 조건을 보고한다.
    timing = next(check for check in checks if check.check_id.endswith(".timing"))
    assert timing.status is CheckStatus.FAIL
    assert "timestamp_regressions=1" in timing.detail


def test_stationary_pose_warns_for_drift_and_fails_for_jump() -> None:
    # Given: 누적 0.12 m drift와 단일 0.30 m jump가 있는 정지 odometry
    accumulator = TopicAccumulator(_odom_contract())
    for index, x_position in enumerate((0.0, 0.12, 0.42)):
        accumulator.observe(
            ObservedMessage(
                receive_nanoseconds=index * 50_000_000,
                stamp_nanoseconds=1_000_000_000 + index * 50_000_000,
                frame_id="odom",
                child_frame_id="base",
                is_valid=True,
                pose=Pose2D(x=x_position, y=0.0, yaw=0.0),
            )
        )

    # When: 정지 pose 기준을 판정한다.
    check = assess_stationary_pose(accumulator.summary())

    # Then: 단일 pose jump가 drift 경고보다 우선해 FAIL이 된다.
    assert check.status is CheckStatus.FAIL
    assert "max_step_translation_m=0.300000" in check.detail


def test_overall_status_preserves_warning_without_promoting_it_to_failure() -> None:
    # Given: 통과 항목과 정지 drift 경고가 함께 있다.
    accumulator = TopicAccumulator(_odom_contract())
    accumulator.observe_graph(("nav_msgs/msg/Odometry",), 1)
    for index, x_position in enumerate((0.0, 0.12)):
        accumulator.observe(
            ObservedMessage(
                receive_nanoseconds=index * 50_000_000,
                stamp_nanoseconds=1_000_000_000 + index * 50_000_000,
                frame_id="odom",
                child_frame_id="base",
                is_valid=True,
                pose=Pose2D(x=x_position, y=0.0, yaw=0.0),
            )
        )
    pose_check = assess_stationary_pose(accumulator.summary())

    # When: 전체 상태를 계산한다.
    status = overall_status((*assess_topic(accumulator.summary()), pose_check))

    # Then: 실행은 보존 가능하지만 후속 확인이 필요한 WARN이다.
    assert pose_check.status is CheckStatus.WARN
    assert status is CheckStatus.WARN
