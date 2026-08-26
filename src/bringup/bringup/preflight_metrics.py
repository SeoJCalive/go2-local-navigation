"""topic 연속성과 정지 pose summary를 PASS·WARN·FAIL로 판정한다."""

from bringup.preflight_types import (
    CheckResult,
    CheckStatus,
    TopicSummary,
)


DRIFT_WARNING_TRANSLATION_M = 0.10
DRIFT_WARNING_YAW_RAD = 0.10
STEP_FAILURE_TRANSLATION_M = 0.25
STEP_FAILURE_YAW_RAD = 0.25


def assess_topic(summary: TopicSummary) -> tuple[CheckResult, ...]:
    """한 topic의 graph·수신·schema·timing을 독립 항목으로 판정한다."""
    contract = summary.contract
    graph_passed = (
        contract.expected_type in summary.observed_types
        and summary.maximum_publisher_count > 0
    )
    receive_passed = (
        summary.received_messages > 1
        and summary.rate_hz >= contract.minimum_rate_hz
    )
    frame_passed = (
        contract.expected_frame is None
        or summary.observed_frames == (contract.expected_frame,)
    )
    child_frame_passed = (
        contract.expected_child_frame is None
        or summary.observed_child_frames == (contract.expected_child_frame,)
    )
    schema_passed = (
        summary.invalid_messages == 0
        and frame_passed
        and child_frame_passed
    )
    timing_passed = (
        summary.received_messages > 1
        and summary.timestamp_regressions == 0
        and summary.maximum_gap_seconds <= contract.maximum_gap_seconds
    )
    topic_id = contract.topic.strip("/").replace("/", ".")
    return (
        CheckResult(
            check_id=f"topic.{topic_id}.graph",
            status=CheckStatus.PASS if graph_passed else CheckStatus.FAIL,
            detail=(
                f"types={summary.observed_types}; "
                f"max_publishers={summary.maximum_publisher_count}"
            ),
        ),
        CheckResult(
            check_id=f"topic.{topic_id}.receive",
            status=CheckStatus.PASS if receive_passed else CheckStatus.FAIL,
            detail=(
                f"messages={summary.received_messages}; rate_hz={summary.rate_hz:.3f}; "
                f"minimum_rate_hz={contract.minimum_rate_hz:.3f}"
            ),
        ),
        CheckResult(
            check_id=f"topic.{topic_id}.schema",
            status=CheckStatus.PASS if schema_passed else CheckStatus.FAIL,
            detail=(
                f"invalid_messages={summary.invalid_messages}; "
                f"frames={summary.observed_frames}; "
                f"child_frames={summary.observed_child_frames}"
            ),
        ),
        CheckResult(
            check_id=f"topic.{topic_id}.timing",
            status=CheckStatus.PASS if timing_passed else CheckStatus.FAIL,
            detail=(
                f"timestamp_regressions={summary.timestamp_regressions}; "
                f"max_gap_seconds={summary.maximum_gap_seconds:.6f}; "
                f"limit_seconds={contract.maximum_gap_seconds:.6f}"
            ),
        ),
    )


def assess_stationary_pose(summary: TopicSummary) -> CheckResult:
    """정지 pose의 누적 drift는 경고, 단일 큰 jump는 실패로 판정한다."""
    values = (
        summary.drift_translation_m,
        summary.drift_yaw_rad,
        summary.maximum_step_translation_m,
        summary.maximum_step_yaw_rad,
    )
    if any(value is None for value in values):
        return CheckResult(
            check_id="odometry.stationary_pose",
            status=CheckStatus.FAIL,
            detail="pose_samples=0",
        )
    drift_translation = summary.drift_translation_m or 0.0
    drift_yaw = summary.drift_yaw_rad or 0.0
    step_translation = summary.maximum_step_translation_m or 0.0
    step_yaw = summary.maximum_step_yaw_rad or 0.0
    if (
        step_translation > STEP_FAILURE_TRANSLATION_M
        or step_yaw > STEP_FAILURE_YAW_RAD
    ):
        status = CheckStatus.FAIL
    elif (
        drift_translation > DRIFT_WARNING_TRANSLATION_M
        or drift_yaw > DRIFT_WARNING_YAW_RAD
    ):
        status = CheckStatus.WARN
    else:
        status = CheckStatus.PASS
    return CheckResult(
        check_id="odometry.stationary_pose",
        status=status,
        detail=(
            f"drift_translation_m={drift_translation:.6f}; "
            f"drift_yaw_rad={drift_yaw:.6f}; "
            f"max_step_translation_m={step_translation:.6f}; "
            f"max_step_yaw_rad={step_yaw:.6f}"
        ),
    )


def overall_status(checks: tuple[CheckResult, ...]) -> CheckStatus:
    """FAIL과 WARN을 우선하고 나머지 실행 가능 상태를 보존한다."""
    statuses = {check.status for check in checks}
    if CheckStatus.FAIL in statuses:
        return CheckStatus.FAIL
    if CheckStatus.WARN in statuses:
        return CheckStatus.WARN
    if CheckStatus.PASS in statuses:
        return CheckStatus.PASS
    return CheckStatus.DEFERRED
