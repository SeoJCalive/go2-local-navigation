"""고빈도 ROS callback의 topic 통계를 고정 크기로 누적한다."""

from math import atan2, cos, hypot, sin

from bringup.preflight_types import (
    ObservedMessage,
    Pose2D,
    TopicContract,
    TopicSummary,
)


NANOSECONDS_PER_SECOND = 1_000_000_000


class TopicAccumulator:
    """message 원문을 보존하지 않고 연속성·frame·pose 통계만 누적한다."""

    def __init__(self, contract: TopicContract) -> None:
        self.contract = contract
        self.received_messages = 0
        self.invalid_messages = 0
        self.timestamp_regressions = 0
        self.observed_frames: set[str] = set()
        self.observed_child_frames: set[str] = set()
        self.observed_types: set[str] = set()
        self.maximum_publisher_count = 0
        self._first_receive_nanoseconds: int | None = None
        self._last_receive_nanoseconds: int | None = None
        self._previous_stamp_nanoseconds: int | None = None
        self._maximum_gap_nanoseconds = 0
        self._first_pose: Pose2D | None = None
        self._last_pose: Pose2D | None = None
        self._maximum_step_translation_m = 0.0
        self._maximum_step_yaw_rad = 0.0

    def observe_graph(
        self,
        observed_types: tuple[str, ...],
        publisher_count: int,
    ) -> None:
        """관찰한 graph type과 실행 중 최대 publisher 수를 누적한다."""
        self.observed_types.update(observed_types)
        self.maximum_publisher_count = max(
            self.maximum_publisher_count,
            publisher_count,
        )

    def observe(self, sample: ObservedMessage) -> None:
        """수신 message의 간격·timestamp·frame·pose를 누적한다."""
        self.received_messages += 1
        if not sample.is_valid:
            self.invalid_messages += 1
        if self._first_receive_nanoseconds is None:
            self._first_receive_nanoseconds = sample.receive_nanoseconds
        if self._last_receive_nanoseconds is not None:
            self._maximum_gap_nanoseconds = max(
                self._maximum_gap_nanoseconds,
                sample.receive_nanoseconds - self._last_receive_nanoseconds,
            )
        self._last_receive_nanoseconds = sample.receive_nanoseconds
        self._observe_stamp(sample.stamp_nanoseconds)
        if sample.frame_id:
            self.observed_frames.add(sample.frame_id)
        if sample.child_frame_id:
            self.observed_child_frames.add(sample.child_frame_id)
        self._observe_pose(sample.pose)

    def _observe_stamp(self, stamp_nanoseconds: int | None) -> None:
        if stamp_nanoseconds is None or stamp_nanoseconds <= 0:
            return
        if (
            self._previous_stamp_nanoseconds is not None
            and stamp_nanoseconds < self._previous_stamp_nanoseconds
        ):
            self.timestamp_regressions += 1
            return
        self._previous_stamp_nanoseconds = stamp_nanoseconds

    def _observe_pose(self, pose: Pose2D | None) -> None:
        if pose is None:
            return
        if self._first_pose is None:
            self._first_pose = pose
        if self._last_pose is not None:
            self._maximum_step_translation_m = max(
                self._maximum_step_translation_m,
                hypot(pose.x - self._last_pose.x, pose.y - self._last_pose.y),
            )
            self._maximum_step_yaw_rad = max(
                self._maximum_step_yaw_rad,
                abs(_angle_difference(pose.yaw, self._last_pose.yaw)),
            )
        self._last_pose = pose

    def summary(self) -> TopicSummary:
        """현재 누적값을 불변 summary로 변환한다."""
        elapsed_nanoseconds = 0
        if (
            self._first_receive_nanoseconds is not None
            and self._last_receive_nanoseconds is not None
        ):
            elapsed_nanoseconds = (
                self._last_receive_nanoseconds - self._first_receive_nanoseconds
            )
        rate_hz = 0.0
        if self.received_messages > 1 and elapsed_nanoseconds > 0:
            rate_hz = (
                (self.received_messages - 1)
                * NANOSECONDS_PER_SECOND
                / elapsed_nanoseconds
            )
        drift_translation_m, drift_yaw_rad = self._drift()
        return TopicSummary(
            contract=self.contract,
            received_messages=self.received_messages,
            invalid_messages=self.invalid_messages,
            timestamp_regressions=self.timestamp_regressions,
            observed_frames=tuple(sorted(self.observed_frames)),
            observed_child_frames=tuple(sorted(self.observed_child_frames)),
            observed_types=tuple(sorted(self.observed_types)),
            maximum_publisher_count=self.maximum_publisher_count,
            rate_hz=rate_hz,
            maximum_gap_seconds=(
                self._maximum_gap_nanoseconds / NANOSECONDS_PER_SECOND
            ),
            drift_translation_m=drift_translation_m,
            drift_yaw_rad=drift_yaw_rad,
            maximum_step_translation_m=(
                self._maximum_step_translation_m
                if self._first_pose is not None
                else None
            ),
            maximum_step_yaw_rad=(
                self._maximum_step_yaw_rad
                if self._first_pose is not None
                else None
            ),
        )

    def _drift(self) -> tuple[float | None, float | None]:
        if self._first_pose is None or self._last_pose is None:
            return None, None
        return (
            hypot(
                self._last_pose.x - self._first_pose.x,
                self._last_pose.y - self._first_pose.y,
            ),
            abs(_angle_difference(self._last_pose.yaw, self._first_pose.yaw)),
        )


def _angle_difference(current: float, previous: float) -> float:
    return atan2(sin(current - previous), cos(current - previous))
