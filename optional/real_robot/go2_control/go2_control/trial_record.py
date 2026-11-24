"""
제한적 물리 시험의 읽기 전용 관찰을 미래 실행 record JSON으로 보존한다.

이 모듈은 ROS publisher나 Unitree control interface를 만들지 않는다. recorder
node가 candidate, preview, odometry에서 받은 값을 이 자료형으로 전달하고,
호출자가 지정한 경로에 unverified 실행 record를 쓴다.
"""

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Final
from uuid import uuid4

from go2_control.sport_request import MOVE_API_ID, STOP_MOVE_API_ID


RECORD_ID: Final = "go2-local-navigation-limited-motion-trial"
RECORD_KIND: Final = "limited_physical_motion_trial_observation"
STATUS_UNVERIFIED: Final = "unverified"


@dataclass(frozen=True, slots=True)
class MotionCandidateObservation:
    velocity_x: float
    velocity_y: float
    yaw_rate: float


@dataclass(frozen=True, slots=True)
class PreviewObservation:
    api_id: int
    parameter: str


@dataclass(frozen=True, slots=True)
class OdometryObservation:
    position_x: float
    position_y: float
    yaw_radians: float


@dataclass(frozen=True, slots=True)
class TrialRecord:
    record_id: str
    record_kind: str
    run_label: str
    status: str
    recorded_at: str
    provenance: str
    candidate_message_count: int
    preview_message_count: int
    odometry_message_count: int
    observed_topics: tuple[str, ...]
    latest_candidate: MotionCandidateObservation | None
    latest_candidate_received_at_nanoseconds: int | None
    latest_preview: PreviewObservation | None
    latest_preview_received_at_nanoseconds: int | None
    latest_move_preview: PreviewObservation | None
    latest_move_preview_received_at_nanoseconds: int | None
    latest_stop_preview: PreviewObservation | None
    latest_stop_preview_received_at_nanoseconds: int | None
    first_odometry: OdometryObservation | None
    first_odometry_received_at_nanoseconds: int | None
    last_odometry: OdometryObservation | None
    last_odometry_received_at_nanoseconds: int | None


class TrialRecordAccumulator:  # noqa: MUTABLE_OK
    """고정 수의 event와 첫·마지막 odometry만 축적한다."""

    def __init__(self) -> None:
        self._candidate_message_count = 0
        self._preview_message_count = 0
        self._odometry_message_count = 0
        self._latest_candidate: MotionCandidateObservation | None = None
        self._latest_candidate_received_at_nanoseconds: int | None = None
        self._latest_preview: PreviewObservation | None = None
        self._latest_preview_received_at_nanoseconds: int | None = None
        self._latest_move_preview: PreviewObservation | None = None
        self._latest_move_preview_received_at_nanoseconds: int | None = None
        self._latest_stop_preview: PreviewObservation | None = None
        self._latest_stop_preview_received_at_nanoseconds: int | None = None
        self._first_odometry: OdometryObservation | None = None
        self._first_odometry_received_at_nanoseconds: int | None = None
        self._last_odometry: OdometryObservation | None = None
        self._last_odometry_received_at_nanoseconds: int | None = None

    def observe_candidate(
        self,
        received_at_nanoseconds: int,
        observation: MotionCandidateObservation,
    ) -> None:
        self._candidate_message_count += 1
        self._latest_candidate = observation
        self._latest_candidate_received_at_nanoseconds = received_at_nanoseconds

    def observe_preview(
        self,
        received_at_nanoseconds: int,
        observation: PreviewObservation,
    ) -> None:
        self._preview_message_count += 1
        self._latest_preview = observation
        self._latest_preview_received_at_nanoseconds = received_at_nanoseconds
        if observation.api_id == MOVE_API_ID:
            self._latest_move_preview = observation
            self._latest_move_preview_received_at_nanoseconds = (
                received_at_nanoseconds
            )
        if observation.api_id == STOP_MOVE_API_ID:
            self._latest_stop_preview = observation
            self._latest_stop_preview_received_at_nanoseconds = (
                received_at_nanoseconds
            )

    def observe_odometry(
        self,
        received_at_nanoseconds: int,
        observation: OdometryObservation,
    ) -> None:
        self._odometry_message_count += 1
        if self._first_odometry is None:
            self._first_odometry = observation
            self._first_odometry_received_at_nanoseconds = (
                received_at_nanoseconds
            )
        self._last_odometry = observation
        self._last_odometry_received_at_nanoseconds = received_at_nanoseconds

    def snapshot(self, run_label: str, recorded_at: str) -> TrialRecord:
        observed_topics = tuple(
            topic
            for topic, message_count in (
                ("/go2_control/cmd_vel_candidate", self._candidate_message_count),
                ("/go2_control/sport_request_preview", self._preview_message_count),
                ("/odom", self._odometry_message_count),
            )
            if message_count > 0
        )
        return TrialRecord(
            record_id=f"{RECORD_ID}-{uuid4()}",
            record_kind=RECORD_KIND,
            run_label=run_label,
            status=STATUS_UNVERIFIED,
            recorded_at=recorded_at,
            provenance="read_only_recorder_no_control_publication",
            candidate_message_count=self._candidate_message_count,
            preview_message_count=self._preview_message_count,
            odometry_message_count=self._odometry_message_count,
            observed_topics=observed_topics,
            latest_candidate=self._latest_candidate,
            latest_candidate_received_at_nanoseconds=(
                self._latest_candidate_received_at_nanoseconds
            ),
            latest_preview=self._latest_preview,
            latest_preview_received_at_nanoseconds=(
                self._latest_preview_received_at_nanoseconds
            ),
            latest_move_preview=self._latest_move_preview,
            latest_move_preview_received_at_nanoseconds=(
                self._latest_move_preview_received_at_nanoseconds
            ),
            latest_stop_preview=self._latest_stop_preview,
            latest_stop_preview_received_at_nanoseconds=(
                self._latest_stop_preview_received_at_nanoseconds
            ),
            first_odometry=self._first_odometry,
            first_odometry_received_at_nanoseconds=(
                self._first_odometry_received_at_nanoseconds
            ),
            last_odometry=self._last_odometry,
            last_odometry_received_at_nanoseconds=(
                self._last_odometry_received_at_nanoseconds
            ),
        )


def write_trial_record(path: Path, record: TrialRecord) -> None:
    serialized_record = (
        json.dumps(
            asdict(record),
            ensure_ascii=False,
            indent=2,
            allow_nan=False,
        )
        + "\n"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as record_file:
        record_file.write(serialized_record)
