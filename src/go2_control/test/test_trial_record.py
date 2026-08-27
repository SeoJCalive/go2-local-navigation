import json
from pathlib import Path

import pytest

from go2_control.trial_record import (
    MotionCandidateObservation,
    OdometryObservation,
    PreviewObservation,
    TrialRecordAccumulator,
    write_trial_record,
)


def test_given_readonly_observations_when_snapshot_created_then_preserves_latest_values() -> None:
    # Given: future physical trial에서 수집할 비작동 관찰값
    accumulator = TrialRecordAccumulator()
    accumulator.observe_candidate(
        received_at_nanoseconds=10,
        observation=MotionCandidateObservation(
            velocity_x=0.1,
            velocity_y=0.0,
            yaw_rate=0.0,
        ),
    )
    accumulator.observe_odometry(
        received_at_nanoseconds=20,
        observation=OdometryObservation(
            position_x=1.0,
            position_y=2.0,
            yaw_radians=0.5,
        ),
    )
    accumulator.observe_odometry(
        received_at_nanoseconds=40,
        observation=OdometryObservation(
            position_x=1.5,
            position_y=2.5,
            yaw_radians=0.7,
        ),
    )
    accumulator.observe_preview(
        received_at_nanoseconds=30,
        observation=PreviewObservation(api_id=1008, parameter="{}"),
    )
    accumulator.observe_preview(
        received_at_nanoseconds=50,
        observation=PreviewObservation(api_id=1003, parameter=""),
    )

    # When: trial record snapshot을 만든다.
    record = accumulator.snapshot(
        run_label="axis_x_forward",
        recorded_at="2026-08-27T00:00:00+00:00",
    )

    # Then: 측정값을 확정하지 않고 bounded first/last sample과 시간을 남긴다.
    assert record.status == "unverified"
    assert record.candidate_message_count == 1
    assert record.preview_message_count == 2
    assert record.odometry_message_count == 2
    assert record.latest_candidate is not None
    assert record.latest_candidate.velocity_x == 0.1
    assert record.latest_candidate_received_at_nanoseconds == 10
    assert record.first_odometry is not None
    assert record.first_odometry.position_x == 1.0
    assert record.first_odometry_received_at_nanoseconds == 20
    assert record.last_odometry is not None
    assert record.last_odometry.yaw_radians == 0.7
    assert record.last_odometry_received_at_nanoseconds == 40
    assert record.latest_preview is not None
    assert record.latest_preview.api_id == 1003
    assert record.latest_preview_received_at_nanoseconds == 50
    assert record.latest_move_preview_received_at_nanoseconds == 30
    assert record.latest_stop_preview_received_at_nanoseconds == 50


def test_given_matching_snapshot_inputs_when_created_then_ids_are_distinct() -> None:
    # Given: sample이 없는 bounded accumulator
    accumulator = TrialRecordAccumulator()

    # When: 같은 run label과 recorded_at으로 두 record를 만든다.
    first = accumulator.snapshot(
        run_label="axis_y_left",
        recorded_at="2026-08-27T00:00:00+00:00",
    )
    second = accumulator.snapshot(
        run_label="axis_y_left",
        recorded_at="2026-08-27T00:00:00+00:00",
    )

    # Then: future trial artifact가 서로 다른 run-scoped ID를 가진다.
    assert first.record_id != second.record_id


def test_given_trial_record_when_written_then_creates_machine_readable_future_artifact(
    tmp_path: Path,
) -> None:
    # Given: 한 개의 읽기 전용 candidate 관찰을 가진 accumulator
    accumulator = TrialRecordAccumulator()
    accumulator.observe_candidate(
        received_at_nanoseconds=10,
        observation=MotionCandidateObservation(
            velocity_x=0.0,
            velocity_y=0.1,
            yaw_rate=0.0,
        ),
    )
    record = accumulator.snapshot(
        run_label="axis_y_left",
        recorded_at="2026-08-27T00:00:00+00:00",
    )
    artifact_path = tmp_path / "trial_record.json"

    # When: record artifact를 쓴다.
    write_trial_record(artifact_path, record)

    # Then: 향후 실행 record가 unverified 상태와 관찰 topic을 보존한다.
    document = json.loads(artifact_path.read_text(encoding="utf-8"))
    assert document["record_id"].startswith(
        "go2-local-navigation-limited-motion-trial-"
    )
    assert document["status"] == "unverified"
    assert document["observed_topics"] == ["/go2_control/cmd_vel_candidate"]


def test_given_existing_artifact_when_written_then_preserves_original(
    tmp_path: Path,
) -> None:
    # Given: 같은 경로에 이미 보존된 trial artifact
    artifact_path = tmp_path / "trial_record.json"
    artifact_path.write_text("original\n", encoding="utf-8")
    record = TrialRecordAccumulator().snapshot(
        run_label="axis_y_left",
        recorded_at="2026-08-27T00:00:00+00:00",
    )

    # When: recorder가 같은 경로에 새 record를 쓰려고 한다.
    with pytest.raises(FileExistsError):
        write_trial_record(artifact_path, record)

    # Then: 기존 artifact 내용은 바뀌지 않는다.
    assert artifact_path.read_text(encoding="utf-8") == "original\n"


def test_given_nonfinite_observation_when_written_then_leaves_no_artifact(
    tmp_path: Path,
) -> None:
    # Given: JSON으로 보존할 수 없는 NaN candidate 관찰값
    accumulator = TrialRecordAccumulator()
    accumulator.observe_candidate(
        received_at_nanoseconds=10,
        observation=MotionCandidateObservation(
            velocity_x=float("nan"),
            velocity_y=0.0,
            yaw_rate=0.0,
        ),
    )
    record = accumulator.snapshot(
        run_label="invalid_candidate",
        recorded_at="2026-08-27T00:00:00+00:00",
    )
    artifact_path = tmp_path / "trial_record.json"

    # When: 엄격한 JSON 직렬화를 시도한다.
    with pytest.raises(ValueError):
        write_trial_record(artifact_path, record)

    # Then: 재시도를 막는 빈 artifact가 남지 않는다.
    assert not artifact_path.exists()


def test_given_recorder_source_when_inspected_then_has_no_publishers() -> None:
    # Given: future trial용 ROS recorder 구현 파일
    source_path = Path(__file__).parents[1] / "go2_control" / "trial_recorder_node.py"

    # When: output 경로를 정적으로 검사한다.
    source = source_path.read_text(encoding="utf-8")

    # Then: recorder는 관찰 topic만 구독하고 control publisher를 만들지 않는다.
    assert "create_subscription" in source
    assert "create_publisher" not in source
    assert '"/api/sport/request"' not in source
    assert '"/lowcmd"' not in source
