from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def test_given_stopped_mapping_launch_when_runtime_contract_is_read_then_graph_settle_is_bounded() -> None:
    # Given: launch 종료 뒤 graph teardown을 판정하는 runtime source
    source = (
        PACKAGE_ROOT / "go2_validation/mapping_runtime_execution.py"
    ).read_text(encoding="utf-8")

    # When/Then: fixed delay가 아니라 empty-graph 조건을 wall-time 상한으로 기다린다.
    assert "spin_until(observer, observer.teardown_complete, 30.0)" in source


def test_given_accumulated_profile_launch_log_when_accounting_is_loaded_then_terminal_fields_are_kept(
    tmp_path: Path,
) -> None:
    # Given: ros2 launch prefix를 포함한 accumulator terminal accounting log
    launch_log = tmp_path / "launch.log"
    launch_log.write_text(
        "[go2_mapping_cloud_accumulator-5] MAPPING_CLOUD_ACCOUNTING "
        '{"received":4,"future_waited":1,"recovered_after_retry":1,'
        '"processed":4,"output_published":4,"dropped_unrecoverable":0,'
        '"dropped_overflow":0,"pending_at_shutdown":0,'
        '"partial_frames_not_emitted":0,"emit_every":1,'
        '"output_stamp_regression_count":0}\n',
        encoding="utf-8",
    )

    # When: runtime이 쓰는 accumulated-profile boundary를 적용한다.
    from go2_validation.mapping_cloud_accounting import mapping_cloud_accounting_for_profile

    accounting = mapping_cloud_accounting_for_profile(
        launch_log,
        "dimos_odom_accumulated",
    )

    # Then: MappingObservation에 넣을 immutable accounting이 손실 없이 남는다.
    assert accounting is not None
    assert accounting.received == accounting.processed == accounting.output_published == 4


def test_given_emit10_profile_launch_log_when_accounting_is_loaded_then_it_is_required(
    tmp_path: Path,
) -> None:
    # Given: emit10 terminal marker with one intentionally un-emitted partial frame
    launch_log = tmp_path / "launch.log"
    launch_log.write_text(
        "[node-1] MAPPING_CLOUD_ACCOUNTING "
        '{"received":21,"future_waited":0,"recovered_after_retry":0,'
        '"processed":21,"output_published":2,"dropped_unrecoverable":0,'
        '"dropped_overflow":0,"pending_at_shutdown":0,'
        '"partial_frames_not_emitted":1,"emit_every":10,'
        '"output_stamp_regression_count":0}\n',
        encoding="utf-8",
    )

    # When: emit10 accumulated profile boundary를 적용한다.
    from go2_validation.mapping_cloud_accounting import mapping_cloud_accounting_for_profile

    accounting = mapping_cloud_accounting_for_profile(
        launch_log, "dimos_odom_accumulated_emit10"
    )

    # Then: marker는 raw처럼 생략되지 않고 batch·partial 수를 보존한다.
    assert accounting is not None
    assert accounting.output_published == 2
    assert accounting.partial_frames_not_emitted == 1
    assert accounting.emit_every == 10


def test_given_raw_profile_launch_log_when_accounting_is_loaded_then_it_is_not_required(
    tmp_path: Path,
) -> None:
    # Given: terminal accounting marker가 없는 raw mapping launch log
    launch_log = tmp_path / "launch.log"
    launch_log.write_text("[slam_toolbox-1] shutdown complete\n", encoding="utf-8")

    # When: runtime이 쓰는 raw-profile boundary를 적용한다.
    from go2_validation.mapping_cloud_accounting import mapping_cloud_accounting_for_profile

    accounting = mapping_cloud_accounting_for_profile(launch_log, "raw_single")

    # Then: raw profile은 accounting 없이 계속 허용된다.
    assert accounting is None
