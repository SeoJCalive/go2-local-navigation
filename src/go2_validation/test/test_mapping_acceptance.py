import ast
from dataclasses import asdict, replace
import json
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
NAV2_ROOT = PACKAGE_ROOT.parent / "go2_nav2"


def test_given_mapping_profile_when_loaded_then_domain63_frames_and_slam_owner_are_fixed() -> None:
    # Given: Todo 12가 소유해야 하는 SLAM parameter 파일
    config_path = NAV2_ROOT / "config/slam_mapping.yaml"

    # When: source-space 설정을 읽는다.
    assert config_path.is_file()
    document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    parameters = document["slam_toolbox"]["ros__parameters"]

    # Then: 기존 scan·odom·base 계약과 mapping mode가 고정된다.
    assert parameters["map_frame"] == "map"
    assert parameters["odom_frame"] == "odom"
    assert parameters["base_frame"] == "base"
    assert parameters["scan_topic"] == "/scan"
    assert parameters["mode"] == "mapping"
    assert parameters["use_sim_time"] is True


def test_given_launch_source_when_inspected_then_only_slam_mapping_inputs_are_composed() -> None:
    # Given: Todo 12 launch source
    launch_path = NAV2_ROOT / "launch/go2_slam_mapping.launch.py"

    # When: machine-consumed package와 executable token을 읽는다.
    assert launch_path.is_file()
    source = launch_path.read_text(encoding="utf-8")

    # Then: 기존 scan·odom launch와 단일 SLAM node만 조합한다.
    assert "go2_mapping_scan.launch.py" in source
    assert "go2_odometry_adapter.launch.py" in source
    assert 'package="slam_toolbox"' in source
    assert 'executable="async_slam_toolbox_node"' in source
    assert 'name="slam_toolbox"' in source
    assert "/api/sport/request" not in source
    assert "/lowcmd" not in source


def _passed_observation():
    from go2_validation.mapping_acceptance import (
        MappingArtifactObservation,
        MappingObservation,
        MappingOwnershipObservation,
        MappingProcessObservation,
        MappingStreamObservation,
        MappingVariant,
    )
    from go2_validation.mapping_pose_continuity import MappingCorrectionContinuityObservation
    from go2_validation.mapping_tf_continuity import MappingTfContinuityObservation

    return MappingObservation(
        variant=MappingVariant.PROJECT_STATIONARY,
        provenance="project_stationary",
        source_checksum="stationary-source-sha256",
        replay_checksum="stationary-bag-sha256",
        streams=MappingStreamObservation(
            expected_cloud_count=300,
            observed_cloud_count=300,
            expected_odometry_count=300,
            observed_odometry_count=300,
            scan_count=300,
            odom_count=298,
            map_count=4,
            map_frames=("map",),
            map_has_cells=True,
        ),
        ownership=MappingOwnershipObservation(
            slam_services_ready=True,
            clock_publisher_max=1,
            clock_progressed=True,
            clock_stalled=False,
            global_edges=(("map", "odom"),),
            global_owner_nodes=("/slam_toolbox",),
            command_publisher_max=0,
            control_node_max=0,
        ),
        artifacts=MappingArtifactObservation(
            occupancy_saved=True,
            pose_graph_saved=True,
            pose_graph_reloaded=True,
            checksums=(
                ("occupancy.pgm", "map-sha256"),
                ("occupancy.yaml", "yaml-sha256"),
                ("pose_graph.data", "data-sha256"),
                ("pose_graph.posegraph", "graph-sha256"),
            ),
        ),
        process=MappingProcessObservation(
            player_exit_code=0,
            launch_exit_code=0,
            residual_nodes=(),
            residual_processes=(),
            teardown_clock_publishers=0,
            teardown_global_owner_nodes=(),
        ),
        sensor_tf_profile="project_default",
        continuity=MappingTfContinuityObservation(2, 0.0, 0.0),
        map_correction_continuity=MappingCorrectionContinuityObservation(2, 0.0, 0.0),
    )


def test_given_complete_stationary_mapping_when_assessed_then_it_passes() -> None:
    # Given: full-input mapping, unique owners, reload와 clean teardown 관찰값
    observation = _passed_observation()

    # When: Todo 12 pure acceptance를 적용한다.
    from go2_validation.mapping_acceptance import assess_mapping

    result = assess_mapping(observation)

    # Then: stationary mapping smoke가 통과한다.
    assert result.status.value == "passed"
    assert result.failed_checks == ()


def test_given_response_expansion_selection_when_assessed_then_observation_and_result_json_provenance_preserve_it() -> None:
    # Given: a canonical DimOS selection that disables response expansion.
    from go2_validation.mapping_acceptance import assess_mapping

    observation = replace(_passed_observation(), use_response_expansion=False)

    # When: the mapping result projection is assessed.
    result = assess_mapping(observation)

    # Then: both JSON dataclass projections retain the actual selected bool.
    assert asdict(observation)["use_response_expansion"] is False
    assert asdict(result)["use_response_expansion"] is False


def test_given_loop_closing_selection_when_assessed_then_observation_and_result_json_preserve_false() -> None:
    # Given: loop closing을 명시적으로 끈 mapping 관찰값
    from go2_validation.mapping_acceptance import assess_mapping

    observation = replace(_passed_observation(), do_loop_closing=False)

    # When: terminal result projection을 만든다.
    result = assess_mapping(observation)

    # Then: acceptance 의미를 바꾸지 않고 실제 bool이 두 JSON projection에 남는다.
    assert result.status.value == "passed"
    assert asdict(observation)["do_loop_closing"] is False
    assert asdict(result)["do_loop_closing"] is False


def test_given_correction_over_translation_and_yaw_thresholds_when_assessed_then_renamed_checks_reject() -> None:
    # Given: raw map→odom diagnostic은 작지만 공통 current pose의 correction이 두 기준을 넘는다.
    from go2_validation.mapping_acceptance import assess_mapping
    from go2_validation.mapping_pose_continuity import MappingCorrectionContinuityObservation

    observation = replace(
        _passed_observation(),
        map_correction_continuity=MappingCorrectionContinuityObservation(2, 0.51, 0.21),
    )

    # When: mapping acceptance를 적용한다.
    result = assess_mapping(observation)

    # Then: 이전 map→base 이름 없이 correction threshold ID로 거부한다.
    assert "map_correction_translation_step" in result.failed_checks
    assert "map_correction_yaw_step" in result.failed_checks


def test_given_unaligned_correction_when_assessed_then_it_is_rejected_explicitly() -> None:
    # Given: common-current odometry를 양쪽에서 bracket하지 못한 correction이다.
    from go2_validation.mapping_acceptance import assess_mapping
    from go2_validation.mapping_pose_continuity import MappingCorrectionContinuityObservation

    observation = replace(
        _passed_observation(),
        map_correction_continuity=MappingCorrectionContinuityObservation(
            2,
            0.0,
            0.0,
            unaligned_sample_count=1,
        ),
    )

    # When: mapping acceptance를 적용한다.
    result = assess_mapping(observation)

    # Then: fabricated pose 대신 명시적 alignment failure를 반환한다.
    assert "map_correction_unaligned" in result.failed_checks


def test_given_duplicate_global_owner_when_assessed_then_it_fails() -> None:
    # Given: SLAM 이외의 두 번째 map→odom owner가 관찰된 실행
    observation = _passed_observation()
    duplicated = replace(
        observation,
        ownership=replace(
            observation.ownership,
            global_owner_nodes=("/other_mapper", "/slam_toolbox"),
        ),
    )

    # When: owner 계약을 판정한다.
    from go2_validation.mapping_acceptance import assess_mapping

    result = assess_mapping(duplicated)

    # Then: owner identity와 cardinality가 실패한다.
    assert result.status.value == "failed"
    assert "global_tf_owner" in result.failed_checks


def test_given_truncated_full_replay_when_assessed_then_it_fails() -> None:
    # Given: expected cloud 중 일부만 관찰된 external-full 실행
    observation = _passed_observation()
    truncated = replace(
        observation,
        streams=replace(
            observation.streams,
            expected_cloud_count=17_776,
            observed_cloud_count=17_775,
        ),
    )

    # When: 전체 입력 소비 계약을 판정한다.
    from go2_validation.mapping_acceptance import assess_mapping

    result = assess_mapping(truncated)

    # Then: replay truncation을 deferred가 아닌 실패로 보존한다.
    assert result.status.value == "failed"
    assert "cloud_count" in result.failed_checks


def test_given_complete_accumulated_accounting_when_assessed_then_mapping_passes() -> None:
    # Given: expected cloud count를 빠짐없이 publish한 accumulated mapping 실행
    from go2_validation.mapping_cloud_accounting import MappingCloudAccounting
    from go2_validation.mapping_acceptance import assess_mapping

    observation = replace(
        _passed_observation(),
        scan_projection_profile="dimos_odom_accumulated",
        cloud_accounting=MappingCloudAccounting(
            received=300,
            future_waited=3,
            recovered_after_retry=3,
            processed=300,
            output_published=300,
            dropped_unrecoverable=0,
            dropped_overflow=0,
            pending_at_shutdown=0,
            partial_frames_not_emitted=0,
            emit_every=1,
            output_stamp_regression_count=0,
        ),
    )

    # When: accumulated-profile acceptance를 적용한다.
    result = assess_mapping(observation)

    # Then: terminal accounting까지 exact인 candidate만 통과한다.
    assert result.status.value == "passed"
    assert result.failed_checks == ()
    assert asdict(observation)["cloud_accounting"] == {
        "received": 300,
        "future_waited": 3,
        "recovered_after_retry": 3,
        "processed": 300,
        "output_published": 300,
        "dropped_unrecoverable": 0,
        "dropped_overflow": 0,
        "pending_at_shutdown": 0,
        "partial_frames_not_emitted": 0,
        "emit_every": 1,
        "output_stamp_regression_count": 0,
    }


def test_given_default_intrinsic_cloud_drop_expectation_when_one_cloud_is_unrecoverable_then_mapping_fails() -> None:
    # Given: source evidence가 없는 accumulated profile과 unrecoverable cloud 한 개
    from go2_validation.mapping_cloud_accounting import MappingCloudAccounting
    from go2_validation.mapping_acceptance import assess_mapping

    observation = replace(
        _passed_observation(),
        scan_projection_profile="dimos_odom_accumulated",
        cloud_accounting=MappingCloudAccounting(
            received=300,
            future_waited=3,
            recovered_after_retry=3,
            processed=299,
            output_published=299,
            dropped_unrecoverable=1,
            dropped_overflow=0,
            pending_at_shutdown=0,
            partial_frames_not_emitted=0,
            emit_every=1,
            output_stamp_regression_count=0,
        ),
    )

    # When: default acceptance를 적용한다.
    result = assess_mapping(observation)

    # Then: source evidence가 없으면 기존 exact-zero contract가 그대로 거부한다.
    assert result.status.value == "failed"
    assert "cloud_accounting_dropped_unrecoverable" in result.failed_checks


def test_given_intrinsic_cloud_drop_expectation_one_when_one_cloud_is_unrecoverable_then_mapping_passes() -> None:
    # Given: source evidence로 정확히 한 개의 pre-odometry cloud를 기대한 run
    from go2_validation.mapping_cloud_accounting import MappingCloudAccounting
    from go2_validation.mapping_acceptance import assess_mapping

    observation = replace(
        _passed_observation(),
        scan_projection_profile="dimos_odom_accumulated",
        expected_intrinsic_untransformable_cloud_count=1,
        cloud_accounting=MappingCloudAccounting(
            received=300,
            future_waited=3,
            recovered_after_retry=3,
            processed=299,
            output_published=299,
            dropped_unrecoverable=1,
            dropped_overflow=0,
            pending_at_shutdown=0,
            partial_frames_not_emitted=0,
            emit_every=1,
            output_stamp_regression_count=0,
        ),
    )

    # When: source-aware exact acceptance를 적용한다.
    result = assess_mapping(observation)

    # Then: observation/result JSON projection 모두 expected count를 보존하고 통과한다.
    assert result.status.value == "passed"
    assert result.failed_checks == ()
    assert asdict(observation)["expected_intrinsic_untransformable_cloud_count"] == 1
    assert asdict(result)["expected_intrinsic_untransformable_cloud_count"] == 1


def test_given_intrinsic_cloud_drop_expectation_one_when_two_clouds_are_unrecoverable_then_mapping_fails() -> None:
    # Given: source evidence는 한 개지만 terminal accounting은 두 개를 unrecoverable로 기록한다.
    from go2_validation.mapping_cloud_accounting import MappingCloudAccounting
    from go2_validation.mapping_acceptance import assess_mapping

    observation = replace(
        _passed_observation(),
        scan_projection_profile="dimos_odom_accumulated",
        expected_intrinsic_untransformable_cloud_count=1,
        cloud_accounting=MappingCloudAccounting(
            received=300,
            future_waited=3,
            recovered_after_retry=3,
            processed=298,
            output_published=298,
            dropped_unrecoverable=2,
            dropped_overflow=0,
            pending_at_shutdown=0,
            partial_frames_not_emitted=0,
            emit_every=1,
            output_stamp_regression_count=0,
        ),
    )

    # When: source-aware exact acceptance를 적용한다.
    result = assess_mapping(observation)

    # Then: global allowance가 아니므로 source evidence보다 큰 drop은 거부한다.
    assert result.status.value == "failed"
    assert "cloud_accounting_dropped_unrecoverable" in result.failed_checks


def test_given_mapping_variant_spec_when_runtime_observation_is_constructed_then_expected_intrinsic_drop_is_propagated() -> None:
    # Given: runtime execution source와 immutable run spec field
    runtime_source = (
        PACKAGE_ROOT / "go2_validation/mapping_runtime_execution.py"
    ).read_text(encoding="utf-8")

    # When: _run_owned_mapping이 MappingObservation keyword를 구성한다.
    tree = ast.parse(runtime_source)
    observation_calls = tuple(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "MappingObservation"
    )

    # Then: runtime path는 spec 값을 observation/result JSON path로 그대로 전달한다.
    assert len(observation_calls) == 1
    field = next(
        keyword
        for keyword in observation_calls[0].keywords
        if keyword.arg == "expected_intrinsic_untransformable_cloud_count"
    )
    assert isinstance(field.value, ast.Attribute)
    assert isinstance(field.value.value, ast.Name)
    assert field.value.value.id == "spec"
    assert field.value.attr == "expected_intrinsic_untransformable_cloud_count"


def test_given_mapping_variant_spec_when_runtime_mapping_is_constructed_then_loop_closing_is_propagated() -> None:
    # Given: immutable run spec을 launch와 observation으로 연결하는 runtime source
    runtime_source = (
        PACKAGE_ROOT / "go2_validation/mapping_runtime_execution.py"
    ).read_text(encoding="utf-8")

    # When: nested MappingLaunchConfiguration와 MappingObservation call의 keyword를 읽는다.
    tree = ast.parse(runtime_source)
    launch_configuration_call = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "MappingLaunchConfiguration"
    )
    observation_call = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "MappingObservation"
    )

    # Then: 두 소비자 모두 이름 있는 spec bool을 그대로 받는다.
    for call in (launch_configuration_call, observation_call):
        keyword = next(item for item in call.keywords if item.arg == "do_loop_closing")
        assert isinstance(keyword.value, ast.Attribute)
        assert isinstance(keyword.value.value, ast.Name)
        assert keyword.value.value.id == "spec"
        assert keyword.value.attr == "do_loop_closing"


def test_given_canonical_dimos_scan_profile_ab_when_specs_are_built_then_source_evidence_sets_expected_intrinsic_drop_to_one() -> None:
    # Given: fixed canonical DimOS short A/B runner source
    runner_source = (
        PACKAGE_ROOT / "go2_validation/mapping_scan_profile_ab_runner.py"
    ).read_text(encoding="utf-8")

    # When/Then: first cloud가 first odometry보다 18.651008 ms 빠른 source evidence를 supply한다.
    assert "expected_intrinsic_untransformable_cloud_count=1" in runner_source


def test_given_complete_emit10_accounting_when_assessed_then_conservation_passes() -> None:
    from go2_validation.mapping_cloud_accounting import MappingCloudAccounting
    from go2_validation.mapping_acceptance import assess_mapping

    # Given: 300 processed clouds emitted as thirty non-overlapping batches
    observation = replace(
        _passed_observation(),
        scan_projection_profile="dimos_odom_accumulated_emit10",
        cloud_accounting=MappingCloudAccounting(
            received=300,
            future_waited=3,
            recovered_after_retry=3,
            processed=300,
            output_published=30,
            dropped_unrecoverable=0,
            dropped_overflow=0,
            pending_at_shutdown=0,
            partial_frames_not_emitted=0,
            emit_every=10,
            output_stamp_regression_count=0,
        ),
    )

    # When: explicit no-shutdown-flush conservation gates are applied.
    result = assess_mapping(observation)

    # Then: callback, processing, batch output and retry conservation all pass.
    assert result.status.value == "passed"
    assert result.failed_checks == ()


def test_given_missing_accumulated_accounting_when_assessed_then_mapping_fails_explicitly() -> None:
    # Given: terminal accounting을 찾지 못한 accumulated mapping 실행
    from go2_validation.mapping_acceptance import assess_mapping

    observation = replace(
        _passed_observation(),
        scan_projection_profile="dimos_odom_accumulated",
    )

    # When: accumulated-profile acceptance를 적용한다.
    result = assess_mapping(observation)

    # Then: absence는 pass가 아닌 diagnosable failed check가 된다.
    assert result.status.value == "failed"
    assert "cloud_accounting" in result.failed_checks


def test_given_invalid_accumulated_accounting_when_assessed_then_each_accounting_gate_fails() -> None:
    # Given: 하나씩 contract를 위반한 terminal accounting 값들
    from go2_validation.mapping_cloud_accounting import MappingCloudAccounting
    from go2_validation.mapping_acceptance import assess_mapping

    baseline = MappingCloudAccounting(
        received=300,
        future_waited=3,
        recovered_after_retry=3,
        processed=300,
        output_published=300,
        dropped_unrecoverable=0,
        dropped_overflow=0,
        pending_at_shutdown=0,
        partial_frames_not_emitted=0,
        emit_every=1,
        output_stamp_regression_count=0,
    )
    invalid_cases = (
        ("received", 299, "cloud_accounting_received"),
        ("processed", 299, "cloud_accounting_input_conservation"),
        ("output_published", 299, "cloud_accounting_output_conservation"),
        ("dropped_unrecoverable", 1, "cloud_accounting_dropped_unrecoverable"),
        ("dropped_overflow", 1, "cloud_accounting_dropped_overflow"),
        ("pending_at_shutdown", 1, "cloud_accounting_pending_at_shutdown"),
        (
            "output_stamp_regression_count",
            1,
            "cloud_accounting_output_stamp_regression",
        ),
        ("future_waited", 2, "cloud_accounting_future_recovery"),
    )

    # When/Then: 각 violation은 그 accounting check를 명시적으로 실패시킨다.
    for field_name, value, check_id in invalid_cases:
        observation = replace(
            _passed_observation(),
            scan_projection_profile="dimos_odom_accumulated",
            cloud_accounting=replace(baseline, **{field_name: value}),
        )

        result = assess_mapping(observation)

        assert result.status.value == "failed"
        assert check_id in result.failed_checks


def test_given_raw_profile_without_accounting_when_assessed_then_mapping_remains_unaffected() -> None:
    # Given: existing raw profile observation without an accumulator terminal marker
    from go2_validation.mapping_acceptance import assess_mapping

    observation = _passed_observation()

    # When: existing raw acceptance를 적용한다.
    result = assess_mapping(observation)

    # Then: raw profile에는 새 accounting gate가 생기지 않는다.
    assert result.status.value == "passed"
    assert result.failed_checks == ()


def test_given_saved_files_when_validated_then_references_and_checksums_are_returned(
    tmp_path: Path,
) -> None:
    # Given: SLAM Toolbox가 저장하는 occupancy와 pose graph 파일 집합
    artifact_root = tmp_path / "artifacts"
    artifact_root.mkdir()
    (artifact_root / "occupancy.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    (artifact_root / "occupancy.yaml").write_text(
        "image: occupancy.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n",
        encoding="utf-8",
    )
    (artifact_root / "pose_graph.data").write_bytes(b"serialized-data")
    (artifact_root / "pose_graph.posegraph").write_bytes(b"serialized-graph")

    # When: 저장 artifact 경계를 파싱한다.
    from go2_validation.mapping_artifacts import validate_saved_mapping_artifacts

    artifacts = validate_saved_mapping_artifacts(artifact_root)

    # Then: YAML image 참조와 네 파일 checksum이 모두 고정된다.
    assert artifacts.image_path == artifact_root / "occupancy.pgm"
    assert tuple(path.name for path in artifacts.paths) == (
        "occupancy.pgm",
        "occupancy.yaml",
        "pose_graph.data",
        "pose_graph.posegraph",
    )
    assert len(artifacts.checksums) == 4


def test_given_missing_map_image_when_validated_then_it_is_rejected(
    tmp_path: Path,
) -> None:
    # Given: 존재하지 않는 image를 참조하는 occupancy YAML
    artifact_root = tmp_path / "artifacts"
    artifact_root.mkdir()
    (artifact_root / "occupancy.yaml").write_text(
        "image: missing.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n",
        encoding="utf-8",
    )
    (artifact_root / "pose_graph.data").write_bytes(b"serialized-data")
    (artifact_root / "pose_graph.posegraph").write_bytes(b"serialized-graph")

    # When/Then: 누락 artifact가 명시적 typed error가 된다.
    from go2_validation.mapping_artifacts import (
        MappingArtifactError,
        validate_saved_mapping_artifacts,
    )

    try:
        validate_saved_mapping_artifacts(artifact_root)
    except MappingArtifactError as error:
        assert error.reason_code == "mapping_artifact_missing"
    else:
        raise AssertionError("missing map image must be rejected")


def test_given_rosbag_metadata_when_parsed_then_selected_counts_and_duration_are_typed(
    tmp_path: Path,
) -> None:
    # Given: canonical cloud·odometry 두 topic의 rosbag2 metadata
    bag_root = tmp_path / "bag"
    bag_root.mkdir()
    (bag_root / "metadata.yaml").write_text(
        yaml.safe_dump(
            {
                "rosbag2_bagfile_information": {
                    "duration": {"nanoseconds": 10_000_000_000},
                    "starting_time": {"nanoseconds_since_epoch": 20_000_000_000},
                    "topics_with_message_count": [
                        {
                            "topic_metadata": {
                                "name": "/utlidar/cloud",
                                "type": "sensor_msgs/msg/PointCloud2",
                            },
                            "message_count": 150,
                        },
                        {
                            "topic_metadata": {
                                "name": "/utlidar/robot_odom",
                                "type": "nav_msgs/msg/Odometry",
                            },
                            "message_count": 1_500,
                        },
                    ],
                }
            }
        ),
        encoding="utf-8",
    )

    # When: runtime data boundary가 metadata를 파싱한다.
    from go2_validation.mapping_runtime_data import read_bag_expectation

    expectation = read_bag_expectation(bag_root)

    # Then: 전체 소비 판정과 bounded timeout에 필요한 값이 보존된다.
    assert expectation.cloud_count == 150
    assert expectation.odometry_count == 1_500
    assert expectation.start_nanoseconds == 20_000_000_000
    assert expectation.end_nanoseconds == 30_000_000_000
    assert expectation.playback_timeout_seconds == 132.0


def test_given_passed_conversion_when_read_then_full_fixture_custody_is_preserved(
    tmp_path: Path,
) -> None:
    # Given: Todo 9의 passed external-full conversion result
    result_path = tmp_path / "conversion.json"
    result_path.write_text(
        json.dumps(
            {
                "status": "passed",
                "provenance": "external_dynamic",
                "source_checksum": "source-sha256",
                "full_bag_path": "data/external/derived/full",
                "full_checksum": "full-tree-sha256",
                "full_cloud_count": 17_776,
                "full_odometry_count": 173_616,
            }
        ),
        encoding="utf-8",
    )

    # When: full replay boundary를 읽는다.
    from go2_validation.mapping_runtime_data import read_external_full_replay

    replay = read_external_full_replay(result_path)

    # Then: raw source와 derived bag identity가 섞이지 않는다.
    assert replay.status.value == "passed"
    assert replay.provenance == "external_dynamic"
    assert replay.bag_path == Path("data/external/derived/full")
    assert replay.source_checksum == "source-sha256"
    assert replay.replay_checksum == "full-tree-sha256"
    assert replay.cloud_count == 17_776
    assert replay.odometry_count == 173_616


def test_given_mapping_bag_when_command_built_then_replay_is_bounded_and_one_x() -> None:
    # Given: 한 mapping input bag과 1.0배속
    bag_path = Path("data/external/derived/full")

    # When: shell 없는 player argv를 만든다.
    from go2_validation.mapping_runtime_execution import mapping_bag_play_command

    command = mapping_bag_play_command(bag_path, 1.0)

    # Then: 단일 clock, selected raw topics와 keyboard 차단이 고정된다.
    assert command[:4] == ("ros2", "bag", "play", str(bag_path))
    assert command[command.index("--rate") + 1] == "1.0"
    assert command[command.index("--clock") + 1] == "100"
    assert "--loop" not in command
    assert command[-3:] == (
        "/utlidar/cloud",
        "/utlidar/robot_odom",
        "--disable-keyboard-controls",
    )


def test_given_package_metadata_when_inspected_then_mapping_runtime_is_installable() -> None:
    # Given: ament package metadata와 process marker source
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    package_source = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    host_source = (
        PACKAGE_ROOT.parent / "bringup/bringup/preflight_host.py"
    ).read_text(encoding="utf-8")

    # When/Then: install-space entry point, runtime dependency와 teardown marker가 존재한다.
    assert "mapping_acceptance = go2_validation.mapping_acceptance_runner:main" in setup_source
    assert "<exec_depend>slam_toolbox</exec_depend>" in package_source
    assert '"async_slam_toolbox_node"' in host_source
