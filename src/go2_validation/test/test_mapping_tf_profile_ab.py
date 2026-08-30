from dataclasses import replace
import json
from math import cos, isclose, sin
from pathlib import Path
from typing import Final

import yaml


PACKAGE_ROOT: Final = Path(__file__).parents[1]
NAV2_ROOT: Final = PACKAGE_ROOT.parent / "go2_nav2"


def test_given_nonpassed_conversion_manifest_when_tf_ab_input_loads_then_it_is_rejected_before_bag_read(
    tmp_path: Path,
) -> None:
    # Given: short bag identity가 아직 passed custody가 아닌 conversion manifest
    from go2_validation.mapping_runtime_data import MappingRuntimeDataError
    from go2_validation.mapping_tf_profile_ab_input import load_mapping_tf_profile_ab_input

    manifest_path = tmp_path / "conversion.json"
    manifest_path.write_text(json.dumps({"status": "deferred"}), encoding="utf-8")

    # When: TF A/B 입력으로 파싱한다.
    try:
        load_mapping_tf_profile_ab_input(tmp_path, manifest_path)
    except MappingRuntimeDataError as error:
        rejection = error
    else:
        raise AssertionError("non-passed TF A/B manifest must be rejected")

    # Then: bag metadata를 읽기 전에 custody status로 거부한다.
    assert rejection.reason_code == "tf_ab_fixture_not_passed"


def _continuous_mapping_observation():
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
        variant=MappingVariant.EXTERNAL_DYNAMIC_SHORT,
        provenance="external_dynamic",
        source_checksum="source-sha256",
        replay_checksum="short-replay-sha256",
        streams=MappingStreamObservation(
            expected_cloud_count=1_843,
            observed_cloud_count=1_843,
            expected_odometry_count=18_026,
            observed_odometry_count=18_026,
            scan_count=1_842,
            odom_count=17_000,
            map_count=40,
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
        sensor_tf_profile="dimos_replay",
        continuity=MappingTfContinuityObservation(
            sample_count=2_000,
            maximum_translation_step_m=0.08,
            maximum_yaw_step_rad=0.03,
        ),
        map_correction_continuity=MappingCorrectionContinuityObservation(
            sample_count=2_000,
            maximum_translation_step_m=0.08,
            maximum_yaw_step_rad=0.03,
        ),
    )


def test_given_large_raw_map_to_odom_steps_when_map_correction_is_continuous_then_mapping_passes() -> None:
    # Given: map origin diagnostic is large but the current-pose correction is continuous.
    from go2_validation.mapping_acceptance import assess_mapping
    from go2_validation.mapping_tf_continuity import MappingTfContinuityObservation

    observation = _continuous_mapping_observation()
    discontinuous = replace(
        observation,
        continuity=MappingTfContinuityObservation(
            sample_count=2_000,
            maximum_translation_step_m=6.0,
            maximum_yaw_step_rad=0.4,
        ),
    )

    # When: mapping acceptance를 적용한다.
    result = assess_mapping(discontinuous)

    # Then: raw map→odom remains diagnostic and does not reject the mapping run.
    assert result.status.value == "passed"
    assert result.failed_checks == ()


def test_given_continuous_map_to_odom_steps_when_assessed_then_mapping_passes() -> None:
    # Given: artifact·owner·stream과 global TF 연속성이 모두 유효한 실행
    from go2_validation.mapping_acceptance import assess_mapping

    # When: 같은 acceptance를 적용한다.
    result = assess_mapping(_continuous_mapping_observation())

    # Then: continuity check를 포함해 통과한다.
    assert result.status.value == "passed"
    assert result.failed_checks == ()
    assert result.sensor_tf_profile == "dimos_replay"


def test_given_map_correction_jump_when_assessed_then_mapping_fails() -> None:
    # Given: raw diagnostic is small but map correction discontinuously moves and rotates.
    from go2_validation.mapping_acceptance import assess_mapping
    from go2_validation.mapping_pose_continuity import MappingCorrectionContinuityObservation

    observation = replace(
        _continuous_mapping_observation(),
        map_correction_continuity=MappingCorrectionContinuityObservation(2, 1.0, 0.3),
    )

    # When: acceptance applies the current-pose correction gates.
    result = assess_mapping(observation)

    # Then: translation and yaw jumps reject an otherwise complete mapping run.
    assert "map_correction_translation_step" in result.failed_checks
    assert "map_correction_yaw_step" in result.failed_checks


def test_given_regressive_map_correction_stamp_when_assessed_then_mapping_fails() -> None:
    # Given: one regressive effective map correction timestamp.
    from go2_validation.mapping_acceptance import assess_mapping
    from go2_validation.mapping_pose_continuity import MappingCorrectionContinuityObservation

    observation = replace(
        _continuous_mapping_observation(),
        map_correction_continuity=MappingCorrectionContinuityObservation(
            2, 0.0, 0.0, regressive_stamp_count=1
        ),
    )

    # When: acceptance applies timestamp-order validity.
    result = assess_mapping(observation)

    # Then: a regressive output is a deterministic mapping failure.
    assert "map_correction_stamp_regression" in result.failed_checks


def test_given_tf_samples_when_accumulated_then_translation_and_shortest_yaw_steps_are_measured(
) -> None:
    # Given: 원점과 3-4-5 이동·0.4 rad 회전의 연속 transform
    from go2_validation.mapping_tf_continuity import MappingTfContinuityAccumulator

    accumulator = MappingTfContinuityAccumulator()
    accumulator.observe(
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        stamp_nanoseconds=1,
    )
    accumulator.observe(
        (3.0, 4.0, 0.0),
        (0.0, 0.0, sin(0.2), cos(0.2)),
        stamp_nanoseconds=2,
    )

    # When: bounded observation으로 투영한다.
    observation = accumulator.observation()

    # Then: payload 보관 없이 count와 최대 step만 남는다.
    assert observation.sample_count == 2
    assert isclose(observation.maximum_translation_step_m, 5.0)
    assert isclose(observation.maximum_yaw_step_rad, 0.4)
    assert observation.translation_exceedance_count == 1
    assert observation.yaw_exceedance_count == 1
    assert observation.maximum_translation_step_stamp_ns == 2
    assert observation.maximum_yaw_step_stamp_ns == 2


def test_given_mapping_profiles_when_launch_command_built_then_response_expansion_selection_is_explicit() -> None:
    # Given: project 기본 mapping과 DimOS scan A/B의 서로 다른 response expansion 선택
    from go2_validation.mapping_command_builders import MappingLaunchConfiguration
    from go2_validation.mapping_runtime_execution import mapping_launch_command

    # When: mapping launch argv를 shell 없이 만든다.
    default_command = mapping_launch_command(
        MappingLaunchConfiguration("project_default")
    )
    dimos_command = mapping_launch_command(
        MappingLaunchConfiguration(
            "dimos_replay",
            "dimos_odom_accumulated",
            use_response_expansion=False,
        )
    )

    # Then: default=true는 보존하고 canonical DimOS selection만 false를 launch로 전달한다.
    assert "use_response_expansion:=true" in default_command
    assert "sensor_tf_profile:=dimos_replay" in dimos_command
    assert "scan_projection_profile:=dimos_odom_accumulated" in dimos_command
    assert "use_response_expansion:=false" in dimos_command


def test_given_default_mapping_when_launch_command_built_then_loop_closing_remains_enabled() -> None:
    # Given: 별도 loop-closing 선택이 없는 일반 mapping profile
    from go2_validation.mapping_command_builders import MappingLaunchConfiguration
    from go2_validation.mapping_runtime_execution import mapping_launch_command

    # When: 기본 mapping launch argv를 만든다.
    command = mapping_launch_command(MappingLaunchConfiguration("project_default"))

    # Then: 기존 일반 mapping 동작은 loop closing enabled로 고정된다.
    assert "do_loop_closing:=true" in command


def test_given_explicit_loop_closing_false_when_launch_command_built_then_bools_are_independent() -> None:
    # Given: response expansion과 loop closing을 각각 끄는 canonical DimOS 선택
    from go2_validation.mapping_command_builders import MappingLaunchConfiguration
    from go2_validation.mapping_runtime_execution import mapping_launch_command

    # When: 두 bool을 이름으로 지정해 launch argv를 만든다.
    command = mapping_launch_command(MappingLaunchConfiguration(
        sensor_tf_profile="dimos_replay",
        scan_projection_profile="dimos_odom_accumulated",
        use_response_expansion=False,
        do_loop_closing=False,
    ))

    # Then: 두 선택이 서로의 argv를 대체하지 않고 각각 false로 전달된다.
    assert "use_response_expansion:=false" in command
    assert "do_loop_closing:=false" in command


def test_given_mapping_launch_chain_when_read_then_tf_profile_reaches_static_tf_owner() -> None:
    # Given: Nav2 mapping launch와 perception scan launch
    slam_source = (NAV2_ROOT / "launch/go2_slam_mapping.launch.py").read_text(
        encoding="utf-8"
    )
    perception_source = (
        PACKAGE_ROOT.parent / "go2_perception/launch/go2_mapping_scan.launch.py"
    ).read_text(encoding="utf-8")

    # When/Then: 한 profile argument가 두 include 경계를 모두 통과한다.
    assert "sensor_tf_profile" in slam_source
    assert "sensor_tf_profile" in perception_source
    assert "scan_projection_profile" in slam_source
    assert "scan_projection_profile" in perception_source
    assert '"sensor_tf_profile": sensor_tf_profile' in slam_source
    assert '"sensor_tf_profile": sensor_tf_profile' in perception_source
    assert '"scan_projection_profile": scan_projection_profile' in slam_source


def test_given_mapping_launch_when_response_expansion_is_selected_then_bool_override_follows_base_yaml() -> None:
    # Given: SLAM Toolbox mapping launch source and its base YAML defaults
    launch_source = (NAV2_ROOT / "launch/go2_slam_mapping.launch.py").read_text(
        encoding="utf-8"
    )
    mapping_yaml = yaml.safe_load(
        (NAV2_ROOT / "config/slam_mapping.yaml").read_text(encoding="utf-8")
    )

    # When: launch arguments and parameter layering are inspected.
    # Then: bool conversion and later override preserve base defaults while allowing a selected value.
    assert mapping_yaml["slam_toolbox"]["ros__parameters"]["use_response_expansion"] is True
    assert 'DeclareLaunchArgument("use_response_expansion", default_value="true")' in launch_source
    assert '"use_response_expansion": ParameterValue(' in launch_source
    assert "use_response_expansion," in launch_source
    assert "value_type=bool," in launch_source
    assert "response_expansion_parameter" in launch_source
    assert "slam_parameters," in launch_source


def test_given_mapping_launch_when_loop_closing_is_selected_then_typed_override_follows_true_base_yaml() -> None:
    # Given: SLAM Toolbox mapping launch source와 일반 mapping YAML
    launch_source = (NAV2_ROOT / "launch/go2_slam_mapping.launch.py").read_text(
        encoding="utf-8"
    )
    mapping_yaml = yaml.safe_load(
        (NAV2_ROOT / "config/slam_mapping.yaml").read_text(encoding="utf-8")
    )

    # When: launch argument와 SLAM parameter layering을 검사한다.
    # Then: YAML·launch 기본값은 true이고 선택값은 typed bool override로 뒤에 적용된다.
    assert mapping_yaml["slam_toolbox"]["ros__parameters"]["do_loop_closing"] is True
    assert 'DeclareLaunchArgument("do_loop_closing", default_value="true")' in launch_source
    assert '"do_loop_closing": ParameterValue(' in launch_source
    assert "do_loop_closing," in launch_source
    assert "value_type=bool," in launch_source
    assert "loop_closing_parameter" in launch_source
    assert "slam_parameters," in launch_source


def test_given_mapping_variant_spec_when_defaults_are_inspected_then_loop_closing_is_true() -> None:
    # Given: 모든 일반 mapping caller가 공유하는 immutable variant spec
    from dataclasses import fields

    from go2_validation.mapping_runtime_execution import MappingVariantSpec

    # When: dataclass field defaults를 읽는다.
    defaults = {field.name: field.default for field in fields(MappingVariantSpec)}

    # Then: caller가 선택하지 않으면 기존 loop-closing mapping이 유지된다.
    assert defaults["do_loop_closing"] is True


def test_given_scan_ranges_when_accumulated_then_valid_beam_quantiles_are_bounded() -> None:
    from go2_validation.mapping_scan_quality import MappingScanQualityAccumulator

    accumulator = MappingScanQualityAccumulator()
    accumulator.observe((1.0, float("inf"), 2.0, float("nan")), 0.25, 5.0)
    accumulator.observe((1.0, 2.0, 3.0, 4.0), 0.25, 5.0)

    observation = accumulator.observation()

    assert observation.sample_count == 2
    assert observation.minimum_valid_beams == 2
    assert observation.median_valid_beams == 2
    assert observation.maximum_valid_beams == 4


def test_given_package_metadata_when_read_then_scan_profile_ab_runner_is_installable() -> None:
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    runner_path = PACKAGE_ROOT / "go2_validation/mapping_scan_profile_ab_runner.py"

    assert runner_path.is_file()
    assert '"mapping_scan_profile_ab = "' in setup_source
    assert '"go2_validation.mapping_scan_profile_ab_runner:main"' in setup_source


def test_given_selected_scan_profiles_when_ab_runs_then_specs_and_directories_use_actual_ids(
    monkeypatch,
    tmp_path: Path,
) -> None:
    from types import SimpleNamespace

    import go2_validation.mapping_scan_profile_ab_runner as runner

    # Given: parent-selected sliding baseline와 emit10 candidate
    replay = SimpleNamespace(
        bag_path=Path("short-bag"),
        provenance="external_dynamic",
        source_checksum="source",
        replay_checksum="replay",
        expectation=None,
    )
    calls = []
    monkeypatch.setattr(
        runner,
        "run_mapping_variant",
        lambda spec, path: calls.append((spec, path)) or spec,
    )
    monkeypatch.setattr(runner, "write_mapping_ab_execution", lambda *_args: None)

    # When: profile IDs를 source edit 없이 직접 전달한다.
    baseline, candidate = runner.run_mapping_scan_profile_ab(
        replay,
        tmp_path,
        runner.MappingScanProfilePair(
            "dimos_odom_accumulated",
            "dimos_odom_accumulated_emit10",
        ),
    )

    # Then: actual IDs가 실행 spec, 결과 객체와 directory에 그대로 남는다.
    assert baseline.scan_projection_profile == "dimos_odom_accumulated"
    assert candidate.scan_projection_profile == "dimos_odom_accumulated_emit10"
    assert baseline.use_response_expansion is False
    assert candidate.use_response_expansion is False
    assert baseline.do_loop_closing is True
    assert candidate.do_loop_closing is True
    assert baseline.coarse_search_angle_offset == 0.349
    assert candidate.coarse_search_angle_offset == 0.1745
    assert tuple(path.name for _spec, path in calls) == (
        "dimos_odom_accumulated",
        "dimos_odom_accumulated_emit10",
    )


def test_given_arbitrary_scan_ab_when_candidate_passes_continuity_then_density_is_diagnostic(
    monkeypatch,
) -> None:
    from types import SimpleNamespace

    import go2_validation.mapping_scan_profile_ab_runner as runner
    from go2_validation.mapping_acceptance import MappingStatus

    # Given: lower-density candidate that passes after a continuity-failing baseline
    quality = lambda median: SimpleNamespace(median_valid_beams=median)
    baseline = SimpleNamespace(
        result=SimpleNamespace(failed_checks=("map_correction_yaw_step",)),
        observation=SimpleNamespace(
            streams=SimpleNamespace(scan_quality=quality(300))
        ),
    )
    candidate = SimpleNamespace(
        result=SimpleNamespace(status=MappingStatus.PASSED),
        observation=SimpleNamespace(
            streams=SimpleNamespace(scan_quality=quality(250))
        ),
    )
    monkeypatch.setattr(runner, "mapping_ab_execution_document", lambda _value: {})
    monkeypatch.setattr(runner, "asdict", lambda value: {"median": value.median_valid_beams})

    # When: arbitrary selected IDs are summarized.
    document, passed = runner._summary_document(
        baseline,
        candidate,
        runner.MappingScanProfilePair(
            "dimos_odom_accumulated",
            "dimos_odom_accumulated_emit10",
        ),
    )

    # Then: continuity/candidate pass toggles while density remains an explicit diagnostic.
    assert passed is True
    assert document["toggle_confirmed"] is True
    assert document["scan_density_improved"] is False
    assert document["baseline_scan_profile"] == "dimos_odom_accumulated"
    assert document["candidate_scan_profile"] == "dimos_odom_accumulated_emit10"
    assert (
        document["comparison_dimension"]
        == "scan_projection_and_coarse_search_bound"
    )


def test_given_scan_ab_runner_source_when_read_then_ros_profile_parameters_keep_old_defaults() -> None:
    runner_source = (
        PACKAGE_ROOT / "go2_validation/mapping_scan_profile_ab_runner.py"
    ).read_text(encoding="utf-8")

    assert '"baseline_scan_profile"' in runner_source
    assert '"candidate_scan_profile"' in runner_source
    assert 'BASELINE_SCAN_PROFILE: Final = "raw_single"' in runner_source
    assert 'CANDIDATE_SCAN_PROFILE: Final = "dimos_odom_accumulated_emit3"' in runner_source


def test_given_mapping_ab_summary_sources_when_read_then_renamed_correction_schema_is_version_two() -> None:
    # Given: TF와 scan profile A/B JSON producer source다.
    tf_source = (
        PACKAGE_ROOT / "go2_validation/mapping_tf_profile_ab_runner.py"
    ).read_text(encoding="utf-8")
    scan_source = (
        PACKAGE_ROOT / "go2_validation/mapping_scan_profile_ab_runner.py"
    ).read_text(encoding="utf-8")

    # When: machine-consumed continuity check IDs와 schema version을 읽는다.
    sources = (tf_source, scan_source)

    # Then: new JSON은 correction schema version 2와 새 check ID를 함께 갖는다.
    assert all('"schema_version": 2' in source for source in sources)
    assert all("map_correction_translation_step" in source for source in sources)


def test_given_package_metadata_when_read_then_tf_ab_runner_is_installable() -> None:
    # Given: package setup과 A/B runner source
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    runner_path = PACKAGE_ROOT / "go2_validation/mapping_tf_profile_ab_runner.py"

    # When/Then: 반복 가능한 실제 ROS entry point가 설치된다.
    assert runner_path.is_file()
    assert '"mapping_tf_profile_ab = "' in setup_source
    assert '"go2_validation.mapping_tf_profile_ab_runner:main"' in setup_source
