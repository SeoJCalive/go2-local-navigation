import json
from dataclasses import replace
from pathlib import Path

from go2_validation.mapping_input_acceptance_runner import (
    ExternalReplayStatus,
    ExternalShortReplayBoundary,
    MappingInputObservation,
    MappingInputRunSpec,
    MappingInputVariant,
    assess_mapping_input,
    build_run_specs,
    parse_external_short_replay,
    summarize_variants,
)
from go2_validation.mapping_input_execution import bag_play_command
from go2_validation.mapping_input_execution import mapping_launch_commands
from go2_validation.mapping_input_runtime import read_external_short_replay


def _stationary_observation() -> MappingInputObservation:
    return MappingInputObservation(
        variant=MappingInputVariant.PROJECT_STATIONARY,
        scan_message_type="sensor_msgs/msg/LaserScan",
        scan_frame_id="base",
        scan_stamps_monotonic=True,
        scan_ranges_finite_or_infinite=True,
        scan_minimum_rate_met=True,
        clock_publishers=1,
        global_map_to_odom_owners=0,
        command_publishers=0,
        domain_id=62,
        loopback_only=True,
        odom_overlaps_scan_clock=True,
        source_checksum="project-stationary-sha256",
    )


def test_given_stationary_observation_when_assessed_then_it_passes() -> None:
    result = assess_mapping_input(
        _stationary_observation(), ExternalReplayStatus.PASSED
    )

    assert result.status.value == "passed"
    assert result.provenance == "project_stationary"


def test_given_todo9_minimum_external_boundary_when_parsed_then_provenance_and_checksum_stay_separate() -> (
    None
):
    replay = parse_external_short_replay(
        ExternalShortReplayBoundary(
            status="passed",
            provenance="external_dynamic",
            short_bag_path="data/external/dimos/derived/short",
            source_checksum="source-sha256",
        )
    )

    assert replay.status is ExternalReplayStatus.PASSED
    assert replay.provenance == "external_dynamic"
    assert replay.short_bag_path == Path("data/external/dimos/derived/short")
    assert replay.source_checksum == "source-sha256"


def test_given_passed_external_fixture_when_planned_then_variants_are_sequential_and_external_includes_odom_adapter() -> (
    None
):
    replay = parse_external_short_replay(
        ExternalShortReplayBoundary(
            status="passed",
            provenance="external_dynamic",
            short_bag_path="data/external/dimos/derived/short",
            source_checksum="source-sha256",
        )
    )

    plans = build_run_specs(replay)

    assert plans == (
        MappingInputRunSpec(
            variant=MappingInputVariant.PROJECT_STATIONARY,
            launch_files=("go2_mapping_scan.launch.py",),
        ),
        MappingInputRunSpec(
            variant=MappingInputVariant.EXTERNAL_DYNAMIC_SHORT,
            launch_files=(
                "go2_mapping_scan.launch.py",
                "go2_odometry_adapter.launch.py",
            ),
        ),
    )


def test_given_deferred_external_fixture_when_summarized_then_stationary_pass_is_preserved() -> (
    None
):
    stationary = assess_mapping_input(
        _stationary_observation(), ExternalReplayStatus.PASSED
    )
    external = assess_mapping_input(
        replace(
            _stationary_observation(),
            variant=MappingInputVariant.EXTERNAL_DYNAMIC_SHORT,
        ),
        ExternalReplayStatus.DEFERRED,
    )

    summary = summarize_variants((stationary, external))

    assert stationary.status.value == "passed"
    assert external.status.value == "deferred"
    assert external.source_checksum is None
    assert summary.overall_status.value == "passed"


def test_given_external_conflict_when_summarized_then_overall_fails() -> None:
    stationary = assess_mapping_input(
        _stationary_observation(), ExternalReplayStatus.PASSED
    )
    external = assess_mapping_input(
        replace(
            _stationary_observation(),
            variant=MappingInputVariant.EXTERNAL_DYNAMIC_SHORT,
        ),
        ExternalReplayStatus.CONFLICT,
    )

    summary = summarize_variants((stationary, external))

    assert external.status.value == "conflict"
    assert summary.overall_status.value == "failed"


def test_given_external_without_odom_overlap_when_assessed_then_it_fails() -> None:
    result = assess_mapping_input(
        replace(
            _stationary_observation(),
            variant=MappingInputVariant.EXTERNAL_DYNAMIC_SHORT,
            odom_overlaps_scan_clock=False,
        ),
        ExternalReplayStatus.PASSED,
    )

    assert result.status.value == "failed"
    assert "odom_clock_overlap" in result.failed_checks


def test_given_passed_conversion_json_when_read_then_short_fixture_is_resolved(
    tmp_path: Path,
) -> None:
    # Given: the minimal passed Todo 9 conversion result.
    result_path = tmp_path / "conversion.json"
    result_path.write_text(
        json.dumps(
            {
                "status": "passed",
                "provenance": "external_dynamic",
                "short_bag_path": "data/external/derived/short",
                "source_checksum": "source-sha256",
            }
        ),
        encoding="utf-8",
    )

    # When: the runtime boundary reads it.
    replay = read_external_short_replay(result_path)

    # Then: path and source checksum remain separate and explicit.
    assert replay.short_bag_path == Path("data/external/derived/short")
    assert replay.source_checksum == "source-sha256"


def test_given_bag_path_when_player_command_built_then_rate_clock_and_topics_are_bounded() -> None:
    # Given: one canonical fixture path.
    bag_path = Path("data/external/derived/short")

    # When: the owned player command is built.
    command = bag_play_command(bag_path)

    # Then: replay is 1.0x, owns clock, has no loop, and selects only raw inputs.
    assert command[:4] == ("ros2", "bag", "play", str(bag_path))
    assert command[command.index("--rate") + 1] == "1.0"
    assert command[command.index("--clock") + 1] == "100"
    assert command[-3:] == (
        "/utlidar/cloud",
        "/utlidar/robot_odom",
        "--disable-keyboard-controls",
    )


def test_given_mapping_variants_when_launch_commands_built_then_each_profile_reaches_its_owner() -> None:
    stationary, = mapping_launch_commands(MappingInputVariant.PROJECT_STATIONARY)
    external_mapping, external_odometry = mapping_launch_commands(
        MappingInputVariant.EXTERNAL_DYNAMIC_SHORT
    )

    assert "execution_mode:=onboard" in stationary
    assert not any(argument.startswith("continuity_profile:=") for argument in stationary)
    assert "execution_mode:=external_replay" in external_mapping
    assert not any(
        argument.startswith("continuity_profile:=")
        for argument in external_mapping
    )
    assert "continuity_profile:=replay_enforce" in external_odometry
