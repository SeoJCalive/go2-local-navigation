import ast
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).parents[3]
VALIDATION_ROOT = PROJECT_ROOT / "src/go2_validation"
NAV2_ROOT = PROJECT_ROOT / "src/go2_nav2"
PERCEPTION_ROOT = PROJECT_ROOT / "src/go2_perception"


def _class_fields(source: str, class_name: str) -> set[str]:
    tree = ast.parse(source)
    class_node = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return {
        statement.target.id
        for statement in class_node.body
        if isinstance(statement, ast.AnnAssign) and isinstance(statement.target, ast.Name)
    }


def test_given_emit3_profile_when_registry_is_loaded_then_dimos_replay_contract_is_exact() -> None:
    # Given: stable DimOS replay-only profile registry.
    from go2_perception.mapping_scan_profiles import load_mapping_scan_profile

    document = yaml.safe_load(
        (PERCEPTION_ROOT / "config/mapping_scan.yaml").read_text(encoding="utf-8")
    )

    # When: emit3 profile contract를 읽는다.
    profile = document["mapping_scan"]["projection_profiles"]["profiles"][
        "dimos_odom_accumulated_emit3"
    ]

    # Then: frame3/emit3, converter override와 DimOS provenance가 보존된다.
    assert profile["status"] == "engineering_candidate"
    assert profile["scope"] == "dimos_external_replay_only"
    assert profile["converter_input_topic"] == "/go2_mapping/cloud_accumulated"
    assert profile["converter_override"] == {"min_height": -0.10, "queue_size": 64}
    assert profile["accumulator"] == {
        "enabled": True,
        "frame_limit": 3,
        "emit_every": 3,
        "input_qos_depth": 64,
        "retry_queue_capacity": 64,
        "target_frame": "odom",
        "output_topic": "/go2_mapping/cloud_accumulated",
    }
    assert profile["source"] == document["mapping_scan"]["projection_profiles"][
        "profiles"
    ]["dimos_odom_accumulated_emit10"]["source"]
    assert profile["physical_suitability"] == "unverified"
    loaded_profile = load_mapping_scan_profile(
        PERCEPTION_ROOT / "config/mapping_scan.yaml",
        "dimos_odom_accumulated_emit3",
        "external_replay",
    )
    assert loaded_profile.frame_limit == 3
    assert loaded_profile.emit_every == 3
    assert loaded_profile.converter_min_height == -0.10
    assert loaded_profile.converter_queue_size == 64


def test_given_emit3_profile_when_cloud_accounting_is_selected_then_terminal_marker_is_required(
    tmp_path: Path,
) -> None:
    # Given: terminal marker가 없는 owned launch log.
    from go2_validation.mapping_cloud_accounting import (
        MappingCloudAccountingError,
        mapping_cloud_accounting_for_profile,
    )

    log_path = tmp_path / "launch.log"
    log_path.write_text("shutdown complete\n", encoding="utf-8")

    # When: emit3 profile의 terminal accounting을 선택한다.
    try:
        mapping_cloud_accounting_for_profile(
            log_path,
            "dimos_odom_accumulated_emit3",
        )
    except MappingCloudAccountingError as error:
        rejection = error
    else:
        raise AssertionError("emit3 must require terminal cloud accounting")

    # Then: raw profile처럼 marker 없이 통과하지 않는다.
    assert rejection.reason_code == "mapping_cloud_accounting_missing"


def test_given_stable_emit3_when_wiring_sources_are_read_then_coarse_override_reaches_result() -> None:
    # Given: replay launch, command, runtime, acceptance와 canonical A/B runner source.
    acceptance_source = (VALIDATION_ROOT / "go2_validation/mapping_acceptance.py").read_text(
        encoding="utf-8"
    )
    command_source = (VALIDATION_ROOT / "go2_validation/mapping_command_builders.py").read_text(
        encoding="utf-8"
    )
    runtime_source = (VALIDATION_ROOT / "go2_validation/mapping_runtime_execution.py").read_text(
        encoding="utf-8"
    )
    launch_source = (NAV2_ROOT / "launch/go2_slam_mapping.launch.py").read_text(
        encoding="utf-8"
    )
    runner_source = (VALIDATION_ROOT / "go2_validation/mapping_scan_profile_ab_runner.py").read_text(
        encoding="utf-8"
    )

    # When: machine-consumed field·argv·typed launch override와 A/B values를 확인한다.
    observation_fields = _class_fields(acceptance_source, "MappingObservation")
    result_fields = _class_fields(acceptance_source, "MappingResult")
    spec_fields = _class_fields(runtime_source, "MappingVariantSpec")

    # Then: default 0.349와 candidate 0.1745가 YAML 뒤 override를 거쳐 provenance에 남는다.
    assert "coarse_search_angle_offset" in observation_fields
    assert "coarse_search_angle_offset" in result_fields
    assert "coarse_search_angle_offset" in spec_fields
    assert "MappingLaunchConfiguration" in command_source
    assert "coarse_search_angle_offset" in command_source
    assert "coarse_search_angle_offset:=" in command_source
    assert 'DeclareLaunchArgument("coarse_search_angle_offset", default_value="0.349")' in launch_source
    assert '"coarse_search_angle_offset": ParameterValue(' in launch_source
    assert "coarse_search_angle_offset," in launch_source
    assert "value_type=float," in launch_source
    assert launch_source.index("slam_parameters,") < launch_source.index(
        "coarse_search_angle_offset_parameter,"
    )
    assert 'CANDIDATE_SCAN_PROFILE: Final = "dimos_odom_accumulated_emit3"' in runner_source
    assert "BASELINE_COARSE_SEARCH_ANGLE_OFFSET: Final = 0.349" in runner_source
    assert "CANDIDATE_COARSE_SEARCH_ANGLE_OFFSET: Final = 0.1745" in runner_source
    assert "do_loop_closing=True" in runner_source
