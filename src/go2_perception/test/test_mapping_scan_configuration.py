"""프로젝트 2D scan projection 설정을 검증한다."""

from pathlib import Path
from typing import Final

import pytest
import yaml

from go2_perception.mapping_scan_profiles import (
    MappingScanProfileError,
    load_mapping_scan_profile,
)


PACKAGE_ROOT: Final = Path(__file__).parents[1]
CONFIG_PATH: Final = PACKAGE_ROOT / "config/mapping_scan.yaml"
LAUNCH_PATH: Final = PACKAGE_ROOT / "launch/go2_mapping_scan.launch.py"


def test_given_mapping_scan_config_when_loaded_then_cloud_is_converted_to_base_scan() -> None:
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    contract = config["mapping_scan"]
    parameters = contract["pointcloud_to_laserscan"]["ros__parameters"]

    assert contract["input"]["topic"] == "/go2_mapping/cloud_validated"
    assert contract["output"]["topic"] == "/scan"
    assert parameters["target_frame"] == "base"
    assert parameters["min_height"] < parameters["max_height"]
    assert parameters["range_min"] < parameters["range_max"]
    assert set(contract["projection_profiles"]["profiles"]) == {"raw_single"}


@pytest.mark.parametrize(
    ("profile_id", "execution_mode"),
    (
        ("raw_single", "onboard"),
    ),
)
def test_given_allowed_mode_when_scan_profile_loads_then_selected_profile_is_returned(
    profile_id: str,
    execution_mode: str,
) -> None:
    assert load_mapping_scan_profile(
        CONFIG_PATH,
        profile_id,
        execution_mode,
    ).profile_id == profile_id


@pytest.mark.parametrize(
    ("profile_id", "execution_mode", "reason_code"),
    (
        ("raw_single", "unsupported", "unknown_mapping_scan_execution_mode"),
    ),
)
def test_given_disallowed_mode_when_scan_profile_loads_then_typed_error_is_raised(
    profile_id: str,
    execution_mode: str,
    reason_code: str,
) -> None:
    with pytest.raises(MappingScanProfileError) as raised:
        load_mapping_scan_profile(CONFIG_PATH, profile_id, execution_mode)

    assert raised.value.reason_code == reason_code


def test_given_invalid_converter_override_when_loaded_then_parser_rejects_it(
    tmp_path: Path,
) -> None:
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    config["mapping_scan"]["projection_profiles"]["profiles"]["raw_single"][
        "converter_override"
    ] = {"queue_size": 0}
    invalid_path = tmp_path / "mapping_scan.yaml"
    invalid_path.write_text(yaml.safe_dump(config), encoding="utf-8")

    with pytest.raises(MappingScanProfileError) as raised:
        load_mapping_scan_profile(invalid_path, "raw_single", "onboard")

    assert raised.value.reason_code == "mapping_scan_profile_integer_invalid"


def test_given_mapping_launch_when_read_then_inputs_reach_converter() -> None:
    source = LAUNCH_PATH.read_text(encoding="utf-8")

    assert 'executable="mapping_cloud_gate"' in source
    assert 'executable="pointcloud_to_laserscan_node"' in source
    assert '("cloud_in", profile.converter_input_topic)' in source
    assert '("scan", "/scan")' in source
    assert '"raw_cloud_topic",' in source
    assert "mapping_cloud_accumulator" not in source
    assert '"execution_mode": execution_mode' in source
    assert '"use_sim_time": use_sim_time' in source
    assert 'converter_parameters["min_height"] = parsed_min_height' in source
    assert 'DeclareLaunchArgument("converter_min_height", default_value="")' in source
