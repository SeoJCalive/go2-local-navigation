"""프로젝트 2D scan projection 설정을 검증한다."""

from pathlib import Path
from typing import Final

import pytest
import yaml
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from launch_test_support import evaluate



from go2_inputs.mapping_scan_profiles import (
    MappingScanProfileError,
    load_mapping_scan_profile,
)


PACKAGE_ROOT: Final = Path(__file__).parents[1]
CONFIG_PATH: Final = PACKAGE_ROOT / "config/mapping_scan.yaml"


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


def test_mapping_launch_evaluates_typed_time_and_converter() -> None:
    _, nodes, includes = evaluate("go2_inputs", "go2_mapping_scan.launch.py",
                                  {"use_sim_time": "false"})
    assert {n["executable"] for n in nodes} == {
        "mapping_cloud_gate", "pointcloud_to_laserscan_node"}
    assert len(includes) == 1
    for node in nodes:
        assert any(group.get("use_sim_time") is False for group in node["parameters"])
    converter = next(n for n in nodes if n["executable"] == "pointcloud_to_laserscan_node")
    assert converter["parameters"][0]["target_frame"] == "base"
