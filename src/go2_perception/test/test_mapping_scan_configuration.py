from pathlib import Path
from typing import Final

import pytest
import yaml

PACKAGE_ROOT: Final = Path(__file__).parents[1]
CONFIG_PATH: Final = PACKAGE_ROOT / "config" / "mapping_scan.yaml"
LAUNCH_PATH: Final = PACKAGE_ROOT / "launch" / "go2_mapping_scan.launch.py"


def test_given_mapping_scan_config_when_loaded_then_validated_cloud_is_converted_to_base_scan() -> (
    None
):
    with CONFIG_PATH.open(encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    contract = config["mapping_scan"]
    parameters = contract["pointcloud_to_laserscan"]["ros__parameters"]

    assert contract["input"]["topic"] == "/go2_mapping/cloud_validated"
    assert contract["output"]["topic"] == "/scan"
    assert parameters["target_frame"] == "base"
    assert parameters["min_height"] == -0.25
    assert "queue_size" not in parameters
    assert parameters["min_height"] < parameters["max_height"]
    assert parameters["range_min"] < parameters["range_max"]
    assert contract["provenance"]["parameter_status"] == "engineering_candidate"
    assert contract["runtime"] == {
        "domain_id": 62,
        "loopback_only": True,
        "use_sim_time": True,
        "clock_owner": "rosbag_player",
        "global_map_to_odom_owners": 0,
    }


def test_given_mapping_launch_when_read_then_it_uses_upstream_converter_and_existing_static_tf() -> (
    None
):
    launch_source = LAUNCH_PATH.read_text(encoding="utf-8")

    assert 'package="pointcloud_to_laserscan"' in launch_source
    assert 'executable="pointcloud_to_laserscan_node"' in launch_source
    assert '"cloud_in", "/go2_mapping/cloud_validated"' in launch_source
    assert '"scan", "/scan"' in launch_source
    assert "go2_static_tf.launch.py" in launch_source
    assert 'executable="mapping_cloud_gate"' in launch_source
    assert '"use_sim_time"' in launch_source
    assert "PointCloud2" not in launch_source
    assert "yaml.safe_load" in launch_source
    assert "parameters=[scan_config" not in launch_source
    assert "parameters=[scan_parameters, sim_time_parameter]" not in launch_source
    assert "converter_parameters = scan_parameters.copy()" in launch_source
    assert 'converter_parameters["min_height"] = profile.converter_min_height' in launch_source
    assert 'converter_parameters["queue_size"] = profile.converter_queue_size' in launch_source
    assert "parameters=[converter_parameters, sim_time_parameter]" in launch_source
    assert "scan_projection_profile" in launch_source
    assert 'executable="mapping_cloud_accumulator"' in launch_source


@pytest.mark.parametrize(
    ("profile_id", "execution_mode"),
    (
        ("raw_single", "onboard"),
        ("raw_single", "external_replay"),
        ("dimos_odom_accumulated_emit3", "external_replay"),
    ),
)
def test_given_allowed_execution_mode_when_mapping_scan_profile_loads_then_it_is_available(
    profile_id: str,
    execution_mode: str,
) -> None:
    # Given: generic raw profile와 DimOS external-replay profile의 허용 mode
    from go2_perception.mapping_scan_profiles import load_mapping_scan_profile

    # When: mapping launch loader가 profile과 execution mode를 함께 파싱한다.
    profile = load_mapping_scan_profile(CONFIG_PATH, profile_id, execution_mode)

    # Then: 허용 조합만 선택된 profile ID를 보존한다.
    assert profile.profile_id == profile_id


@pytest.mark.parametrize(
    ("profile_id", "execution_mode", "reason_code"),
    (
        (
            "dimos_odom_accumulated_emit3",
            "onboard",
            "mapping_scan_profile_external_replay_required",
        ),
        (
            "raw_single",
            "unsupported_mode",
            "unknown_mapping_scan_execution_mode",
        ),
    ),
)
def test_given_disallowed_execution_mode_when_mapping_scan_profile_loads_then_typed_error_is_raised(
    profile_id: str,
    execution_mode: str,
    reason_code: str,
) -> None:
    # Given: DimOS replay-only scope 또는 등록되지 않은 execution mode
    from go2_perception.mapping_scan_profiles import (
        MappingScanProfileError,
        load_mapping_scan_profile,
    )

    # When/Then: mapping node 생성 전에 typed loader error로 거부한다.
    with pytest.raises(MappingScanProfileError) as raised:
        load_mapping_scan_profile(CONFIG_PATH, profile_id, execution_mode)

    assert raised.value.reason_code == reason_code


def test_given_mapping_launch_chain_when_read_then_execution_mode_reaches_profile_loaders() -> None:
    # Given: Nav2 mapping, perception scan, static TF launch source
    mapping_source = (
        PACKAGE_ROOT.parent / "go2_nav2/launch/go2_slam_mapping.launch.py"
    ).read_text(encoding="utf-8")
    scan_source = LAUNCH_PATH.read_text(encoding="utf-8")
    static_tf_source = (
        PACKAGE_ROOT.parent / "bringup/launch/go2_static_tf.launch.py"
    ).read_text(encoding="utf-8")

    # When/Then: onboard 기본 mode가 선언되고 한 값이 모든 include 경계를 통과한다.
    assert 'DeclareLaunchArgument("execution_mode", default_value="onboard")' in mapping_source
    assert 'DeclareLaunchArgument("execution_mode", default_value="onboard")' in scan_source
    assert 'DeclareLaunchArgument("execution_mode", default_value="onboard")' in static_tf_source
    assert '"execution_mode": execution_mode' in mapping_source
    assert '"execution_mode": execution_mode' in scan_source
    assert 'LaunchConfiguration("execution_mode")' in scan_source


def test_given_accumulated_profiles_when_loaded_then_input_depth_is_profile_driven_without_output_widening() -> None:
    # Given: accumulated profile registry, launch adapter와 accumulator node source
    from go2_perception.mapping_scan_profiles import load_mapping_scan_profile

    profiles = tuple(
        load_mapping_scan_profile(CONFIG_PATH, profile_id, "external_replay")
        for profile_id in (
            "dimos_odom_accumulated",
            "dimos_odom_accumulated_emit10",
        )
    )
    launch_source = LAUNCH_PATH.read_text(encoding="utf-8")
    node_source = (
        PACKAGE_ROOT / "go2_perception/mapping_cloud_accumulator_node.py"
    ).read_text(encoding="utf-8")

    # When/Then: input subscription과 retry queue만 depth 64를 받고 output은 기존 QoS다.
    assert tuple(profile.input_qos_depth for profile in profiles) == (64, 64)
    assert tuple(profile.retry_queue_capacity for profile in profiles) == (64, 64)
    assert '"input_qos_depth": profile.input_qos_depth' in launch_source
    assert '"retry_queue_capacity": profile.retry_queue_capacity' in launch_source
    assert '"emit_every": profile.emit_every' in launch_source
    assert "input_qos" in node_source
    assert "HistoryPolicy.KEEP_LAST" in node_source
    assert "depth=input_qos_depth" in node_source
    assert "output_topic,\n            POINT_CLOUD_QOS" in node_source


def test_given_emit10_profile_when_loaded_then_only_it_has_converter_overrides() -> None:
    from go2_perception.mapping_scan_profiles import load_mapping_scan_profile

    raw = load_mapping_scan_profile(CONFIG_PATH, "raw_single", "onboard")
    accumulated = load_mapping_scan_profile(
        CONFIG_PATH,
        "dimos_odom_accumulated",
        "external_replay",
    )
    emit10 = load_mapping_scan_profile(
        CONFIG_PATH,
        "dimos_odom_accumulated_emit10",
        "external_replay",
    )

    assert raw.converter_min_height is None
    assert raw.converter_queue_size is None
    assert accumulated.converter_min_height is None
    assert accumulated.converter_queue_size is None
    assert emit10.converter_min_height == -0.10
    assert emit10.converter_queue_size == 64


@pytest.mark.parametrize(
    ("converter_override", "reason_code"),
    (
        ({"min_height": float("nan")}, "mapping_scan_profile_numeric_invalid"),
        ({"queue_size": 0}, "mapping_scan_profile_integer_invalid"),
    ),
)
def test_given_invalid_converter_override_when_loaded_then_parser_rejects_it(
    tmp_path: Path,
    converter_override: dict[str, float | int],
    reason_code: str,
) -> None:
    from go2_perception.mapping_scan_profiles import (
        MappingScanProfileError,
        load_mapping_scan_profile,
    )

    with CONFIG_PATH.open(encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    config["mapping_scan"]["projection_profiles"]["profiles"][
        "dimos_odom_accumulated_emit10"
    ]["converter_override"] = converter_override
    invalid_config_path = tmp_path / "mapping_scan.yaml"
    invalid_config_path.write_text(yaml.safe_dump(config), encoding="utf-8")

    with pytest.raises(MappingScanProfileError) as raised:
        load_mapping_scan_profile(
            invalid_config_path,
            "dimos_odom_accumulated_emit10",
            "external_replay",
        )

    assert raised.value.reason_code == reason_code
