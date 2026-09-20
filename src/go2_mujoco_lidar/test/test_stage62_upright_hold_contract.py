from pathlib import Path
from typing import Final
from xml.etree import ElementTree


PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
CONTROLLER_SOURCE: Final = PACKAGE_ROOT / "src" / "upright_hold_node.cpp"
LAUNCH_SOURCE: Final = PACKAGE_ROOT / "launch" / "go2_mujoco_stage62_upright_hold.launch.py"
SLAM_LAUNCH_SOURCE: Final = PACKAGE_ROOT / "launch" / "go2_mujoco_stage62_slam.launch.py"
STAGE45_LAUNCH_SOURCE: Final = PACKAGE_ROOT / "launch" / "go2_mujoco_stage45.launch.py"
DDS_CONFIGURATION: Final = PACKAGE_ROOT / "config" / "cyclonedds_loopback.xml"
SCENE_TEMPLATE: Final = PACKAGE_ROOT / "scene" / "go2_indoor.xml.in"


def test_given_upright_hold_when_built_then_it_uses_official_lowcmd_semantics() -> None:
    source = CONTROLLER_SOURCE.read_text(encoding="utf-8")

    assert '#include <motor_crc.h>' in source
    assert 'kLowCmdTopic[] = "/lowcmd"' in source
    assert "std::array<double, 12> kStandUpJointPositions" in source
    assert "std::array<double, 12> kStandDownJointPositions" in source
    assert "std::tanh(elapsed_seconds_ / kRampSeconds)" in source
    assert "get_crc(command_);" in source
    assert "motor.tau = 0.0;" in source
    assert "cmd_vel" not in source
    assert "sport" not in source


def test_given_controller_when_started_then_domain_one_loopback_is_required() -> None:
    source = CONTROLLER_SOURCE.read_text(encoding="utf-8")

    assert 'std::string{domain} != "1"' in source
    assert '"name=\\\"lo\\\""' in source
    assert '"127.0.0.1"' in source
    assert "requires ROS_DOMAIN_ID=1" in source
    assert 'requires CycloneDDS loopback lo and 127.0.0.1' in source


def test_given_stage62_launch_when_generated_then_only_loopback_domain_one_is_wired() -> None:
    source = LAUNCH_SOURCE.read_text(encoding="utf-8")

    assert 'SetEnvironmentVariable("ROS_DOMAIN_ID", "1")' in source
    assert 'SetEnvironmentVariable(\n                "CYCLONEDDS_URI"' in source
    assert 'executable="go2_mujoco_upright_hold"' in source
    assert '"model_path": scene_path' in source
    assert "ExecuteProcess" not in source


def test_given_stage62_slam_when_started_then_upright_inputs_feed_one_slam_owner() -> None:
    source = SLAM_LAUNCH_SOURCE.read_text(encoding="utf-8")

    assert '"go2_mujoco_stage62_upright_hold.launch.py"' in source
    assert '"go2_odometry_adapter.launch.py"' in source
    assert '"go2_slam_mapping.launch.py"' in source
    assert '"start_inputs": "false"' in source
    assert '"use_sim_time": "true"' in source


def test_given_horizontal_resolution_when_selected_then_it_reaches_the_lidar_node() -> None:
    stage45 = STAGE45_LAUNCH_SOURCE.read_text(encoding="utf-8")
    upright = LAUNCH_SOURCE.read_text(encoding="utf-8")
    slam = SLAM_LAUNCH_SOURCE.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("horizontal_samples", default_value="360")' in stage45
    assert 'horizontal_samples = LaunchConfiguration("horizontal_samples")' in stage45
    assert '"horizontal_samples": horizontal_samples' in stage45
    assert 'DeclareLaunchArgument("horizontal_samples", default_value="360")' in upright
    assert '"horizontal_samples": horizontal_samples' in upright
    assert 'DeclareLaunchArgument("horizontal_samples", default_value="360")' in slam
    assert '"horizontal_samples": horizontal_samples' in slam


def test_given_vertical_layers_when_selected_then_they_reach_the_lidar_node() -> None:
    stage45 = STAGE45_LAUNCH_SOURCE.read_text(encoding="utf-8")
    upright = LAUNCH_SOURCE.read_text(encoding="utf-8")
    slam = SLAM_LAUNCH_SOURCE.read_text(encoding="utf-8")

    for source in (stage45, upright, slam):
        assert 'DeclareLaunchArgument("vertical_samples", default_value="8")' in source
        assert 'DeclareLaunchArgument("vertical_min", default_value="-0.2617993877991494")' in source
        assert 'DeclareLaunchArgument("vertical_max", default_value="0.2617993877991494")' in source
        assert '"vertical_samples": vertical_samples' in source
        assert '"vertical_min": vertical_min' in source
        assert '"vertical_max": vertical_max' in source


def test_given_environment_only_raycast_when_selected_then_robot_geometry_is_excluded() -> None:
    model_source = (PACKAGE_ROOT / "src" / "lidar_model.cpp").read_text(encoding="utf-8")
    node_source = (PACKAGE_ROOT / "src" / "mujoco_lidar_node.cpp").read_text(encoding="utf-8")
    stage45_source = STAGE45_LAUNCH_SOURCE.read_text(encoding="utf-8")
    upright_source = LAUNCH_SOURCE.read_text(encoding="utf-8")
    slam_source = SLAM_LAUNCH_SOURCE.read_text(encoding="utf-8")

    assert "environment_only" in model_source
    assert 'declare_parameter<bool>("environment_only", false)' in node_source
    assert 'DeclareLaunchArgument("environment_only", default_value="false")' in stage45_source
    assert '"environment_only": environment_only' in stage45_source
    assert 'DeclareLaunchArgument("environment_only", default_value="false")' in upright_source
    assert '"environment_only": environment_only' in upright_source
    assert 'DeclareLaunchArgument("environment_only", default_value="false")' in slam_source
    assert '"environment_only": environment_only' in slam_source


def test_given_loopback_dds_when_loaded_then_local_unicast_discovery_is_complete() -> None:
    root = ElementTree.parse(DDS_CONFIGURATION).getroot()
    namespace = {"dds": "https://cdds.io/config"}

    interface = root.find(".//dds:NetworkInterface", namespace)
    peer = root.find(".//dds:Peer", namespace)
    participant_limit = root.find(".//dds:MaxAutoParticipantIndex", namespace)

    assert interface is not None and interface.attrib["name"] == "lo"
    assert peer is not None and peer.attrib["Address"] == "127.0.0.1"
    assert participant_limit is not None and int(participant_limit.text or "0") >= 120


def test_given_indoor_scene_template_when_parsed_then_it_has_the_required_named_geometry() -> None:
    root = ElementTree.parse(SCENE_TEMPLATE).getroot()
    included = root.find("include")
    compiler = root.find("compiler")
    names = {geometry.attrib["name"] for geometry in root.findall(".//geom")}

    assert included is not None
    assert included.attrib["file"] == "@GO2_XML@"
    assert compiler is not None
    assert compiler.attrib["meshdir"] == "@GO2_ASSETS@"
    assert {
        "indoor_floor_6m",
        "indoor_wall_north",
        "indoor_wall_south",
        "indoor_wall_east",
        "indoor_wall_west",
        "indoor_l_wall_horizontal",
        "indoor_l_wall_vertical",
        "indoor_box_pillar",
    } <= names
