"""환경·/proc fixture의 sim boundary와 실제 설치 map-save CLI 실행을 검사한다."""

import importlib.util
from pathlib import Path
import subprocess

import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / "go2_mujoco_lidar/sim_session.py"
SPEC = importlib.util.spec_from_file_location("sim_session", MODULE_PATH)
SESSION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SESSION)


def test_installed_map_save_cli_runs_without_ros_startup():
    """Both copy and symlink installs must expose an executable CLI."""
    from ament_index_python.packages import get_package_prefix
    executable = (
        Path(get_package_prefix("go2_mujoco_lidar"))
        / "lib/go2_mujoco_lidar/sim_save_map")
    completed = subprocess.run(
        [str(executable), "--help"], capture_output=True, timeout=5.0)
    assert completed.returncode == 0, completed.stderr


@pytest.mark.parametrize("domain,localhost,allowed", [
    ("0", "0", False), ("0", "1", False), ("1", "0", False),
    ("1", "1", True), ("41", "1", True), ("", "1", False),
])
def test_environment_matrix(domain, localhost, allowed):
    """Domain zero is the robot default; a different DDS domain still needs loopback."""
    environment = {"ROS_LOCALHOST_ONLY": localhost}
    if domain:
        environment["ROS_DOMAIN_ID"] = domain
    if allowed:
        assert SESSION.simulation_domain(environment) == int(domain)
    else:
        with pytest.raises(RuntimeError, match="simulation refused"):
            SESSION.simulation_domain(environment)


def test_explicit_override():
    """The caller can opt into the otherwise refused execution boundary."""
    assert SESSION.simulation_domain({}, allow_unsafe=True) == 0


@pytest.mark.parametrize("interface,peer,xml_domain,allowed", [
    ("lo", "127.0.0.1", "any", True), ("eth0", "127.0.0.1", "any", False),
    ("lo", "192.168.123.161", "any", False), ("lo", "127.0.0.1", "0", False),
    ("lo", "127.0.0.1", "1", True),
])
def test_existing_domain_one_xml(tmp_path, interface, peer, xml_domain, allowed):
    """The old runbook's actual file contents determine isolation."""
    config = tmp_path / "misleading_loopback_name.xml"
    config.write_text(
        f'<CycloneDDS><Domain id="{xml_domain}"><General><Interfaces>'
        f'<NetworkInterface name="{interface}"/>'
        f'</Interfaces></General><Discovery><Peers><Peer Address="{peer}"/>'
        '</Peers></Discovery></Domain></CycloneDDS>')
    environment = {
        "ROS_DOMAIN_ID": "1", "ROS_LOCALHOST_ONLY": "0",
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp", "CYCLONEDDS_URI": config.as_uri(),
    }
    if allowed:
        assert SESSION.simulation_domain(environment) == 1
    else:
        with pytest.raises(RuntimeError, match="simulation refused"):
            SESSION.simulation_domain(environment)


def test_world_from_live_process_contract(tmp_path):
    """CLI world owns raycast selection, including simulator-relative paths and symlinks."""
    proc = tmp_path / "123"
    proc.mkdir()
    (proc / "cwd").symlink_to(tmp_path, target_is_directory=True)
    world = tmp_path / "world.xml"
    # SHA-256 of empty bytes is a published standard test vector.
    world.write_bytes(b"")
    (proc / "cmdline").write_bytes(b"\0".join(
        arg.encode() for arg in ["unitree_mujoco", "-s", "world.xml", "-i", "41", "-n", "lo"]))
    alias = tmp_path / "alias.xml"
    alias.symlink_to(world)
    path, digest = SESSION.simulator_world(123, str(alias), 41, proc_root=tmp_path)
    assert path == world
    assert digest == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
    other = tmp_path / "different.xml"
    other.write_bytes(b"")
    with pytest.raises(RuntimeError, match="world mismatch"):
        SESSION.simulator_world(123, str(other), 41, proc_root=tmp_path)
    with pytest.raises(RuntimeError, match="domain does not match"):
        SESSION.simulator_world(123, "", 1, proc_root=tmp_path)


def test_non_loopback_physics_refused(tmp_path):
    """A ROS-only loopback setting cannot hide a simulator on a physical interface."""
    proc = tmp_path / "123"
    proc.mkdir()
    (proc / "cmdline").write_bytes(b"\0".join(
        arg.encode() for arg in ["unitree_mujoco", "-s", "x", "-i", "41", "-n", "eth0"]))
    with pytest.raises(RuntimeError, match="-n lo"):
        SESSION.simulator_world(123, "", 41, proc_root=tmp_path)


@pytest.mark.parametrize("filename", [
    "go2_sim_navigation.launch.py", "go2_sim_bringup.launch.py",
    "go2_mujoco_stage45.launch.py", "go2_mujoco_stage62_upright_hold.launch.py",
    "go2_mujoco_stage62_slam.launch.py",
])
def test_all_sim_launch_defaults_define_valid_spherical_raster(filename):
    """Sampling is integral and positive; elevation stays inside spherical poles."""
    import math
    from launch.actions import DeclareLaunchArgument
    from launch import LaunchContext

    path = MODULE_PATH.parents[1] / "launch" / filename
    spec = importlib.util.spec_from_file_location("raster_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    defaults = {
        action.name: "".join(value.perform(context) for value in action.default_value)
        for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument) and action.default_value is not None
    }
    assert int(defaults["horizontal_samples"]) >= 4
    assert int(defaults["vertical_samples"]) >= 1
    assert -math.pi / 2 <= float(defaults["vertical_min"])
    assert float(defaults["vertical_min"]) <= float(defaults["vertical_max"]) <= math.pi / 2
