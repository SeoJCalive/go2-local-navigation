"""Installed diagnostic composition; no Domain 1 participant is started."""
from pathlib import Path
import subprocess
import pytest
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from launch_test_support import evaluate



@pytest.mark.parametrize("filename", [
    "go2_mujoco_stage62_upright_hold.launch.py", "go2_mujoco_stage62_slam.launch.py"])
def test_caller_raster_and_filter_reach_include(filename):
    override = {"horizontal_samples": "64", "vertical_samples": "3",
                "vertical_min": "-0.1", "vertical_max": "0.2", "environment_only": "true"}
    _, nodes, includes = evaluate("go2_mujoco_lidar", filename, override)
    target = next(i for i in includes if "stage45" in str(i["path"]) or "upright" in str(i["path"]))
    assert all(target["arguments"][key] == value for key, value in override.items())


def test_slam_disables_duplicate_inputs():
    _, nodes, includes = evaluate("go2_mujoco_lidar", "go2_mujoco_stage62_slam.launch.py")
    assert not nodes
    assert len(includes) == 3
    slam = next(i for i in includes if "go2_slam_mapping" in str(i["path"]))
    assert slam["arguments"] == {"start_inputs": "false", "use_sim_time": "true"}


def test_upright_rejects_isolated_non_one_domain_before_command_owner(monkeypatch):
    from ament_index_python.packages import get_package_prefix
    monkeypatch.setenv("ROS_DOMAIN_ID", "41")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    executable = Path(get_package_prefix("go2_mujoco_lidar")) / "lib/go2_mujoco_lidar/go2_mujoco_upright_hold"
    result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=10)
    assert result.returncode != 0
    assert "requires ROS_DOMAIN_ID=1" in result.stderr
