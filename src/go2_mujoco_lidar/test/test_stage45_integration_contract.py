"""Evaluate installed Stage45 composition with an isolated domain."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from launch_test_support import evaluate

def test_stage45_composes_one_sensor_and_planar_mapping_input(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "41")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    _, nodes, includes = evaluate("go2_mujoco_lidar", "go2_mujoco_stage45.launch.py")
    assert [n["executable"] for n in nodes] == ["mujoco_lidar_node"]
    assert len(includes) == 1
    assert includes[0]["arguments"]["raw_cloud_topic"] == (
        "/simulation/utlidar/cloud_planar_selected")
