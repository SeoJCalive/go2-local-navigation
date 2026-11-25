"""Check real ROS cloud/odom output and the Stage 6 launch composition."""

import math
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Final
from xml.etree import ElementTree


import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from launch_test_support import evaluate

PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]


def test_given_mujoco_state_when_cloud_is_published_then_matching_source_odometry_is_published(
    monkeypatch, tmp_path,
) -> None:
    """Yaw +90 degrees maps world +Y to child +X at the base origin."""
    monkeypatch.setenv("ROS_DOMAIN_ID", "41")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    monkeypatch.setenv(
        "CYCLONEDDS_URI", (PACKAGE_ROOT / "test/headless/cyclonedds_w03.xml").as_uri()
    )
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2, JointState
    from unitree_go.msg import LowState, SportModeState

    simulator = Path(os.environ["UNITREE_MUJOCO_ROOT"])
    site = ElementTree.parse(simulator / "unitree_robots/go2/go2.xml").find(
        ".//site[@name='imu']"
    )
    # The expected base is (1,2,1); the independent MJCF site geometry determines
    # the supplied IMU position. Rz(+90) maps (x,y,z) to (-y,x,z).
    x, y, z = (float(value) for value in site.attrib["pos"].split())
    low = LowState()
    for index in range(12):
        low.motor_state[index].q = index * 0.03125
    sport = SportModeState()
    sport.position = [1.0 - y, 2.0 + x, 1.0 + z]
    sport.imu_state.quaternion = [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)]
    sport.velocity = [0.0, 2.0, 0.0]
    clouds = {}
    odometry = {}
    joints = {}
    rclpy.init(args=[])
    node = rclpy.create_node("w02_odometry_contract")

    def stamp(message):
        return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec

    subscriptions = [
        node.create_subscription(
            PointCloud2, "/w02_contract/cloud",
            lambda message: clouds.update({stamp(message): message}), 10),
        node.create_subscription(
            Odometry, "/w02_contract/odom",
            lambda message: odometry.update({stamp(message): message}), 10),
        node.create_subscription(
            JointState, "/w02_contract/joints",
            lambda message: joints.update({stamp(message): message}), 10),
    ]
    low_pub = node.create_publisher(LowState, "/w02_contract/lowstate", 10)
    sport_pub = node.create_publisher(SportModeState, "/w02_contract/sport", 10)
    command = [
        os.environ["GO2_LIDAR_EXECUTABLE"], "--ros-args",
        "-p", f"model_path:={simulator / 'unitree_robots/go2/scene_terrain.xml'}",
        "-p", "horizontal_samples:=16", "-p", "vertical_samples:=1",
        "-p", "low_state_topic:=/w02_contract/lowstate",
        "-p", "sport_state_topic:=/w02_contract/sport",
    ]
    for source, target in (
        ("/utlidar/cloud", "/w02_contract/cloud"),
        ("/utlidar/robot_odom", "/w02_contract/odom"),
        ("/simulation/utlidar/cloud_self_filtered", "/w02_contract/filtered"),
        ("/simulation/utlidar/cloud_planar_selected", "/w02_contract/planar"),
        ("/clock", "/w02_contract/clock"),
        ("/joint_states", "/w02_contract/joints"),
    ):
        command += ["-r", f"{source}:={target}"]
    process = None
    try:
        with (tmp_path / "lidar.log").open("w") as log:
            process = subprocess.Popen(
                command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 10
            step = 0
            while time.monotonic() < deadline and not (
                    clouds.keys() & odometry.keys() & joints.keys()):
                assert process.poll() is None, (tmp_path / "lidar.log").read_text()
                low.tick = 1000 + step * 2
                sport.stamp.sec = low.tick // 1000
                sport.stamp.nanosec = (low.tick % 1000) * 1000000
                low_pub.publish(low)
                sport_pub.publish(sport)
                rclpy.spin_once(node, timeout_sec=0.02)
                step += 1
            common = clouds.keys() & odometry.keys() & joints.keys()
            assert common, (tmp_path / "lidar.log").read_text()
            output = odometry[min(common)]
            assert output.header.frame_id == "odom"
            assert output.child_frame_id == "base_link"
            assert clouds[min(common)].header.frame_id == "utlidar_lidar"
            assert math.dist(
                [getattr(output.pose.pose.position, axis) for axis in "xyz"],
                [1.0, 2.0, 1.0],
            ) < 1e-6
            assert abs(output.twist.twist.linear.x - 2.0) < 1e-6
            assert abs(output.twist.twist.linear.y) < 1e-6
            joint = joints[min(common)]
            assert joint.name == [
                "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"]
            assert list(joint.position) == [index * 0.03125 for index in range(12)]
    finally:
        if process is not None:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            assert not Path(f"/proc/{process.pid}").exists()
        node.destroy_node()
        rclpy.shutdown()


def test_stage6_composes_external_inputs_and_single_slam():
    _, nodes, includes = evaluate("go2_mujoco_lidar", "go2_mujoco_stage6_slam.launch.py")
    assert not nodes
    assert len(includes) == 3
    slam = next(item for item in includes if "go2_slam_mapping" in str(item["path"]))
    assert slam["arguments"]["start_inputs"] == "false"
    assert slam["arguments"]["use_sim_time"] == "true"
