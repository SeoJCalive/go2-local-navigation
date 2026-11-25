"""실제 LiDAR process에 유효/무효 state를 넣어 오류 예산의 결과를 검사한다.

Domain 41과 전용 discovery tag를 사용하고 시작한 PID만 종료한다. zero quaternion은
회전으로 정의될 수 없으므로 무효 oracle이며, identity quaternion은 유효 대조군이다.
"""

import os
from pathlib import Path
import signal
import subprocess
import time

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("limit", [2, 3])
def test_consecutive_failures_exit_but_success_resets_budget(monkeypatch, tmp_path, limit):
    """K-1 errors survive; success clears them; the Kth subsequent error exits non-zero."""
    monkeypatch.setenv("ROS_DOMAIN_ID", "41")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    monkeypatch.setenv("CYCLONEDDS_URI", (ROOT / "test/headless/cyclonedds_w03.xml").as_uri())
    import rclpy
    from nav_msgs.msg import Odometry
    from unitree_go.msg import LowState, SportModeState

    rclpy.init(args=[])
    node = rclpy.create_node("w03_error_budget_observer")
    low_pub = node.create_publisher(LowState, "/w03/low", 10)
    sport_pub = node.create_publisher(SportModeState, "/w03/sport", 10)
    outputs = []
    subscription = node.create_subscription(Odometry, "/w03/odom", outputs.append, 10)
    command = [
        os.environ["GO2_LIDAR_EXECUTABLE"], "--ros-args",
        "-p", f"model_path:={os.environ['UNITREE_MUJOCO_ROOT']}/unitree_robots/go2/scene_terrain.xml",
        "-p", "horizontal_samples:=16", "-p", "vertical_samples:=1",
        "-p", f"raycast_error_limit:={limit}",
        "-p", "low_state_topic:=/w03/low", "-p", "sport_state_topic:=/w03/sport",
        "-r", "/utlidar/robot_odom:=/w03/odom", "-r", "/clock:=/w03/clock",
    ]
    log_path = tmp_path / "lidar.log"
    process = None
    try:
        with log_path.open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            failures = 0
            tick = 1000

            def sample(valid):
                nonlocal failures, tick
                tick += 100
                low = LowState()
                low.tick = tick
                sport = SportModeState()
                sport.stamp.sec, milliseconds = divmod(tick, 1000)
                sport.stamp.nanosec = milliseconds * 1000000
                sport.imu_state.quaternion = [1.0 if valid else 0.0, 0.0, 0.0, 0.0]
                previous_outputs = len(outputs)
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline:
                    low_pub.publish(low)
                    sport_pub.publish(sport)
                    rclpy.spin_once(node, timeout_sec=0.02)
                    text = log_path.read_text()
                    if valid and len(outputs) > previous_outputs:
                        return
                    if not valid and text.count(": raycast error ") > failures:
                        failures += 1
                        return
                    assert process.poll() is None, text
                pytest.fail(log_path.read_text())

            for _ in range(limit - 1):
                sample(False)
                assert process.poll() is None
            sample(True)
            assert outputs
            for _ in range(limit - 1):
                sample(False)
                assert process.poll() is None
            sample(False)
            assert process.wait(timeout=5) == 1, log_path.read_text()
            assert "consecutive raycast error budget exhausted" in log_path.read_text()
            assert log_path.read_text().count(": raycast error ") == 2 * limit - 1
    finally:
        if process is not None:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5)
            assert not Path(f"/proc/{process.pid}").exists()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def test_zero_error_budget_is_rejected():
    """Zero cannot express a bounded positive number of tolerated attempts."""
    result = subprocess.run([
        os.environ["GO2_LIDAR_EXECUTABLE"], "--ros-args",
        "-p", f"model_path:={os.environ['UNITREE_MUJOCO_ROOT']}/unitree_robots/go2/scene_terrain.xml",
        "-p", "raycast_error_limit:=0",
    ], capture_output=True, text=True, timeout=10)
    assert result.returncode == 1
    assert "raycast_error_limit must be positive" in result.stderr
