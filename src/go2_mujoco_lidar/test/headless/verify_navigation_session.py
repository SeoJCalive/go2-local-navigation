"""W05 Domain 41에서 mapping/save/localization과 실제 TF publisher를 검사한다.

W03 무중력 정지 simulator와 전용 discovery XML을 재사용한다. RL/FIFO bridge가
Domain 41을 거부하므로 주행·goal 성공·지도 정확도는 검증하지 않는다. 실제 bridge
설치본의 거부를 기록하고 그 전 단계인 AMCL 수락과 Nav2 active까지 검사한다.
자신이 시작한 PID만 signal/reap하며 기존 기본 install/RL/bridge PID를 전후 비교한다.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile

from verify_bringup import descendants


def existing_stack(project):
    result = {}
    names = {
        "unitree_mujoco", "go2_ctrl", "map_server", "amcl", "planner_server",
        "controller_server", "bt_navigator", "behavior_server", "async_slam_toolbox_node",
    }
    for path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            arguments = path.read_bytes().rstrip(b"\0").decode().split("\0")
            if not arguments or not arguments[0]:
                continue
            if (Path(arguments[0]).name in names
                    or any(arg.startswith(str(project / "install") + "/") for arg in arguments)
                    or any("/go2_nav2_mujoco_bridge/install/" in arg for arg in arguments)):
                result[path.parent.name] = {
                    "start_ticks": (path.parent / "stat").read_text().rsplit(")", 1)[1].split()[19],
                    "argv": arguments,
                }
        except (FileNotFoundError, ProcessLookupError):
            pass
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulator", default="/tmp/w03-headless-build/w03_headless")
    parser.add_argument("--tf-observer", default="/tmp/w05-build/go2_mujoco_lidar/tf_owner_observer")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[4]
    config = Path(__file__).with_name("cyclonedds_w03.xml").resolve()
    if (os.environ.get("ROS_DOMAIN_ID") != "41" or os.environ.get("ROS_LOCALHOST_ONLY") != "1"
            or os.environ.get("CYCLONEDDS_URI") != config.as_uri()):
        raise RuntimeError("Domain 41, localhost and W03 dedicated discovery XML required")
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    baseline = existing_stack(project)
    (output / "preexisting.json").write_text(json.dumps(baseline, indent=2))
    rclpy.init()
    node = rclpy.create_node("w05_session_observer", parameter_overrides=[
        Parameter("use_sim_time", value=True)])
    maps = []
    poses = []
    initial = []
    subscriptions = [
        node.create_subscription(
            OccupancyGrid, "/map", lambda msg: maps.append(
                [msg.info.width, msg.info.height, msg.info.resolution]),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose",
            lambda msg: poses.append([msg.pose.pose.position.x, msg.pose.pose.position.y]), 10),
        node.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", lambda msg: initial.append(
                [msg.pose.pose.position.x, msg.pose.pose.position.y,
                 msg.header.stamp.sec, msg.header.stamp.nanosec]), 10),
    ]
    processes = []
    logs = []
    owned = set()
    result = {"domain": 41, "goals": [], "ground_truth_goal_errors": None}

    def start(name, command, environment=None):
        log = (output / f"{name}.log").open("w")
        logs.append(log)
        process = subprocess.Popen(
            command, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True, env=environment)
        processes.append(process)
        owned.add(process.pid)
        return process

    def stop(process):
        owned.update(descendants(process.pid))
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()

    def spin_until(predicate, timeout, running):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for process in running:
                owned.update(descendants(process.pid))
                if process.poll() is not None:
                    raise RuntimeError(f"owned process {process.pid} exited {process.returncode}")
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise TimeoutError("runtime condition not observed; inspect phase logs")

    def owners(logfile, expected):
        identities, guid_map = {}, {}
        for line in logfile.read_text().splitlines():
            if line.startswith("TF_OWNER\t"):
                _, parent, child, handle = line.split("\t")
                if parent == "map" and child == "odom":
                    identities[handle] = None
            elif line.startswith("GID_MAP\t"):
                _, handle, guid = line.split("\t")
                guid_map[handle] = guid
        endpoints = {
            bytes(endpoint.endpoint_gid).hex(): endpoint.node_name
            for endpoint in node.get_publishers_info_by_topic("/tf")
        }
        if len(identities) != 1 or not all(handle in guid_map for handle in identities):
            return None
        names = [endpoints.get(guid_map[handle]) for handle in identities]
        return names if names == [expected] else None

    try:
        bridge = subprocess.run(
            ["ros2", "run", "go2_nav2_mujoco_bridge", "twist_to_joystick"],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=15)
        (output / "bridge_domain41_refusal.log").write_text(bridge.stdout)
        assert bridge.returncode != 0 and "requires ROS_DOMAIN_ID=1" in bridge.stdout, bridge.stdout
        result["e2e_blocker"] = {
            "bridge_exit": bridge.returncode,
            "reason": "read-only installed bridge requires ROS_DOMAIN_ID=1; Domain 41 is mandatory",
            "rl_started": False, "goals_sent": 0,
        }
        world = Path(get_package_share_directory("go2_mujoco_lidar")) / "scene/go2_indoor.xml"
        native = os.environ.copy()
        native.pop("LD_LIBRARY_PATH", None)
        simulator = start("simulator", [
            args.simulator, "-s", str(world), "-i", "41", "-n", "lo",
            "--dds-config", str(config),
        ], native)
        base = ["ros2", "launch", "go2_mujoco_lidar", "go2_sim_navigation.launch.py",
                f"simulator_pid:={simulator.pid}"]
        tap = start("mapping_tf", [args.tf_observer])
        mapping = start("mapping", base + ["mode:=mapping"])
        spin_until(lambda: bool(maps) and owners(output / "mapping_tf.log", "slam_toolbox"),
                   40, [simulator, mapping, tap])
        result["mapping"] = {"map_shape": maps[-1], "map_odom_owners": ["slam_toolbox"]}
        save = subprocess.run([
            "ros2", "run", "go2_mujoco_lidar", "sim_save_map",
            "--simulator-pid", str(simulator.pid), "--output", str(output / "saved"),
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=35)
        (output / "save.log").write_text(save.stdout)
        assert save.returncode == 0, save.stdout
        result["saved"] = json.loads((output / "saved.session.json").read_text())
        local_args = ["mode:=localization", f"map:={output / 'saved.yaml'}", "initial_pose:=0,0,0"]
        conflict = subprocess.run(base + local_args, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, text=True, timeout=15)
        (output / "overlap_refusal.log").write_text(conflict.stdout)
        assert conflict.returncode != 0 and "stop it before switching" in conflict.stdout
        result["overlap_refusal_exit"] = conflict.returncode
        stop(mapping)
        stop(tap)
        spin_until(lambda: not any(name in ("slam_toolbox", "go2_sim_session_guard")
                                  for name in node.get_node_names()), 10, [simulator])
        tap = start("localization_tf", [args.tf_observer])
        localization = start("localization", base + local_args)
        spin_until(
            lambda: "NAVIGATION_READY" in (output / "localization.log").read_text()
            and owners(output / "localization_tf.log", "amcl"),
            60, [simulator, localization, tap])
        log = (output / "localization.log").read_text()
        assert log.count("INITIAL_POSE_SENT_ONCE") == 1 and len(initial) == 1, initial
        assert "INITIAL_POSE_ACCEPTED" in log and poses
        states = {}
        for name in ("amcl", "map_server", "planner_server", "controller_server", "behavior_server", "bt_navigator"):
            client = node.create_client(GetState, f"/{name}/get_state")
            assert client.wait_for_service(timeout_sec=3), name
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            assert future.done() and future.result(), name
            states[name] = future.result().current_state.id
            node.destroy_client(client)
        assert all(state == 3 for state in states.values()), states
        loaded = {
            int(line.split("pid=")[1].split()[0]): line.split("library=")[1]
            for line in log.splitlines() if "TF2_LOADED pid=" in line
        }
        assert len(loaded) == 6, loaded
        assert all(paths.endswith("/tf2-pin/tf2/lib/libtf2.so") for paths in loaded.values()), loaded
        result["localization"] = {
            "map_odom_owners": ["amcl"], "initial_pose_messages": initial,
            "amcl_poses": poses, "lifecycle_states": states, "tf2_loaded": loaded,
        }
        result["furthest_verified"] = "mapping -> saved map -> AMCL initial pose accepted -> Nav2 active"
    finally:
        for process in reversed(processes):
            stop(process)
        for log in logs:
            log.close()
        remaining = sorted(pid for pid in owned if Path(f"/proc/{pid}").exists())
        current = existing_stack(project)
        unchanged = all(current.get(pid) == row for pid, row in baseline.items())
        result.update({
            "preexisting": baseline, "preexisting_after": {
                pid: current.get(pid) for pid in baseline},
            "preexisting_unchanged": unchanged, "owned_pids": sorted(owned),
            "owned_remaining": remaining,
        })
        (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
        assert unchanged, "pre-existing process PID/start tick/argv changed"
        assert not remaining, remaining


if __name__ == "__main__":
    main()
