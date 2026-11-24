"""W03 installed launch를 실제 MuJoCo DDS로 격리 검증하고 소유 PID만 정리한다.

Domain 41/localhost와 W03 discovery XML을 요구한다. 실제 /map, topic endpoint,
TF edge별 publisher GID, use_sim_time 및 거부 결과를 output에 보존한다. simulator는
무중력 정지 source이며 RL, 지도 정확도 또는 실제 로봇 시험을 대신하지 않는다.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def descendants(parent):
    """현재 parent 하위 PID만 읽는다; 다른 session은 반환하지 않는다."""
    children = {}
    for path in Path("/proc").glob("[0-9]*/stat"):
        try:
            ppid = int(path.read_text().rsplit(")", 1)[1].split()[1])
            children.setdefault(ppid, []).append(int(path.parent.name))
        except (FileNotFoundError, ProcessLookupError):
            continue
    result = set()
    pending = [parent]
    while pending:
        for pid in children.get(pending.pop(), []):
            result.add(pid)
            pending.append(pid)
    return result


def existing_stack():
    """기존 default-install/GUI simulator의 PID·시작 tick을 읽기만 한다."""
    result = {}
    project = Path(__file__).resolve().parents[4]
    for path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            arguments = path.read_bytes().rstrip(b"\0").decode().split("\0")
            if not arguments or not arguments[0]:
                continue
            if (Path(arguments[0]).name == "unitree_mujoco"
                    or any(arg.startswith(str(project / "install") + "/") for arg in arguments)):
                result[path.parent.name] = {
                    "start_ticks": (path.parent / "stat").read_text().rsplit(")", 1)[1].split()[19],
                    "argv": arguments,
                }
        except (FileNotFoundError, ProcessLookupError):
            continue
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulator", required=True)
    parser.add_argument("--tf-observer", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "41" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("W03 verification requires Domain 41 and localhost only")
    config = Path(__file__).with_name("cyclonedds_w03.xml").resolve()
    if os.environ.get("CYCLONEDDS_URI") != config.as_uri():
        raise RuntimeError("W03 dedicated discovery config required")
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    lidar_share = Path(get_package_share_directory("go2_mujoco_lidar"))
    world = lidar_share / "scene/go2_indoor.xml"
    urdf = Path(get_package_share_directory("description")) / "urdf/go2_description.urdf"
    expected = {
        f"{joint.find('parent').get('link')}->{joint.find('child').get('link')}"
        for joint in ElementTree.parse(urdf).findall(".//joint[@type='fixed']")
    } | {"map->odom", "odom->base", "base->utlidar_lidar"}
    rclpy.init()
    node = rclpy.create_node(
        "w03_bringup_observer",
        parameter_overrides=[Parameter("use_sim_time", value=True)])
    counts = {"scan": 0, "odom": 0, "map": 0}
    map_shape = None
    owners = {}
    gid_map = {}

    def receive(kind, message):
        nonlocal map_shape
        counts[kind] += 1
        if kind == "map":
            map_shape = [message.info.width, message.info.height, message.info.resolution]

    subscriptions = [
        node.create_subscription(
            LaserScan, "/scan", lambda msg: receive("scan", msg), qos_profile_sensor_data),
        node.create_subscription(Odometry, "/odom", lambda msg: receive("odom", msg), 10),
        node.create_subscription(OccupancyGrid, "/map", lambda msg: receive("map", msg),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    processes = []
    logs = []
    owned = set()
    result = {}
    baseline = existing_stack()

    def start(name, command, environment=None):
        log = (output / f"{name}.log").open("w")
        logs.append(log)
        process = subprocess.Popen(
            command, stdout=log, stderr=subprocess.STDOUT, env=environment,
            start_new_session=True)
        processes.append(process)
        owned.add(process.pid)
        return process

    try:
        tap = start("tf_owners", [
            args.tf_observer, "--ros-args", "-p", "use_sim_time:=true"])
        native_environment = os.environ.copy()
        native_environment.pop("LD_LIBRARY_PATH", None)
        simulator = start("simulator", [
            args.simulator, "-s", str(world), "-i", "41", "-n", "lo",
            "--dds-config", str(config),
        ], native_environment)
        base = ["ros2", "launch", "go2_mujoco_lidar", "go2_sim_bringup.launch.py",
                f"simulator_pid:={simulator.pid}"]
        refusals = {}
        for name, extra, changes in [
            ("domain_zero", [], {"ROS_DOMAIN_ID": "0"}),
            ("network", [], {"ROS_LOCALHOST_ONLY": "0", "CYCLONEDDS_URI": ""}),
            ("world_mismatch", [
                f"model_path:={os.environ['UNITREE_MUJOCO_ROOT']}/unitree_robots/go2/scene_terrain.xml"
            ], {}),
        ]:
            environment = os.environ.copy()
            environment.update(changes)
            refusal = subprocess.run(base + extra, env=environment, text=True,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=15)
            (output / f"refusal_{name}.log").write_text(refusal.stdout)
            assert refusal.returncode != 0, refusal.stdout
            marker = "world mismatch" if name == "world_mismatch" else "simulation refused"
            assert marker in refusal.stdout, refusal.stdout
            refusals[name] = {"exit": refusal.returncode, "message": marker}
        launch = start("bringup", base + [
            "mapping:=true", "horizontal_samples:=720", "vertical_samples:=1",
            "vertical_min:=0.0", "vertical_max:=0.0",
        ])
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            for line in (output / "tf_owners.log").read_text().splitlines():
                if line.startswith("TF_OWNER\t"):
                    _, parent, child, gid = line.split("\t")
                    owners.setdefault(f"{parent}->{child}", set()).add(gid)
                elif line.startswith("GID_MAP\t"):
                    _, handle, guid = line.split("\t")
                    gid_map[handle] = guid
            owned.update(descendants(launch.pid))
            assert tap.poll() is None, "TF owner observer exited"
            assert simulator.poll() is None, "headless simulator exited"
            assert launch.poll() is None, (output / "bringup.log").read_text()
            mapped = all(gid in gid_map for gids in owners.values() for gid in gids)
            if (counts["map"] and counts["scan"] >= 10 and counts["odom"] >= 10
                    and expected <= owners.keys() and mapped):
                break
        assert counts["map"] and counts["scan"] >= 10 and counts["odom"] >= 10, counts
        assert expected <= owners.keys(), sorted(expected - owners.keys())
        assert set(owners) == expected, sorted(set(owners) - expected)
        assert all(len(gids) == 1 for gids in owners.values()), owners
        endpoints = {}
        for topic in ("/tf", "/tf_static"):
            for endpoint in node.get_publishers_info_by_topic(topic):
                endpoints[bytes(endpoint.endpoint_gid).hex()] = endpoint.node_name
        resolved_owners = {
            edge: endpoints[gid_map[next(iter(gids))]] for edge, gids in sorted(owners.items())
        }
        assert resolved_owners["map->odom"] == "slam_toolbox", resolved_owners
        assert resolved_owners["odom->base"] == "go2_odometry_adapter", resolved_owners
        for edge in expected:
            buffer.lookup_transform(*edge.split("->"), rclpy.time.Time())
        publishers = {
            topic: [endpoint.node_name for endpoint in node.get_publishers_info_by_topic(topic)]
            for topic in ("/clock", "/scan", "/odom", "/map", "/lowstate", "/sportmodestate")
        }
        assert all(len(names) == 1 for names in publishers.values()), publishers
        assert publishers["/clock"] == ["go2_mujoco_lidar"], publishers
        sim_time = {}
        sim_nodes = {
            "go2_mujoco_lidar", "robot_state_publisher", "base_to_utlidar_lidar_static_tf",
            "go2_mapping_cloud_gate", "pointcloud_to_laserscan", "go2_odometry_adapter",
            "slam_toolbox", "w03_tf_owner_observer",
        }
        graph_nodes = node.get_node_names_and_namespaces()
        assert sim_nodes <= {name for name, _ in graph_nodes}, graph_nodes
        # tf2's private transform_listener_impl nodes disable parameter services.
        # The converter constructs their shared TF Buffer with its own sim clock.
        for name in sorted(sim_nodes):
            service = node.create_client(GetParameters, f"/{name}/get_parameters")
            assert service.wait_for_service(timeout_sec=3), name
            future = service.call_async(GetParameters.Request(names=["use_sim_time"]))
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            assert future.done() and future.result(), name
            sim_time[name] = future.result().values[0].bool_value
            node.destroy_client(service)
        assert sim_time and all(sim_time.values()), sim_time
        frames = subprocess.run(
            ["ros2", "run", "tf2_tools", "view_frames", "--ros-args", "-p", "use_sim_time:=true"],
            cwd=output, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=20)
        (output / "view_frames.log").write_text(frames.stdout)
        assert frames.returncode == 0, frames.stdout
        result.update({
            "domain": 41, "discovery_tag": "go2-w03-bringup-20261124",
            "world": str(world.resolve()), "counts": counts, "map_shape": map_shape,
            "topics": node.get_topic_names_and_types(), "publishers": publishers,
            "tf_owners": resolved_owners, "tf_edge_count": len(owners),
            "tf_sample_identities": {edge: sorted(gids) for edge, gids in owners.items()},
            "dds_handle_to_guid": gid_map, "tf_graph_endpoints": endpoints,
            "use_sim_time": sim_time, "refusals": refusals,
            "tf_listener_helpers": [
                name for name, _ in graph_nodes if name.startswith("transform_listener_impl_")
            ],
            "view_frames_exit": frames.returncode,
        })
    finally:
        for process in reversed(processes):
            owned.update(descendants(process.pid))
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for log in logs:
            log.close()
        remaining = sorted(pid for pid in owned if Path(f"/proc/{pid}").exists())
        current = existing_stack()
        unchanged = all(current.get(pid) == row for pid, row in baseline.items())
        result.update({
            "owned_pids": sorted(owned), "owned_pids_remaining": remaining,
            "preexisting_stack": baseline, "preexisting_stack_unchanged": unchanged,
        })
        (output / "result.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
        assert not remaining, remaining
        assert unchanged, "a pre-existing stack process changed"


if __name__ == "__main__":
    main()
