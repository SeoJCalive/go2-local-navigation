"""실제 중력/RL/FIFO 주행, 지도 저장, AMCL 목표와 reset 수용 시험.

실행 전 기존 PID/start tick을 저장하고 자신이 만든 process group만 종료한다.
별도 driver의 qpos를 ROS odometry와 독립적인 base ground truth로 사용한다.
런타임 산출물은 지정한 새 output에 보존하며 기본 install에는 쓰지 않는다.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import traceback

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from go2_inputs.simulation_epoch import watch_simulation_epoch
from lifecycle_msgs.msg import State
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PlannedPath
from rcl_interfaces.srv import GetParameters, ListParameters
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

from owned_processes import OwnedProcesses, existing_stack, snapshot
from ament_index_python.packages import get_package_prefix
from saved_map import (COMMAND_TOPICS, GOALS, WORLD_MAP, amcl_errors,
                       command_ownership, load_csv, map_pose, physics_summary)
from narrow_passage import (GOAL_SPECS, SafetyReader, negative_outcome,
                            passage_coverage, recoveries, verdict_physics)

PROJECT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
SIM_DEPS = Path(os.environ.get(
    "SIM_DEPS", str(Path.home() / ".local/share/go2-local-navigation-sim")))
CONTROLLER_DIR = SIM_DEPS / "acceptance-build/rlroot/deploy/robots/go2/build"


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--goals", type=Path, default=HERE / "goals.json")
    parser.add_argument("--route", type=Path, default=HERE / "route.json")
    parser.add_argument("--saved-map", type=Path,
                        help="G03: skip mapping/reset; fixed H3 transform and three world goals")
    parser.add_argument("--narrow-passage", action="store_true",
                        help="G04 preregistered positive/negative/boundary sequence")
    parser.add_argument("--run-index", type=int, choices=(1, 2, 3), default=1)
    parser.add_argument("--simulator", type=Path,
                        default=SIM_DEPS / "acceptance-build/acceptance_simulator")
    parser.add_argument("--controller", type=Path, default=CONTROLLER_DIR / "go2_ctrl")
    parser.add_argument("--controller-cwd", type=Path, default=CONTROLLER_DIR)
    parser.add_argument("--world", type=Path,
                        default=PROJECT.parent / "go2_nav2_mujoco_bridge/worlds/go2_complex_indoor.xml")
    parser.add_argument("--tf-observer", type=Path,
                        default=Path(get_package_prefix("go2_mujoco_lidar"))
                        / "lib/go2_mujoco_lidar/tf_owner_observer")
    parser.add_argument("--tf2-prefix", type=Path,
                        default=PROJECT / "third_party/tf2/install")
    parser.add_argument("--dds-config", type=Path, default=HERE / "cyclonedds.xml")
    args = parser.parse_args()
    if args.narrow_passage and not args.saved_map:
        parser.error("--narrow-passage requires --saved-map")
    for executable_path in (args.simulator, args.controller, args.tf_observer):
        if not executable_path.is_file():
            parser.error(f"missing executable: {executable_path}; "
                         "run BUILD_ACCEPTANCE=1 bash third_party/unitree_mujoco/setup.sh "
                         "and colcon build --packages-select go2_mujoco_lidar")
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    config = args.dds_config.resolve()
    if args.domain == 0 or args.domain == 42:
        raise ValueError("Use an isolated nonzero domain; Domain 42 is reserved")
    if (os.environ.get("ROS_DOMAIN_ID") != str(args.domain)
            or os.environ.get("ROS_LOCALHOST_ONLY") != "1"
            or os.environ.get("CYCLONEDDS_URI") != config.as_uri()):
        raise ValueError("domain/localhost/DDS environment does not match arguments")
    goals = ([s["world_goal"][:2] for s in GOAL_SPECS] if args.narrow_passage else
             GOALS if args.saved_map else json.loads(args.goals.read_text()))
    route_points = json.loads(args.route.read_text())
    for points in (goals, route_points):
        if not points or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError("goals and route must contain finite [x,y] pairs")
    baseline = existing_stack(PROJECT)
    (output / "preexisting.json").write_text(json.dumps(baseline, indent=2))
    result = {"domain": args.domain, "goals": [], "preexisting": baseline, "criteria": {}}
    progress_path = output.parent / "live/progress.log"

    def progress(message):
        if args.saved_map:
            with progress_path.open("a") as file:
                file.write(("G04/" if args.narrow_passage else "G03/") + message + "\n")

    if args.saved_map:
        progress_path.parent.mkdir(parents=True, exist_ok=True)
        result["saved_map"] = str(args.saved_map.resolve())
        result["world_map_transform"] = WORLD_MAP
        result["mode"] = "saved-map-headless"
        result["criteria_version"] = 4 if args.narrow_passage else 3
        result["run_index"] = args.run_index
        progress(f"run {args.run_index}/3 시작" if args.narrow_passage else
                 "시작 v3 headless 저장 지도 Nav2 단일 실행")
    result["paths"] = {
        name: str(getattr(args, name).resolve()) for name in (
            "simulator", "controller", "controller_cwd", "world", "tf_observer",
            "tf2_prefix", "goals", "route", "dds_config")}
    result["observer_tf2"] = sorted({
        line.split()[-1] for line in Path("/proc/self/maps").read_text().splitlines()
        if "libtf2.so" in line})
    logs = []
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupted(signum, _frame):
        raise RuntimeError(f"acceptance interrupted by signal {signum}")
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    node = rclpy.create_node("go2_sim_acceptance", parameter_overrides=[
        Parameter("use_sim_time", value=True)])
    buffer = Buffer()
    # Humble's Python Buffer does not attach a clock jump callback itself.
    node._epoch_reset = watch_simulation_epoch(node, buffer.clear)
    listener = TransformListener(buffer, node)
    samples = {"cloud": [], "odom": [], "amcl": [], "maps": []}
    commands = []
    costmaps = {}
    planned_paths = (output / "planned_paths.jsonl").open("w")
    amcl_poses = []
    owner_samples = []
    observe_navigation = False
    last_owner_sample = 0.
    current_spec = {}
    safety = SafetyReader(output)
    check_safety = args.narrow_passage

    def record_path(message):
        planned_paths.write(json.dumps({
            "stamp": [message.header.stamp.sec, message.header.stamp.nanosec],
            "received_steady_ns": time.monotonic_ns(),
            "goal_id": current_spec.get("id"), "kind": current_spec.get("kind"),
            "frame": message.header.frame_id,
            "poses": [[p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)]
                      for p in message.poses]}) + "\n")
        planned_paths.flush()

    def record_amcl(message):
        samples["amcl"].append(message)
        p = message.pose.pose
        amcl_poses.append([message.header.stamp.sec + message.header.stamp.nanosec/1e9,
                           p.position.x, p.position.y, yaw(p.orientation)])

    subscriptions = [
        node.create_subscription(PointCloud2, "/utlidar/cloud", lambda m: samples["cloud"].append(
            [m.header.stamp.sec, m.header.stamp.nanosec]), qos_profile_sensor_data),
        node.create_subscription(Odometry, "/odom", lambda m: samples["odom"].append(
            [m.header.stamp.sec, m.header.stamp.nanosec, m.pose.pose.position.x,
             m.pose.pose.position.y]), 20),
        node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 record_amcl, 10),
        node.create_subscription(OccupancyGrid, "/map",
                                 lambda m: samples["maps"].append(m),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(Twist, "/go2_nav2/shadow_cmd_vel", lambda m: commands.append(
            [node.get_clock().now().nanoseconds, m.linear.x, m.angular.z,
             m.linear.y, m.linear.z, m.angular.x, m.angular.y]), 100),
        node.create_subscription(PlannedPath, "/plan", record_path, 20),
    ]
    for name in ("global_costmap", "local_costmap"):
        subscriptions.append(node.create_subscription(
            OccupancyGrid, f"/{name}/costmap",
            lambda m, key=name: costmaps.update({key: m}),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))

    def checkpoint():
        (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")

    def pump():
        # Drain ready callbacks for a bounded executor quantum before the next
        # /proc audit. One callback per full process scan starves the TF reader.
        end = time.monotonic() + 0.02
        rclpy.spin_once(node, timeout_sec=0.02)
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0)
        observe_owners()

    def observe_owners():
        nonlocal last_owner_sample
        if not observe_navigation or time.monotonic() - last_owner_sample < .5:
            return
        last_owner_sample = time.monotonic()
        owners = [{"node": e.node_namespace.rstrip("/") + "/" + e.node_name,
                   "gid": bytes(e.endpoint_gid).hex(), "topic": topic}
                  for topic in COMMAND_TOPICS for e in node.get_publishers_info_by_topic(topic)]
        owner_samples.append({
            "sim_time": node.get_clock().now().nanoseconds/1e9,
            "cmd_vel": owners, "map_odom_amcl": edge_owner(output / "localization_tf.log", "amcl")})

    manager = OwnedProcesses(pump)
    processes = manager.processes

    def executable(package, name):
        return str(Path(get_package_prefix(package)) / "lib" / package / name)

    def start(name, command, env=None, cwd=None):
        log = (output / f"{name}.log").open("w")
        logs.append(log)
        return manager.start(command, stdout=log, stderr=subprocess.STDOUT, env=env, cwd=cwd)

    def stop(process):
        return manager.stop(process)

    def safety_check():
        if check_safety:
            alert = safety.poll()
            if alert:
                result["safety_alert"] = {"message": alert,
                                          "boundary": current_spec.get("kind") == "boundary"}
                progress("경보 " + alert)
                raise RuntimeError("G04 safety stop: " + alert)

    def until(predicate, timeout, running=()):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for p in running:
                if p.poll() is not None:
                    raise RuntimeError(f"owned process {p.pid} exited {p.returncode}")
            rclpy.spin_once(node, timeout_sec=0.05)
            observe_owners()
            safety_check()
            if predicate():
                return
        raise TimeoutError("condition timed out; inspect phase logs")

    def advance(seconds, running=()):
        # Joystick input pulse/stand trajectory duration is simulation time,
        # not an assumed wall delay for asynchronous readiness.
        end = truth()["sim_time"] + seconds
        until(lambda: truth()["sim_time"] >= end, seconds + 5, running)

    def text(name):
        return (output / f"{name}.log").read_text()

    def truth():
        with (output / "truth.csv").open("rb") as file:
            header = file.readline().decode().strip().split(",")
            file.seek(0, os.SEEK_END)
            file.seek(max(0, file.tell()-4096))
            tail = file.read().decode()
            lines = tail.splitlines()
            if not tail.endswith("\n"):
                lines = lines[:-1]
        # Native flushes complete rows. Ignore a partially observed final row.
        rows = [line.split(",") for line in lines if len(line.split(",")) == len(header)]
        return {key: float(value) for key, value in zip(header, rows[-1])}

    def edge_owner(logfile, expected):
        identities, guids = set(), {}
        for line in logfile.read_text().splitlines():
            fields = line.split("\t")
            if fields[0] == "TF_OWNER" and fields[1:3] == ["map", "odom"]:
                identities.add(fields[3])
            if fields[0] == "GID_MAP":
                guids[fields[1]] = fields[2]
        endpoints = {bytes(e.endpoint_gid).hex(): e.node_name
                     for e in node.get_publishers_info_by_topic("/tf")}
        return (len(identities) == 1 and
                all(endpoints.get(guids.get(handle)) == expected for handle in identities))

    fifo = output / "joystick.fifo"
    os.mkfifo(fifo)
    writer = os.open(fifo, os.O_RDWR | os.O_NONBLOCK)

    def event(kind, number, value):
        os.write(writer, struct.pack("<IhBB", int(time.monotonic()*1000) & 0xffffffff,
                                     value, kind, number))

    try:
        native = os.environ.copy()
        native.pop("LD_LIBRARY_PATH", None)
        world = args.world.resolve()
        sim_command = [str(args.simulator.resolve()), "-s", str(world),
                                  "-i", str(args.domain), "-n", "lo", "--dds-config", str(config),
                                  "--fifo", str(fifo), "--output", str(output)]
        if args.saved_map:
            sim_command += ["--progress-log", str(progress_path)]
        sim = start("simulator", sim_command, native)
        ctrl_dir = args.controller_cwd.resolve()
        ctrl = start("controller", ["stdbuf", "-oL", "-eL", str(args.controller.resolve()), "-n", "lo", "-d", str(args.domain)],
                     native, ctrl_dir)
        until(lambda: "FSM: Start Passive" in text("controller"), 20, [sim, ctrl])
        event(2, 5, 32767)
        # Switch LT and receiver LT are smoothed axes. Establish LT.pressed
        # before the single A.on_pressed edge used by the configured transition.
        advance(0.25, [sim, ctrl])
        event(1, 0, 1)
        advance(0.3, [sim, ctrl])
        event(1, 0, 0)
        event(2, 5, 0)
        until(lambda: "to FixStand" in text("controller"), 10, [sim, ctrl])
        advance(3, [sim, ctrl])
        event(1, 11, 1)
        event(2, 7, 32767)
        advance(0.3, [sim, ctrl])
        event(1, 11, 0)
        event(2, 7, 0)
        until(lambda: "to Velocity_Down" in text("controller"), 10, [sim, ctrl])
        result["walking_mode"] = True
        print("WALKING_MODE_READY", flush=True)
        if args.saved_map:
            until(lambda: (output / "standup.json").exists(), 15, [sim, ctrl])
            result["standup"] = json.loads((output / "standup.json").read_text())
            if not result["standup"]["completed"] or result["standup"]["timed_out"]:
                raise RuntimeError("G03 v2 stand-up did not complete within 10 sim seconds")
        base = ["ros2", "launch", "go2_mujoco_lidar", "go2_sim_navigation.launch.py",
                f"simulator_pid:={sim.pid}"]
        if args.tf2_prefix.resolve() != PROJECT / "third_party/tf2/install":
            base.append(f"tf2_prefix:={args.tf2_prefix.resolve()}")
        if not args.saved_map:
            tap = start("mapping_tf", [str(args.tf_observer.resolve())])
            mapping = start("mapping", base + ["mode:=mapping"])
            until(lambda: samples["maps"] and edge_owner(output / "mapping_tf.log", "slam_toolbox"),
                  45, [sim, ctrl, mapping, tap])
        bridge = start("bridge", [executable("go2_nav2_mujoco_bridge", "twist_to_joystick"),
                                  "--ros-args", "-p", f"fifo_path:={fifo}"])
        until(lambda: "simulation bridge ready" in text("bridge"), 10, [bridge])
        if args.saved_map:
            selected_map = str(args.saved_map.resolve())
            p = truth()
            angle = math.atan2(2*(p["qw"]*p["qz"]+p["qx"]*p["qy"]),
                               1-2*(p["qy"]**2+p["qz"]**2))
            map_base = map_pose(p["x"], p["y"], angle)
            a, tx, ty = WORLD_MAP
            map_odom = [tx, ty, a]
            result["initial_truth"] = p
        else:
            selected_map, map_base, map_odom = map_route(
                output, start, stop, until, advance, text, truth, buffer, node,
                samples, base, sim, ctrl, bridge, mapping, tap, result, checkpoint,
                route_points, executable)
        tap_command = [str(args.tf_observer.resolve())]
        if args.saved_map:
            tap_command += ["--ros-args", "-p", "observe_commands:=true", "-p", "use_sim_time:=true"]
        tap = start("localization_tf", tap_command)
        localization = start("localization", base + [
            "mode:=localization", f"map:={selected_map}",
            "initial_pose:="+",".join(map(str, map_base))])
        result["localization_initial_pose"] = map_base
        until(lambda: "NAVIGATION_READY" in text("localization") and
              edge_owner(output / "localization_tf.log", "amcl"), 100,
              [sim, ctrl, bridge, localization, tap])
        result["localization_owner"] = "amcl"
        result["tf2_loaded"] = [line for line in text("localization").splitlines()
                                if "TF2_LOADED" in line]
        print("LOCALIZATION_READY", flush=True)
        result["navigation_start_ns"] = node.get_clock().now().nanoseconds
        observe_navigation = bool(args.saved_map)
        observe_owners()
        if args.narrow_passage:
            # Query all actual parameters, not only source YAML/default guesses.
            dumps = {}
            for target in ("/global_costmap/global_costmap", "/local_costmap/local_costmap",
                           "/planner_server"):
                list_client = node.create_client(ListParameters, target + "/list_parameters")
                get_client = node.create_client(GetParameters, target + "/get_parameters")
                until(lambda: list_client.service_is_ready() and get_client.service_is_ready(), 10)
                request = ListParameters.Request()
                listed = list_client.call_async(request)
                until(listed.done, 10)
                names = listed.result().result.names
                request = GetParameters.Request()
                request.names = names
                queried = get_client.call_async(request)
                until(queried.done, 10)
                fields = {1: "bool_value", 2: "integer_value", 3: "double_value",
                          4: "string_value", 5: "byte_array_value", 6: "bool_array_value",
                          7: "integer_array_value", 8: "double_array_value", 9: "string_array_value"}
                dumps[target] = {name: getattr(value, fields[value.type]) if value.type in fields else None
                                 for name, value in zip(names, queried.result().values)}
                dumps[target] = {k: list(v) if not isinstance(v, (str, bool, int, float, type(None)))
                                 else v for k, v in dumps[target].items()}
                node.destroy_client(list_client)
                node.destroy_client(get_client)
            (output / "parameters.json").write_text(json.dumps(dumps, indent=2) + "\n")
            result["parameters_match"] = all(
                isinstance(dumps[target].get(key), (float, int)) and
                math.isclose(dumps[target][key], value, rel_tol=0, abs_tol=1e-8)
                for target in ("/global_costmap/global_costmap", "/local_costmap/local_costmap")
                for key, value in (("robot_radius", .42), ("resolution", .05),
                                   ("footprint_padding", .01),
                                   ("inflation_layer.inflation_radius", .65)))
            result["parameters_match"] = result["parameters_match"] and all(
                dumps["/planner_server"].get(key) == value for key, value in (
                    ("GridBased.use_astar", False), ("GridBased.allow_unknown", False),
                    ("GridBased.tolerance", .1)))
            if not result["parameters_match"]:
                raise RuntimeError("G04 actual parameter/model mismatch")
        action = ActionClient(node, NavigateToPose, "/navigate_to_pose")
        if not action.wait_for_server(timeout_sec=10):
            raise TimeoutError("NavigateToPose server missing")
        dx, dy, angle = map_odom
        ca, sa = math.cos(angle), math.sin(angle)
        for index, (x,y) in enumerate(goals, 1):
            current_spec = GOAL_SPECS[index-1] if args.narrow_passage else {}
            if current_spec.get("kind") == "boundary" and "boundary_start_sim_time" not in result:
                result["boundary_start_sim_time"] = truth()["sim_time"]
                result["boundary_start_steady_ns"] = time.monotonic_ns()
            safety.minimum = math.inf
            gx, gy = dx+ca*x-sa*y, dy+sa*x+ca*y
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
            goal_angle = angle + (current_spec["world_goal"][2] if current_spec else 0)
            goal.pose.pose.orientation.z = math.sin(goal_angle/2)
            goal.pose.pose.orientation.w = math.cos(goal_angle/2)
            started = time.monotonic()
            start_sim_time = truth()["sim_time"]
            start_steady_ns = time.monotonic_ns()
            future = action.send_goal_async(goal)
            until(future.done, 10, [sim, ctrl, bridge, localization])
            handle = future.result()
            row = {"index":index, "world_goal":[x,y], "map_goal":[gx,gy], "accepted":handle.accepted}
            row.update(current_spec)
            row["start_steady_ns"] = start_steady_ns
            row["goal_uuid"] = bytes(handle.goal_id.uuid).hex()
            if handle.accepted:
                response = handle.get_result_async()
                try:
                    until(response.done, 150, [sim, ctrl, bridge, localization])
                    row["status"] = response.result().status
                    row["action_result"] = str(response.result().result)
                except TimeoutError:
                    cancellation = handle.cancel_goal_async()
                    until(cancellation.done, 10)
                    if args.narrow_passage:
                        until(response.done, 10, [sim, ctrl, bridge, localization])
                        row["cancel_terminal_status"] = response.result().status
                    row["status"] = "timeout"
            else:
                row["status"] = "rejected"
            row["wall_sec"] = time.monotonic()-started
            row["end_steady_ns"] = time.monotonic_ns()
            row["status_name"] = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}.get(
                row["status"], str(row["status"]))
            row["truth"] = truth()
            row["start_sim_time"] = start_sim_time
            row["position_error_m"] = math.hypot(row["truth"]["x"]-x, row["truth"]["y"]-y)
            if samples["amcl"]:
                pose = samples["amcl"][-1]
                row["amcl_sample"] = [pose.header.stamp.sec, pose.header.stamp.nanosec,
                                      pose.pose.pose.position.x, pose.pose.pose.position.y,
                                      yaw(pose.pose.pose.orientation)]
            row["costmaps"] = {key: [value.info.width, value.info.height,
                                     value.info.resolution] for key,value in costmaps.items()}
            if args.narrow_passage:
                safety_check()
                row["live_min_clearance_m"] = safety.minimum if math.isfinite(safety.minimum) else None
                if row["kind"] == "negative":
                    grid = costmaps.get("global_costmap")
                    if grid is None:
                        row["tolerance_allowed_cells"] = None
                    else:
                        res = grid.info.resolution
                        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
                        row["tolerance_allowed_cells"] = sum(
                            1 for j in range(grid.info.height) for i in range(grid.info.width)
                            if abs(ox+(i+.5)*res-gx) <= .1 and abs(oy+(j+.5)*res-gy) <= .1
                            and 0 <= grid.data[j*grid.info.width+i] <= 98)
            if row["status"] != 4 or row["position_error_m"] > .3:
                for key,value in costmaps.items():
                    (output / f"goal{index}_{key}.json").write_text(json.dumps({
                        "width":value.info.width, "height":value.info.height,
                        "resolution":value.info.resolution,
                        "origin":[value.info.origin.position.x, value.info.origin.position.y],
                        "data":list(value.data)}))
            result["goals"].append(row)
            checkpoint()
            print("GOAL_RESULT "+json.dumps(row), flush=True)
            progress(f"목표 {index} {'도달' if row['status'] == 4 else '실패'} "
                     f"{row.get('id', '')} {row['status_name']} "
                     f"시간={row['wall_sec']:.3f}s 오차={row['position_error_m']:.6f}m "
                     f"최소여유={row.get('live_min_clearance_m')}")
        result["navigation_end_ns"] = node.get_clock().now().nanoseconds
        if not args.saved_map:
            before = {key:len(samples[key]) for key in ("cloud","odom")}
            before_pids = {p.pid: sorted(pid for pid, row in snapshot().items()
                                       if row["session"] == p.pid)
                           for p in (localization, bridge)}
            before_time = samples["odom"][-1][0]
            before_cloud_stamp = samples["cloud"][-1]
            (output / "reset.request").touch()
            until(lambda: truth()["epoch"] == 1, 5, [sim, ctrl, bridge, localization])
            until(lambda: all(len(samples[key]) > before[key]+3 for key in ("cloud","odom"))
                  and samples["odom"][-1][0] < before_time
                  and samples["cloud"][-1] < before_cloud_stamp,
                  15, [sim, ctrl, bridge, localization])
            result["reset"] = {
                "cloud_before": before["cloud"], "cloud_after": len(samples["cloud"]),
                "odom_before": before["odom"], "odom_after": len(samples["odom"]),
                "odom_stamp_before_sec": before_time, "odom_stamp_after": samples["odom"][-1][:2],
                "cloud_stamp_before": before_cloud_stamp,
                "cloud_stamp_after": samples["cloud"][-1],
                "same_pids": before_pids == {
                    p.pid: sorted(pid for pid, row in snapshot().items()
                                  if row["session"] == p.pid)
                    for p in (localization, bridge)},
            }
            print("RESET_RECOVERED", flush=True)
    except Exception:
        if args.narrow_passage and current_spec:
            result["interrupted_goal"] = {
                **current_spec, "wall_sec": time.monotonic()-started,
                "truth": truth(), "live_min_clearance_m": safety.minimum}
            progress(f"목표 {current_spec['id']} 중단 시간={time.monotonic()-started:.3f}s "
                     f"최소여유={safety.minimum}")
        if args.narrow_passage and result.get("safety_alert", {}).get("boundary"):
            result["boundary_failure"] = traceback.format_exc()
        else:
            result["failure"] = traceback.format_exc()
        print(traceback.format_exc(), flush=True)
    finally:
        check_safety = False
        observe_navigation = False
        cleanup_errors = []
        for p in reversed(processes):
            try:
                stop(p)
            except Exception as error:
                cleanup_errors.append(str(error))
        result["cleanup_errors"] = cleanup_errors
        result["shutdowns"] = manager.stops
        for log in logs:
            log.close()
        os.close(writer)
        fifo.unlink()
        current = existing_stack(PROJECT)
        result["preexisting_after"] = {pid: current.get(pid) for pid in baseline}
        result["preexisting_unchanged"] = all(current.get(pid) == row for pid,row in baseline.items())
        result["owned_pids"] = sorted(manager.identities)
        result["owned_remaining"] = manager.remaining()
        with (output / "commands.csv").open("w") as file:
            writer_csv = csv.writer(file)
            writer_csv.writerow(["stamp_ns", "vx", "wz", "vy", "vz", "wx", "wy"])
            writer_csv.writerows(commands)
        contact_file = output / "contacts.csv"
        result["nonfloor_contacts"] = (
            sum(1 for _ in csv.DictReader(contact_file.open()))
            if contact_file.exists() else None)
        result["sigkill_logs"] = [
            path.name for path in output.glob("*.log")
            if "escalating to 'SIGKILL'" in path.read_text(errors="replace")]
        result["sigterm_logs"] = [
            path.name for path in output.glob("*.log")
            if "escalating to 'SIGTERM'" in path.read_text(errors="replace")]
        localization_log = output / "localization.log"
        shutdown_log = localization_log.read_text(errors="replace") if localization_log.exists() else ""
        completed = [
            line.split("MANAGED_SHUTDOWN_COMPLETE ", 1)[1]
            for line in shutdown_log.splitlines() if "MANAGED_SHUTDOWN_COMPLETE " in line]
        result["shutdown_states"] = json.JSONDecoder().raw_decode(completed[-1])[0] if completed else {}
        result["shutdown_errors"] = [
            line for line in shutdown_log.splitlines() if "MANAGED_SHUTDOWN_FAILED" in line]
        result["samples"] = {key:len(value) for key,value in samples.items()}
        planned_paths.close()
        result["criteria"] = {
            "mapping": bool(result.get("route_completed")),
            "mapping_no_stalls": result.get("mapping_stalls") == 0,
            "zero_nonfloor_contacts": result["nonfloor_contacts"] == 0,
            "save": bool(result.get("saved")),
            "goals_succeeded": len(result["goals"]) == len(goals) and all(
                row["status"] == 4 for row in result["goals"]),
            "position_error": len(result["goals"]) == len(goals) and all(
                row["position_error_m"] <= 0.3 for row in result["goals"]),
            "reset_recovery": bool(result.get("reset", {}).get("same_pids")),
            "zero_residue": not result["owned_remaining"] and not cleanup_errors,
            "no_sigkill": not result["sigkill_logs"] and not any(
                int(signal.SIGKILL) in stop["signals"] for stop in manager.stops),
            "no_escalation": not result["sigterm_logs"] and not result["sigkill_logs"] and all(
                set(stop["signals"]) <= {int(signal.SIGINT)} for stop in manager.stops),
            "managed_shutdown": not result["shutdown_errors"] and set(result["shutdown_states"]) == {
                "planner_server", "controller_server", "behavior_server", "bt_navigator",
                "map_server", "amcl",
            } and all(value == State.PRIMARY_STATE_FINALIZED
                      for value in result["shutdown_states"].values()),
            "preexisting_unchanged": result["preexisting_unchanged"],
        }
        if args.saved_map:
            physics = load_csv(output / "physics.csv") if (output / "physics.csv").exists() else []
            result["physics"] = physics_summary(physics)
            result["command_ownership"] = command_ownership(
                (output / "localization_tf.log").read_text() if (
                    output / "localization_tf.log").exists() else "",
                owner_samples, result.get("navigation_start_ns", 0),
                result.get("navigation_end_ns", node.get_clock().now().nanoseconds))
            command_rows = result["command_ownership"]["messages"]
            with (output / "command_owners.csv").open("w") as file:
                writer_csv = csv.DictWriter(file, fieldnames=[
                    "received_steady_ns", "sim_stamp_ns", "topic", "rmw_gid", "publisher_gid",
                    "node", "active_actions", "action_role_matches",
                    "vx", "wz", "vy", "vz", "wx", "wy"])
                writer_csv.writeheader()
                writer_csv.writerows(command_rows)
            errors = amcl_errors(amcl_poses, load_csv(output / "truth.csv")) if (
                output / "truth.csv").exists() else []
            with (output / "amcl_gt.csv").open("w") as file:
                writer_csv = csv.DictWriter(file, fieldnames=[
                    "sim_time", "x", "y", "yaw", "xy_error_m"])
                writer_csv.writeheader()
                writer_csv.writerows(errors)
            result["owner_samples"] = owner_samples
            for row in result["goals"]:
                end = row["truth"]["sim_time"]
                row["physics"] = physics_summary(physics, row["start_sim_time"], end)
                matches = [p for p in errors if row["start_sim_time"] <= p["sim_time"] <= end
                           and p["xy_error_m"] is not None]
                row["amcl_error_m"] = matches[-1]["xy_error_m"] if matches else None
            for key in ("mapping", "mapping_no_stalls", "save", "reset_recovery"):
                del result["criteria"][key]
            result["criteria"].update({
                "goal_time": len(result["goals"]) == 3 and all(
                    row["wall_sec"] <= 150 for row in result["goals"]),
                "zero_falls": bool(result.get("standup", {}).get("completed"))
                              and not result["standup"]["timed_out"]
                              and result["physics"]["falls"] == 0,
                "clearance": result["physics"]["min_clearance_m"] is not None
                             and result["physics"]["min_clearance_m"] >= .10,
                "single_cmd_vel_owner": result["command_ownership"]["passed"],
                "single_map_odom_owner": bool(owner_samples) and all(
                    s["map_odom_amcl"] for s in owner_samples),
                "telemetry": bool(errors) and any(
                    e["xy_error_m"] is not None for e in errors)
                    and bool(commands) and (output / "planned_paths.jsonl").stat().st_size > 0,
            })
            if args.narrow_passage:
                boundary = result.get("boundary_start_sim_time", math.inf)
                result["all_physics"] = result["physics"]
                result["physics"] = verdict_physics(physics, boundary)
                truth_rows = load_csv(output / "truth.csv") if (output / "truth.csv").exists() else []
                paths = [json.loads(line) for line in (
                    output / "planned_paths.jsonl").read_text().splitlines()]
                for row in result["goals"]:
                    row["planned_path_count"] = sum(
                        bool(p["poses"]) for p in paths if p["goal_id"] == row["id"])
                    if row["kind"] == "negative":
                        points = [[float(r["x"]), float(r["y"])] for r in truth_rows
                                  if row["start_sim_time"] <= float(r["sim_time"])
                                  <= row["truth"]["sim_time"]]
                        row["negative_checks"] = negative_outcome(row, points)
                positives = [r for r in result["goals"] if r["kind"] == "positive"]
                negatives = [r for r in result["goals"] if r["kind"] == "negative"]
                main_truth = [r for r in truth_rows if
                              result.get("navigation_start_ns", 0)/1e9 <= float(r["sim_time"]) < boundary]
                result["passage_coverage"] = passage_coverage(main_truth, paths)
                native_log = (output / "localization_tf.log").read_text() if (
                    output / "localization_tf.log").exists() else ""
                result["recoveries"] = recoveries(native_log, result["command_ownership"])
                for recovery in result["recoveries"]:
                    progress("회복 " + json.dumps(recovery, ensure_ascii=False))
                main_recoveries = [r for r in result["recoveries"] if
                                   r["start_ns"] < result.get("boundary_start_steady_ns", math.inf)]
                result["main_nonfloor_contacts"] = sum(
                    float(r["sim_time"]) < boundary for r in load_csv(output / "contacts.csv")) if (
                        output / "contacts.csv").exists() else None
                result["criteria"].update({
                    "goals_succeeded": len(positives) == 34 and all(r["status"] == 4 for r in positives),
                    "position_error": len(positives) == 34 and all(
                        r["position_error_m"] <= .3 for r in positives),
                    "goal_time": len(positives) == 34 and all(r["wall_sec"] <= 150 for r in positives),
                    "negative_goals": len(negatives) == 4 and all(
                        all(r["negative_checks"].values()) for r in negatives),
                    "passage_coverage": result["passage_coverage"]["passed"],
                    "recoveries_terminated": all(r["terminal"] in (4, 5, 6) for r in main_recoveries),
                    "zero_nonfloor_contacts": result["main_nonfloor_contacts"] == 0,
                    "zero_falls": bool(result.get("standup", {}).get("completed"))
                                  and not result["standup"]["timed_out"]
                                  and result["physics"]["falls"] == 0,
                    "clearance": result["physics"]["min_clearance_m"] is not None
                                 and result["physics"]["min_clearance_m"] >= .10,
                    "parameters": result.get("parameters_match", False),
                })
        result["passed"] = not result.get("failure") and all(result["criteria"].values())
        checkpoint()
        progress("종료 정리 판정=" + ("PASS" if result["passed"] else "FAIL")
                 + f" 잔류={len(result['owned_remaining'])}")
        if args.narrow_passage:
            progress(f"run {args.run_index}/3 끝 " + ("PASS" if result["passed"] else "FAIL"))
        node.destroy_node()
        rclpy.shutdown()
        print("ACCEPTANCE_FINISHED "+str(output / "result.json"), flush=True)
    if not result["passed"]:
        raise RuntimeError("Acceptance failed; inspect result.json")


def map_route(output, start, stop, until, advance, text, truth, buffer, node,
              samples, base, sim, ctrl, bridge, mapping, tap, result, checkpoint, route, executable):
        """이미 통과한 매핑 단계를 저장 지도 재시험과 분리한다."""
        runner = start("route", [executable("go2_nav2_mujoco_bridge", "waypoint_runner"),
            "--ros-args", "-p", "waypoints_x:="+str([float(x) for x,y in route]),
            "-p", "waypoints_y:="+str([float(y) for x,y in route]),
            "-p", "timeout:=600.0", "-p", "stall_timeout:=10.0"])
        started = time.monotonic()
        result["mapping_start_sim_time"] = truth()["sim_time"]
        result["route"] = route
        print("MAPPING_ROUTE_STARTED", flush=True)
        until(lambda: any(s in text("route") for s in
                          ("status=completed", "status=timeout", "status=stalled")),
              620, [sim, ctrl, mapping, bridge, runner])
        route_log = text("route")
        result["route_completed"] = "status=completed" in route_log
        result["mapping_wall_sec"] = time.monotonic() - started
        result["mapping_end_sim_time"] = truth()["sim_time"]
        result["mapping_stalls"] = route_log.count("status=stalled")
        result["mapping_contacts"] = sum(
            1 for _ in csv.DictReader((output / "contacts.csv").open()))
        stop(runner)
        checkpoint()
        if not result["route_completed"]:
            raise RuntimeError("mapping route did not complete")
        last_stamp = samples["odom"][-1][:2]
        until(lambda: samples["odom"][-1][:2] > last_stamp,
              10, [sim, ctrl, mapping, bridge])
        stamp = samples["odom"][-1][:2]
        saved_time = rclpy.time.Time(nanoseconds=stamp[0] * 10**9 + stamp[1])
        until(lambda: buffer.can_transform("map", "base", saved_time),
              10, [sim, ctrl, mapping, bridge])
        result["mapping_truth"] = truth()
        result["mapping_map_base"] = None
        t = buffer.lookup_transform("map", "base", saved_time)
        q = t.transform.rotation
        map_base = [t.transform.translation.x, t.transform.translation.y, yaw(q)]
        result["mapping_map_base"] = map_base
        t = buffer.lookup_transform("map", "odom", saved_time)
        map_odom = [t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation)]
        result["map_odom_transform_at_save"] = map_odom
        result["mapping_owner"] = "slam_toolbox"
        m = samples["maps"][-1]
        result["map_shape"] = [m.info.width, m.info.height, m.info.resolution]
        save = start("save", [executable("go2_mujoco_lidar", "sim_save_map"),
                             "--simulator-pid", str(sim.pid), "--output", str(output / "saved")])
        until(lambda: save.poll() is not None, 35, [sim, ctrl, mapping, bridge])
        if save.returncode != 0:
            raise RuntimeError(text("save"))
        stop(save)
        result["saved"] = json.loads((output / "saved.session.json").read_text())
        checkpoint()
        print("MAPPING_SAVED", flush=True)
        stop(mapping)
        stop(tap)
        until(lambda: "slam_toolbox" not in node.get_node_names(), 10, [sim, ctrl, bridge])
        return str(output / "saved.yaml"), map_base, map_odom


if __name__ == "__main__":
    main()
