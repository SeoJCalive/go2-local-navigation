"""W02의 실제 MuJoCo/Go2Bridge -> LiDAR -> gate/adapter 경계를 격리 검증한다.

Domain 1/localhost만 허용한다. 별도 Release build의 executable 경로를 받으며
자신이 시작한 process group만 종료한다. raw qpos와 ROS 결과를 output에 남긴다.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulator", required=True)
    parser.add_argument("--lidar", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--dds-config", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "1" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("requires Domain 1 and localhost only")
    if os.environ.get("CYCLONEDDS_URI") != Path(args.dds_config).resolve().as_uri():
        raise RuntimeError("ROS and simulator must share the explicit W02 DDS config")
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    processes = []
    logs = []

    def start(name, command, *, native_sdk=False):
        log = (output / f"{name}.log").open("w")
        logs.append(log)
        environment = os.environ.copy()
        if native_sdk:
            # Keep ROS overlay DDS libraries out of the W01 native SDK driver.
            environment.pop("LD_LIBRARY_PATH", None)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=environment)
        processes.append((name, process))
        return process

    rclpy.init()
    node = rclpy.create_node("w02_runtime_observer")
    observations = {topic: [] for topic in (
        "/utlidar/robot_odom", "/odom", "/utlidar/cloud", "/go2_mapping/cloud_validated")}
    subscriptions = []
    last_stamp = {}
    epoch = {}

    def receive(topic, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if topic in last_stamp and stamp < last_stamp[topic]:
            epoch[topic] = epoch.get(topic, 0) + 1
        last_stamp[topic] = stamp
        row = {"wall": time.monotonic(), "stamp": stamp, "epoch": epoch.get(topic, 0)}
        if isinstance(message, Odometry):
            row["position"] = [getattr(message.pose.pose.position, axis) for axis in "xyz"]
            row["linear"] = [getattr(message.twist.twist.linear, axis) for axis in "xyz"]
        observations[topic].append(row)

    for topic in observations:
        msg_type = Odometry if "odom" in topic else PointCloud2
        subscriptions.append(node.create_subscription(
            msg_type, topic, lambda message, topic=topic: receive(topic, message), 100))
    ros_args = ["--ros-args", "-p", "use_sim_time:=true"]
    stopped_at = None
    try:
        start("gate", [sys.executable, "-c",
                      "from go2_perception.mapping_cloud_gate_node import main; main()"] + ros_args)
        start("adapter", [sys.executable, "-c",
                         "from go2_state_estimation.odometry_adapter_node import main; main()"] + ros_args)
        start("lidar", [args.lidar] + ros_args + [
            "-p", f"model_path:={args.model}", "-p", "horizontal_samples:=64",
            "-p", "vertical_samples:=1", "-p", "vertical_min:=0.0",
            "-p", "vertical_max:=0.0"])
        sim = start("simulator", [args.simulator, args.model, str(output / "truth.csv"),
                                  args.dds_config], native_sdk=True)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if sim.poll() is not None:
                if sim.returncode != 0:
                    raise RuntimeError(f"simulator exited {sim.returncode}")
                if stopped_at is None:
                    stopped_at = time.monotonic()
                if time.monotonic() - stopped_at >= 2:
                    break
        if stopped_at is None:
            raise RuntimeError("simulator did not stop before deadline")
        for name, process in processes[:-1]:
            if process.poll() is not None:
                raise RuntimeError(f"{name} exited early: {process.returncode}")
        truth = {}
        with (output / "truth.csv").open() as stream:
            for row in csv.DictReader(stream):
                truth[(int(row["epoch"]), int(row["stamp_ns"]))] = row
        errors = []
        forward = []
        for row in observations["/utlidar/robot_odom"]:
            reference = truth.get((row["epoch"], row["stamp"]))
            if reference is None:
                raise RuntimeError("odometry stamp has no matching simulator step")
            errors.append(math.dist(row["position"], [float(reference[a]) for a in "xyz"]))
            if float(reference["body_vx"]) > 0 and row["stamp"] > 1.1e9:
                forward.append(row["linear"])
        counts = {topic: [sum(row["epoch"] == e for row in rows) for e in (0, 1)]
                  for topic, rows in observations.items()}
        duplicates = {topic: len(rows) - len({(r["epoch"], r["stamp"]) for r in rows})
                      for topic, rows in observations.items()}
        after_stop = {topic: sum(r["wall"] > stopped_at + 0.5 for r in rows)
                      for topic, rows in observations.items()}
        odom_stamps = {(r["epoch"], r["stamp"]) for r in observations["/utlidar/robot_odom"]}
        cloud_stamps = {(r["epoch"], r["stamp"]) for r in observations["/utlidar/cloud"]}
        result = {
            "discovery_tag": "go2-w02-state-contract-20261124",
            "source_publishers": {
                topic: node.count_publishers(topic) for topic in ("/lowstate", "/sportmodestate")
            },
            "counts_per_epoch": counts, "duplicates": duplicates,
            "after_stop_settled": after_stop,
            "cloud_odom_stamp_mismatches": len(odom_stamps ^ cloud_stamps),
            "position_error_max_m": max(errors, default=None),
            "position_samples": len(errors), "body_forward_samples": len(forward),
            "body_forward_x_range": [min(v[0] for v in forward), max(v[0] for v in forward)]
            if forward else None,
            "body_forward_lateral_max": max((abs(v[1]) for v in forward), default=None),
            "owned_pids": [p.pid for _, p in processes],
        }
        (output / "observations.json").write_text(json.dumps(observations, indent=2))
        (output / "result.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2), flush=True)
        # MuJoCo mj_step computes sensors before advancing qpos. One 2ms translation
        # at 0.2m/s gives 0.4mm lag; 1mm also covers float32 DDS state quantization.
        assert errors and max(errors) < 0.001, result
        assert forward and all(abs(v[0] - 0.2) < 0.001 and abs(v[1]) < 0.001
                               for v in forward), result
        assert all(all(count > 10 for count in epochs) for epochs in counts.values()), result
        assert not any(duplicates.values()), result
        assert not any(after_stop.values()), result
        assert not result["cloud_odom_stamp_mismatches"], result
    finally:
        for _, process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for _, process in reversed(processes):
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for log in logs:
            log.close()
        remaining = [process.pid for _, process in processes
                     if Path(f"/proc/{process.pid}").exists()]
        node.destroy_node()
        rclpy.shutdown()
        print(f"OWNED_PROCESSES_REMAINING={remaining}", flush=True)
        if remaining:
            raise RuntimeError(f"owned processes remain: {remaining}")


if __name__ == "__main__":
    main()
