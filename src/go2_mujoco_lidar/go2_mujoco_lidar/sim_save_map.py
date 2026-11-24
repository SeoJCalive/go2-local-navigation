"""활성 mapping 세션의 occupancy map과 world/hash provenance를 저장한다.

별도 CLI로 실행하며 SLAM이 있고 AMCL이 없는 graph만 허용한다. Nav2 map_saver_cli의
성공 후 YAML/PGM을 다시 의미 검증하고 JSON sidecar를 쓴다. 기존 파일은 덮어쓰지
않는다. posegraph 저장이나 원자적 SLAM snapshot을 제공하는 명령은 아니다.
"""

import argparse
import json
from pathlib import Path
import subprocess
import time
import os

import rclpy

from go2_mujoco_lidar.sim_session import simulation_domain, simulator_world
from go2_nav2.sim_session import map_selection


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, help="absolute map prefix, without .yaml")
    parser.add_argument("--simulator-pid", required=True)
    args = parser.parse_args()
    domain = simulation_domain(os.environ)
    world, digest = simulator_world(args.simulator_pid, "", domain)
    output = Path(args.output).expanduser().resolve()
    if output.suffix:
        raise ValueError("output must be a map prefix without a file suffix")
    if any(output.with_suffix(suffix).exists() for suffix in (".yaml", ".pgm", ".session.json")):
        raise FileExistsError("map output exists; choose a new prefix")
    output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("go2_sim_map_save_check")
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            names = node.get_node_names()
            if "amcl" in names:
                raise RuntimeError("save requires mapping mode, but AMCL is present")
            if names.count("slam_toolbox") == 1:
                break
        else:
            raise RuntimeError("save requires one running SLAM Toolbox")
        subprocess.run([
            "ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(output),
            "--fmt", "pgm", "--ros-args", "-p", "use_sim_time:=true",
            "-p", "save_map_timeout:=10.0",
        ], check=True, timeout=30)
        record = {
            "map": map_selection(output.with_suffix(".yaml")),
            "world": str(world), "world_sha256": digest,
            "domain": domain, "map_odom_owner": "slam_toolbox",
        }
        output.with_suffix(".session.json").write_text(json.dumps(record, indent=2) + "\n")
        print(json.dumps(record, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()
