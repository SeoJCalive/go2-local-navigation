"""단일 명령으로 lifecycle을 FINALIZED로 만든 뒤 ROS launch만 종료한다.

`--launch-pid`는 현재 domain의 go2_sim_navigation launch PID다. simulator,
RL, bridge는 이 명령의 소유 대상이 아니다. lifecycle 실패 때 signal을 보내지
않으므로 clock/TF 입력이 먼저 사라지는 종료 경쟁을 만들지 않는다.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import time

from description.dds_environment import cyclonedds_network_matches
from go2_nav2.sim_session_guard import shutdown_managed_nodes


def stop(launch_pid, timeout=20.0):
    proc = Path(f"/proc/{launch_pid}")
    command = (proc / "cmdline").read_bytes().split(b"\0")
    if b"go2_sim_navigation.launch.py" not in command:
        raise ValueError("PID is not a go2_sim_navigation.launch.py process")
    environment = dict(
        field.split(b"=", 1) for field in (proc / "environ").read_bytes().split(b"\0")
        if b"=" in field)
    if environment.get(b"ROS_DOMAIN_ID") != os.environ.get("ROS_DOMAIN_ID", "").encode():
        raise ValueError("launch PID and stop command ROS_DOMAIN_ID differ")
    domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    if not (1 <= domain <= 232 and (
        os.environ.get("ROS_LOCALHOST_ONLY") == "1" or (
            os.environ.get("RMW_IMPLEMENTATION") == "rmw_cyclonedds_cpp" and
            cyclonedds_network_matches(os.environ.get("CYCLONEDDS_URI", ""), "lo", domain)))):
        raise ValueError("stop requires nonzero domain and loopback DDS")
    identity = (proc / "stat").read_text().rsplit(")", 1)[1].split()[19]
    states = shutdown_managed_nodes([
        ("navigation", ["planner_server", "controller_server", "behavior_server", "bt_navigator"]),
        ("localization", ["map_server", "amcl"]),
    ])
    print("MANAGED_SHUTDOWN_COMPLETE " + json.dumps(states, sort_keys=True), flush=True)
    if (proc / "stat").read_text().rsplit(")", 1)[1].split()[19] != identity:
        raise RuntimeError("launch PID identity changed")
    os.kill(launch_pid, signal.SIGINT)
    deadline = time.monotonic() + timeout
    while proc.exists():
        try:
            fields = (proc / "stat").read_text().rsplit(")", 1)[1].split()
        except FileNotFoundError:
            break
        if fields[0] == "Z" or fields[19] != identity:
            break
        if time.monotonic() >= deadline:
            raise TimeoutError("launch did not exit after lifecycle shutdown")
        time.sleep(0.05)
    return states


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--launch-pid", type=int, required=True)
    arguments = parser.parse_args()
    stop(arguments.launch_pid)
