"""Prove the real native GID tap on isolated AGX CycloneDDS, without simulation.

Known senders publish distinct numeric markers; their GUIDs come independently
from graph deltas at creation. The unchanged offline G03 judge must attribute
every received marker to its expected GUID. Cases cover sequential, overlapping,
external and 270 late command publications after 300 noise publications.
Output is a fresh directory containing raw TSV, graph bytes and JSON verdicts.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile

from saved_map import COMMAND_TOPICS, command_ownership


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--observer", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "41" or os.environ.get(
            "RMW_IMPLEMENTATION") != "rmw_cyclonedds_cpp":
        raise ValueError("instrument requires G03 Domain 41 and CycloneDDS")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    controller = rclpy.create_node("controller_server")
    behavior = rclpy.create_node("behavior_server")
    external = rclpy.create_node("g03_external")
    noise = [controller.create_publisher(Twist, f"/g03_instrument/noise_{i}", 1)
             for i in range(300)]
    status = controller.create_publisher(
        GoalStatusArray, "/follow_path/_action/status",
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    log_path = output / "native.log"
    known = {}
    graph_rows = []

    def until(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(controller, timeout_sec=.01)
            if predicate():
                return
        raise TimeoutError("instrument condition timed out")

    def graph():
        return [{"topic": topic, "gid": bytes(e.endpoint_gid).hex(),
                 "gid_bytes": list(e.endpoint_gid),
                 "node": e.node_namespace.rstrip("/") + "/" + e.node_name}
                for topic in COMMAND_TOPICS
                for e in controller.get_publishers_info_by_topic(topic)]

    def publisher(node, marker):
        before = {e["gid"] for e in graph()}
        pub = node.create_publisher(Twist, COMMAND_TOPICS[0], 10)
        until(lambda: len({e["gid"] for e in graph()} - before) == 1)
        endpoint = next(e for e in graph() if e["gid"] not in before)
        known[marker] = endpoint
        graph_rows.append(endpoint)
        until(lambda: pub.get_subscription_count() > 0)
        return pub

    def publish(pub, marker):
        message = Twist()
        message.linear.x = float(marker)
        pub.publish(message)
        until(lambda: any(line.startswith("COMMAND_OWNER\t") and
                          float(line.split("\t")[5]) == marker
                          for line in log_path.read_text().splitlines()))

    def gap():
        deadline = time.monotonic() + .30
        until(lambda: time.monotonic() >= deadline, 2)

    def action(state):
        message = GoalStatusArray()
        entry = GoalStatus()
        entry.goal_info.goal_id.uuid = [1] * 16
        entry.status = state
        message.status_list = [entry]
        status.publish(message)
        until(lambda: f"{'01'*16}:{state}" in log_path.read_text())

    def judge(name, markers, expected_pass, safety_stop=None):
        # Wait for delayed discovery metadata as the production offline judge
        # does. No publisher or message is selected by an assumed node identity.
        def mapped():
            lines = log_path.read_text().splitlines()
            gids = {line.split("\t")[1] for line in lines if line.startswith("GID_MAP\t")}
            rows = [line.split("\t") for line in lines if line.startswith("COMMAND_OWNER\t")
                    and float(line.split("\t")[5]) in markers]
            return len(rows) == len(markers) and all(row[4] in gids for row in rows)
        until(mapped)
        lines = log_path.read_text().splitlines()
        selected = "\n".join(line for line in lines if not line.startswith("COMMAND_OWNER\t")
                             or float(line.split("\t")[5]) in markers)
        endpoints = [known[m] for m in markers]
        result = command_ownership(selected, [{"cmd_vel": endpoints}], 0, 2**63-1, safety_stop)
        correct = all(row["publisher_gid"] == known[int(row["vx"])]["gid"]
                      and row["node"] == known[int(row["vx"])]["node"]
                      for row in result["messages"])
        assert len(result["messages"]) == len(markers), name
        assert correct and result["unidentified_or_external_messages"] == (
            1 if name in ("c_external", "f_safety_nonzero") else 0), name
        assert result["passed"] == expected_pass, (name, result)
        if name == "b_overlap":
            assert len(result["overlaps"]) == 1
        if name in ("a_sequential", "d_270_late_publishers"):
            assert not result["overlaps"]
        result["correct_attribution"] = correct
        result["expected_pass"] = expected_pass
        (output / f"{name}.json").write_text(json.dumps(result, indent=2) + "\n")
        return {"messages": len(result["messages"]), "passed": result["passed"],
                "expected_pass": expected_pass, "correct_attribution": correct,
                "unidentified": sum(row["node"] is None for row in result["messages"]),
                "overlaps": len(result["overlaps"]),
                "action_status": result["action_status"]}

    report = {"rmw": os.environ["RMW_IMPLEMENTATION"], "domain": 41, "cases": {}}
    with log_path.open("w") as log:
        observer = subprocess.Popen([
            str(args.observer.resolve()), "--ros-args", "-p", "observe_commands:=true"],
            stdout=log, stderr=subprocess.STDOUT)
        try:
            until(lambda: len([l for l in log_path.read_text().splitlines()
                              if l.startswith("GID_MAP\t")]) > 300)
            until(lambda: status.get_subscription_count() > 0)
            action(2)
            first = publisher(controller, 1)
            second = publisher(behavior, 2)
            publish(first, 1)
            gap()
            publish(second, 2)
            report["cases"]["a"] = judge("a_sequential", {1, 2}, True)
            action(4)
            gap()
            publish(first, 3)
            known[3] = known[1]
            publish(second, 4)
            known[4] = known[2]
            report["cases"]["b"] = judge("b_overlap", {3, 4}, False)
            gap()
            third = publisher(external, 5)
            publish(third, 5)
            report["cases"]["c"] = judge("c_external", {5}, False)
            external.destroy_publisher(third)
            many = [publisher(controller, marker) for marker in range(1000, 1270)]
            gap()
            for marker, pub in zip(range(1000, 1270), many):
                publish(pub, marker)
                gap()
            report["cases"]["d"] = judge(
                "d_270_late_publishers", set(range(1000, 1270)), True)
            safety = publisher(external, 0)
            stop = {"request_steady_ns": time.monotonic_ns(),
                    "zero_endpoints": [known[0]]}
            publish(safety, 0)
            report["cases"]["e"] = judge("e_safety_zero", {0}, True, stop)
            known[6] = known[0]
            publish(safety, 6)
            report["cases"]["f"] = judge("f_safety_nonzero", {6}, False, stop)
            mappings = [l.split("\t")[1:] for l in log_path.read_text().splitlines()
                        if l.startswith("GID_MAP\t")]
            report["metadata_count"] = len(mappings)
            assert len(mappings) > 570
            assert report["cases"]["a"]["action_status"]["messages_with_active_action"] == 2
            assert report["cases"]["b"]["action_status"]["messages_with_active_action"] == 0
            report["passed"] = True
        finally:
            observer.send_signal(signal.SIGINT)
            observer.wait(timeout=10)
            report["observer_exit"] = observer.returncode
            report["observer_reaped"] = not Path(f"/proc/{observer.pid}").exists()
    (output / "known_graph.json").write_text(json.dumps(graph_rows, indent=2) + "\n")
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    for node in (external, behavior, controller):
        node.destroy_node()
    rclpy.shutdown()
    print("INSTRUMENT_RESULT " + json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
