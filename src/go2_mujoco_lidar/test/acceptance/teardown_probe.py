"""실제 Nav2 planner의 library 및 lifecycle 종료 순서를 짧은 동일 세션으로 비교한다.

각 arm은 같은 static map, TF, clock, configure/activate, clock reset을 쓴다.
pin/system은 LD_PRELOAD와 library search path만 다르다. pin_ordered는 같은
pin에서 deactivate/cleanup을 먼저 완료해 signal-first 종료와 비교한다.
새 process session만 소유하며 정지 시 bounded batch gdb stack과 결과를 남긴다.
PR_SET_PTRACER는 자신이 시작하는 native child에만 적용해 자신의 debugger를 허용한다.
"""

import argparse
import ctypes
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.clock import Clock as ROSClock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from tf2_ros import TransformBroadcaster

from owned_processes import OwnedProcesses, existing_stack, snapshot


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pin", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cycles", type=int, default=5)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "41" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("W07 probe requires Domain 41 and localhost")
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("w07_tf2_teardown_probe")
    clock = node.create_publisher(Clock, "/clock", 10)
    maps = node.create_publisher(
        OccupancyGrid, "/map", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    broadcaster = TransformBroadcaster(node)
    samples = []
    subscription = node.create_subscription(
        OccupancyGrid, "/global_costmap/costmap",
        lambda m: samples.append(m.header.stamp.sec * 10**9 + m.header.stamp.nanosec),
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    now = [100 * 10**9]

    def tick():
        now[0] += 20_000_000
        message = Clock()
        message.clock.sec, message.clock.nanosec = divmod(now[0], 10**9)
        clock.publish(message)
        transform = TransformStamped()
        transform.header.stamp = message.clock
        transform.header.frame_id, transform.child_frame_id = "map", "base"
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)

    timer = node.create_timer(0.01, tick, clock=ROSClock(clock_type=ClockType.STEADY_TIME))
    manager = OwnedProcesses(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    libc = ctypes.CDLL(None, use_errno=True)
    tracer = os.getpid()

    def allow_owned_debugger():
        if libc.prctl(0x59616D61, tracer, 0, 0, 0) != 0:
            os._exit(126)

    def until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            if predicate():
                return True
        return False

    def require(predicate, timeout, detail):
        if not until(predicate, timeout):
            raise TimeoutError(detail)

    results, logs = [], []
    baseline = snapshot()
    protected = existing_stack(Path(__file__).resolve().parents[4])
    (args.output / "preexisting.json").write_text(json.dumps(protected, indent=2))
    try:
        for arm, preload, ordered in (
                ("pin", True, False), ("system", False, False), ("pin_ordered", True, True)):
            for cycle in range(1, args.cycles + 1):
                now[0] = (100 + cycle * 100) * 10**9
                samples.clear()
                grid = OccupancyGrid()
                grid.header.frame_id = "map"
                grid.info.resolution = 0.05
                grid.info.width = grid.info.height = 100
                grid.info.origin.position.x = grid.info.origin.position.y = -2.5
                grid.info.origin.orientation.w = 1.0
                grid.data = [100 if x in (0, 99) or y in (0, 99) else 0
                             for y in range(100) for x in range(100)]
                maps.publish(grid)
                environment = os.environ.copy()
                environment.pop("LD_PRELOAD", None)
                environment["LD_LIBRARY_PATH"] = ":".join(
                    item for item in environment.get("LD_LIBRARY_PATH", "").split(":")
                    if item and "/tf2-pin/" not in item)
                if preload:
                    environment["LD_PRELOAD"] = str(args.pin.resolve())
                    environment["LD_LIBRARY_PATH"] = str(args.pin.resolve().parent) + ":" + environment["LD_LIBRARY_PATH"]
                logfile = (args.output / f"{arm}-{cycle}.log").open("w")
                logs.append(logfile)
                child = manager.start(
                    ["/opt/ros/humble/lib/nav2_planner/planner_server", "--ros-args",
                     "-r", "__node:=planner_server", "--params-file",
                     str(Path(__file__).with_suffix(".yaml"))],
                    env=environment, stdout=logfile, stderr=subprocess.STDOUT,
                    preexec_fn=allow_owned_debugger)
                state = node.create_client(GetState, "/planner_server/get_state")
                change = node.create_client(ChangeState, "/planner_server/change_state")
                require(lambda: state.service_is_ready() and change.service_is_ready(),
                        15, "planner lifecycle services")

                def transition(identifier, expected):
                    request = ChangeState.Request()
                    request.transition.id = identifier
                    future = change.call_async(request)
                    require(future.done, 10, "lifecycle transition")
                    if not future.result().success:
                        raise RuntimeError("lifecycle rejected")
                    future = state.call_async(GetState.Request())
                    require(future.done, 5, "lifecycle observed state")
                    if future.result().current_state.id != expected:
                        raise RuntimeError("independent lifecycle state mismatch")

                transition(Transition.TRANSITION_CONFIGURE, State.PRIMARY_STATE_INACTIVE)
                transition(Transition.TRANSITION_ACTIVATE, State.PRIMARY_STATE_ACTIVE)
                require(lambda: len(samples) >= 2, 10, "active costmap output")
                before = samples[-1]
                now[0] = 0
                require(lambda: len(samples) >= 4 and samples[-1] < before, 10,
                        "costmap output after clock reset")
                libraries = sorted({line.split()[-1] for line in
                                    Path(f"/proc/{child.pid}/maps").read_text().splitlines()
                                    if "libtf2.so" in line})
                if ordered:
                    transition(Transition.TRANSITION_DEACTIVATE, State.PRIMARY_STATE_INACTIVE)
                    transition(Transition.TRANSITION_CLEANUP, State.PRIMARY_STATE_UNCONFIGURED)
                started = time.monotonic()
                child.send_signal(signal.SIGINT)
                clean = until(lambda: child.poll() is not None, 4)
                row = {"arm": arm, "cycle": cycle, "pid": child.pid, "libraries": libraries,
                       "hung_after_4s": not clean, "exit_before_escalation": child.poll(),
                       "observed_wait_s": time.monotonic() - started}
                if not clean:
                    capture = subprocess.run(
                        ["gdb", "-batch", "-p", str(child.pid), "-ex", "thread apply all bt"],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=8)
                    (args.output / f"{arm}-{cycle}.stack").write_text(capture.stdout)
                    row["gdb_exit"] = capture.returncode
                manager.stop(child)
                node.destroy_client(change)
                node.destroy_client(state)
                row["remaining"] = manager.remaining()
                results.append(row)
                (args.output / "result.json").write_text(json.dumps(results, indent=2))
                print("TEARDOWN_TRIAL " + json.dumps(row), flush=True)
    finally:
        for child in reversed(manager.processes):
            manager.stop(child)
        for logfile in logs:
            logfile.close()
        after = snapshot()
        protected_after = existing_stack(Path(__file__).resolve().parents[4])
        audit = {
            "preexisting": protected,
            "preexisting_after": {pid: protected_after.get(pid) for pid in protected},
            "preexisting_unchanged": all(
                protected_after.get(pid) == row for pid, row in protected.items()),
            "owned_remaining": manager.remaining(),
        }
        (args.output / "audit.json").write_text(json.dumps(audit, indent=2))
        print("PROBE_ZERO_RESIDUE", manager.remaining(), flush=True)
        print("PROTECTED_DOMAINS_UNCHANGED", audit["preexisting_unchanged"], flush=True)
        print("PREEXISTING_IDENTITIES_CHANGED", [
            pid for pid, row in baseline.items()
            if pid in after and after[pid]["start_ticks"] != row["start_ticks"]], flush=True)
        node.destroy_node()
        rclpy.shutdown()
    print("TEARDOWN_COUNTS", {
        arm: sum(row["hung_after_4s"] for row in results if row["arm"] == arm)
        for arm in ("pin", "system", "pin_ordered")}, flush=True)


if __name__ == "__main__":
    main()
