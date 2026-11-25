#!/usr/bin/env python3
"""공유 GUI binary를 Xvfb에서 실행해 DDS clock·입력 연속성과 Backspace reset을 검사한다.

실제 GUI main/physics/bridge와 설치된 bringup을 사용한다. RL·motion 명령은 없다.
정수 1 ms 격자와 두 DDS stream의 exact stamp 교집합이 독립 시간 oracle다.
sim-time 610초 및 reset 후 5초를 event로 기다리며 wall timeout으로 제한한다.
공유 binary의 동일 inode hard link와 pinned 기본 config로 전용 실행 경로를 만든다.
자신의 POSIX session만 정리하고 기존 process identity를 비교한다.
"""
import argparse
import ctypes
import json
import os
from pathlib import Path
import select
import shutil
import signal
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from unitree_go.msg import LowState, SportModeState

from owned_processes import OwnedProcesses, existing_stack


def reset_key(display_name):
    """전용 X server의 GUI 창에 실제 XTest Backspace press/release를 보낸다."""
    x = ctypes.CDLL("libX11.so.6")
    xt = ctypes.CDLL("libXtst.so.6")
    ptr = ctypes.c_void_p
    x.XOpenDisplay.argtypes, x.XOpenDisplay.restype = [ctypes.c_char_p], ptr
    x.XDefaultRootWindow.argtypes, x.XDefaultRootWindow.restype = [ptr], ctypes.c_ulong
    x.XQueryTree.argtypes = [
        ptr, ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)),
        ctypes.POINTER(ctypes.c_uint)]
    x.XSetInputFocus.argtypes = [ptr, ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
    x.XFetchName.argtypes = [ptr, ctypes.c_ulong, ctypes.POINTER(ptr)]
    x.XKeysymToKeycode.argtypes, x.XKeysymToKeycode.restype = [
        ptr, ctypes.c_ulong], ctypes.c_uint
    x.XSync.argtypes = [ptr, ctypes.c_int]
    x.XFree.argtypes = [ptr]
    x.XCloseDisplay.argtypes = [ptr]
    xt.XTestFakeKeyEvent.argtypes = [ptr, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]
    d = x.XOpenDisplay(display_name.encode())
    if not d:
        raise RuntimeError("XOpenDisplay failed")
    try:
        root = x.XDefaultRootWindow(d)
        root_out, parent = ctypes.c_ulong(), ctypes.c_ulong()
        children = ctypes.POINTER(ctypes.c_ulong)()
        count = ctypes.c_uint()
        if not x.XQueryTree(d, root, ctypes.byref(root_out), ctypes.byref(parent),
                            ctypes.byref(children), ctypes.byref(count)):
            raise RuntimeError("XQueryTree failed")
        windows = []
        for index in range(count.value):
            name = ptr()
            if x.XFetchName(d, children[index], ctypes.byref(name)) and name:
                title = ctypes.string_at(name).decode()
                x.XFree(name)
                if "MuJoCo" in title:
                    windows.append((children[index], title))
        x.XFree(children)
        if len(windows) != 1:
            raise RuntimeError(f"expected one MuJoCo window, got {windows}")
        window, title = windows[0]
        x.XSetInputFocus(d, window, 1, 0)
        code = x.XKeysymToKeycode(d, 0xFF08)
        if not xt.XTestFakeKeyEvent(d, code, 1, 0):
            raise RuntimeError("XTest press failed")
        xt.XTestFakeKeyEvent(d, code, 0, 0)
        x.XSync(d, 0)
        return title
    finally:
        x.XCloseDisplay(d)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulator", type=Path, required=True)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    project = Path(__file__).resolve().parents[4]
    before = existing_stack(project)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupted(signum, _frame):
        raise RuntimeError(f"GUI verification interrupted by signal {signum}")
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    node = rclpy.create_node("f04_gui_clock_observer")
    owned = OwnedProcesses(lambda: rclpy.spin_once(node, timeout_sec=.05))
    samples = {name: [[], []] for name in ("low", "sport", "cloud", "odom")}
    epochs = {name: 0 for name in samples}
    last = {}
    invalid_sport = []
    subscriptions = []

    def record(name, stamp):
        if name in last and stamp < last[name]:
            epochs[name] += 1
        last[name] = stamp
        if epochs[name] > 1:
            raise RuntimeError(f"unexpected extra reset in {name}")
        samples[name][epochs[name]].append(stamp)
        if name == "sport" and stamp % 1_000_000:
            invalid_sport.append(stamp)

    for name, topic, msg_type, stamp in [
        ("low", "/lowstate", LowState, lambda m: m.tick * 1_000_000),
        ("sport", "/sportmodestate", SportModeState,
         lambda m: m.stamp.sec * 10**9 + m.stamp.nanosec),
        ("cloud", "/utlidar/cloud", PointCloud2,
         lambda m: m.header.stamp.sec * 10**9 + m.header.stamp.nanosec),
        ("odom", "/odom", Odometry,
         lambda m: m.header.stamp.sec * 10**9 + m.header.stamp.nanosec),
    ]:
        subscriptions.append(node.create_subscription(
            msg_type, topic, lambda m, n=name, s=stamp: record(n, s(m)),
            qos_profile_sensor_data))

    def until(condition, timeout, label):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() >= deadline:
                raise TimeoutError(label)
            for process in owned.processes:
                if process.poll() is not None:
                    raise RuntimeError(f"owned process exited: {process.args}")
            rclpy.spin_once(node, timeout_sec=.1)

    logs = []
    result = {"passed": False, "preexisting": before}
    began = time.monotonic()
    try:
        # getExecutableDir() selects ../config.yaml. A hard link keeps the actual
        # shared binary/inode while isolating config from the old joystick FIFO.
        stage = args.output / "simulate"
        (stage / "build").mkdir(parents=True)
        simulator = stage / "build/unitree_mujoco"
        os.link(args.simulator, simulator)
        shutil.copyfile(args.config, stage / "config.yaml")
        result["same_binary_inode"] = os.path.samefile(args.simulator, simulator)
        result["source_binary"] = str(args.simulator)
        result["config_snapshot"] = str(args.config)
        read_fd, write_fd = os.pipe()
        log = (args.output / "xvfb.log").open("w")
        logs.append(log)
        try:
            owned.start(["Xvfb", "-displayfd", str(write_fd), "-screen", "0",
                         "1024x768x24", "-nolisten", "tcp"],
                        pass_fds=(write_fd,), stdout=log, stderr=log)
        finally:
            os.close(write_fd)
        try:
            if not select.select([read_fd], [], [], 30)[0]:
                raise TimeoutError("Xvfb display readiness")
            display = ":" + os.read(read_fd, 80).decode().strip()
        finally:
            os.close(read_fd)
        env = dict(os.environ, DISPLAY=display, LIBGL_ALWAYS_SOFTWARE="1",
                   __GLX_VENDOR_LIBRARY_NAME="mesa")
        # GUI uses SDK libraries rather than ROS overlay libraries.
        native_env = {key: value for key, value in env.items()
                      if key not in {"LD_LIBRARY_PATH", "LD_PRELOAD", "AMENT_PREFIX_PATH",
                                     "PYTHONPATH", "COLCON_PREFIX_PATH"}}
        log = (args.output / "simulator.log").open("w")
        logs.append(log)
        sim = owned.start(["stdbuf", "-oL", "-eL", str(simulator), "-r", "go2", "-s", str(args.world),
                           "-i", "41", "-n", "lo"], env=native_env, stdout=log, stderr=log)
        until(lambda: len(samples["sport"][0]) >= 10, 60, "GUI DDS readiness")
        print("GUI_DDS_READY", flush=True)
        log = (args.output / "bringup.log").open("w")
        logs.append(log)
        owned.start(["ros2", "launch", "go2_mujoco_lidar", "go2_sim_bringup.launch.py",
                     f"simulator_pid:={sim.pid}", "mapping:=false"],
                    env=env, stdout=log, stderr=log)
        until(lambda: all(samples[n][0] and samples[n][0][-1] >= 610 * 10**9
                          for n in ("sport", "cloud", "odom")) and
              all(samples[n][0][-1] - samples[n][0][0] >= 600 * 10**9
                  for n in ("cloud", "odom")),
              1200, "610 s GUI physics and continuous outputs")
        print("GUI_610S_REACHED", flush=True)
        result["reset_window"] = reset_key(display)
        until(lambda: all(epochs[n] == 1 and samples[n][1][-1] >= 5 * 10**9
                          for n in samples), 60, "GUI Backspace reset recovery")
        stats = {}
        for name, values in samples.items():
            stats[name] = []
            for stamps in values:
                stats[name].append({
                    "count": len(stamps), "first_ns": stamps[0], "last_ns": stamps[-1],
                    "max_gap_ns": max(b - a for a, b in zip(stamps, stamps[1:])),
                })
        common = [len(set(samples["low"][i]) & set(samples["sport"][i]))
                  for i in range(2)]
        result.update(
            streams=stats, stamp_mismatches=len(invalid_sport),
            exact_common_steps=common, reset_epochs=epochs, display=display,
            criteria={
                "zero_stamp_mismatches": not invalid_sport,
                "exact_pairing": common[0] > 200000 and common[1] > 1000,
                "long_epoch": all(stats[n][0]["last_ns"] - stats[n][0]["first_ns"]
                                  >= 600 * 10**9 for n in ("cloud", "odom")),
                "continuous_outputs": all(stats[n][0]["max_gap_ns"] < 10**9
                                          for n in ("cloud", "odom")),
                "reset_recovered": all(v == 1 for v in epochs.values()),
            })
    finally:
        for process in reversed(owned.processes):
            owned.stop(process)
        for log in logs:
            log.close()
        (args.output / "stamps.json").write_text(json.dumps(samples) + "\n")
        after = existing_stack(project)
        result.update(wall_sec=time.monotonic() - began, shutdowns=owned.stops,
                      residue=owned.remaining(),
                      preexisting_after=after, preexisting_unchanged=before == after)
        result["passed"] = (bool(result.get("criteria")) and all(result["criteria"].values())
                            and not result["residue"] and result["preexisting_unchanged"]
                            and all(9 not in stop["signals"] for stop in owned.stops))
        (args.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    if not result["passed"]:
        raise RuntimeError("GUI clock verification failed")


if __name__ == "__main__":
    main()
