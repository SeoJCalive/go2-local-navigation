
"""격리 launch와 rosbag process의 bounded wait·graceful teardown을 제공한다."""
import os
import signal
import subprocess
from time import monotonic
from typing import Callable

import rclpy
from rclpy.node import Node


def spin_until(node: Node, condition: Callable[[], bool], timeout_sec: float) -> bool:
    """ROS callback을 처리하며 condition을 bounded wall time 안에서 기다린다."""
    deadline = monotonic() + timeout_sec
    while rclpy.ok() and monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if condition():
            return True
    return condition()


def spin_for(node: Node, duration_sec: float) -> None:
    """Teardown discovery와 마지막 callback을 위해 bounded spin한다."""
    deadline = monotonic() + duration_sec
    while rclpy.ok() and monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)


def stop_owned_process(process: subprocess.Popen[str]) -> int:
    """먼저 parent에만 SIGINT를 보내 launch의 단일 child 전파를 보장한다."""
    return_code = process.poll()
    if return_code is not None:
        return return_code
    process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
    try:
        return process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        return process.wait()
