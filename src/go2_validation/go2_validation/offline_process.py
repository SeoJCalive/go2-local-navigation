
"""격리 launch process의 bounded wait·graceful teardown을 제공한다."""
from time import monotonic
from typing import Callable

import rclpy
from rclpy.node import Node
from bringup.preflight_host import stop_owned_process as stop_owned_process


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
