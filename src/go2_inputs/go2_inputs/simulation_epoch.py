"""시뮬레이션 clock 역행을 gate/adapter의 새 입력 epoch로 전달한다.

use_sim_time 노드만 등록하며 일반 입력의 timestamp regression 규칙은 유지한다.
반환 handler를 노드가 보관해야 한다. SLAM/AMCL 지도 상태는 관리하지 않는다.
"""

from collections.abc import Callable
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rclpy.node import Node


def watch_simulation_epoch(node: "Node", reset: Callable[[], None]):
    """공유 ROS clock jump 경계에서 상태를 비우고 epoch당 한 번 기록한다."""
    if not node.get_parameter("use_sim_time").value:
        return None
    from rclpy.clock import JumpThreshold
    from rclpy.duration import Duration

    def on_jump(jump) -> None:
        reset()
        node.get_logger().warning(
            "simulation time moved backwards; resetting input epoch"
        )

    return node.get_clock().create_jump_callback(
        JumpThreshold(
            min_forward=None,
            min_backward=Duration(nanoseconds=-1),
            on_clock_change=False,
        ),
        post_callback=on_jump,
    )
