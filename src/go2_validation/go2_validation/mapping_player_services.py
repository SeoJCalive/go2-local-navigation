"""
Paused rosbag player의 graph discovery와 Resume handshake를 소유한다.

외부 player process가 생성된 뒤 두 원본 topic의 publisher identity와 Resume service를
확인하고, empty Resume 응답이 성공적으로 완료된 시점부터 playback deadline을 만든다.

"""
from dataclasses import dataclass
from time import monotonic
from typing import Final

import rclpy
from rclpy.node import Node
from rosbag2_interfaces.srv import Resume


PLAYER_NODE_PATH: Final = "/rosbag2_player"
PLAYER_RESUME_SERVICE: Final = "/rosbag2_player/resume"
PLAYER_TOPICS: Final = ("/utlidar/cloud", "/utlidar/robot_odom")
GRAPH_SPIN_SECONDS: Final = 0.1


@dataclass(frozen=True, slots=True)
class MappingRuntimeError(Exception):
    """Mapping startup 또는 owned process가 bounded contract를 위반했다."""

    reason_code: str
    detail: str = ""

    def __str__(self) -> str:
        return f"{self.reason_code}: {self.detail}" if self.detail else self.reason_code


class MappingPlayerServices:
    """한 observer에서 player publisher와 Resume 완료를 순서대로 확인한다."""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._resume = node.create_client(Resume, PLAYER_RESUME_SERVICE)

    def synchronize_start(
        self,
        readiness_timeout_seconds: float,
        resume_timeout_seconds: float,
        playback_timeout_seconds: float,
    ) -> float:
        """Discovery와 Resume 완료 뒤 playback wall-time deadline을 반환한다."""
        self.wait_until_ready(readiness_timeout_seconds)
        self.resume(resume_timeout_seconds)
        return monotonic() + playback_timeout_seconds

    def wait_until_ready(self, timeout_seconds: float) -> None:
        """두 player publisher와 Resume service를 한 deadline 안에서 발견한다."""
        deadline = monotonic() + timeout_seconds
        while rclpy.ok() and monotonic() < deadline:
            if self._publishers_ready() and self._resume.service_is_ready():
                return
            remaining = deadline - monotonic()
            rclpy.spin_once(
                self._node,
                timeout_sec=min(GRAPH_SPIN_SECONDS, max(0.0, remaining)),
            )
        raise MappingRuntimeError("mapping_player_readiness_failed")

    def resume(self, timeout_seconds: float) -> None:
        """Empty Resume request가 예외 없이 응답을 반환할 때까지 spin한다."""
        future = self._resume.call_async(Resume.Request())
        deadline = monotonic() + timeout_seconds
        while rclpy.ok() and monotonic() < deadline and not future.done():
            remaining = deadline - monotonic()
            rclpy.spin_once(
                self._node,
                timeout_sec=min(GRAPH_SPIN_SECONDS, max(0.0, remaining)),
            )
        if not future.done():
            raise MappingRuntimeError("mapping_player_resume_timeout")
        failure = future.exception()
        if failure is not None:
            raise MappingRuntimeError(
                "mapping_player_resume_failed",
                type(failure).__name__,
            )
        if future.result() is None:
            raise MappingRuntimeError("mapping_player_resume_failed", "response_absent")

    def _publishers_ready(self) -> bool:
        return all(
            any(
                f"{endpoint.node_namespace.rstrip('/')}/{endpoint.node_name}"
                == PLAYER_NODE_PATH
                for endpoint in self._node.get_publishers_info_by_topic(topic)
            )
            for topic in PLAYER_TOPICS
        )
