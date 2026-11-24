"""Shared reliable, volatile single-cloud QoS for gate and obstacle observer."""

from typing import Final

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


POINT_CLOUD_QOS: Final = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)
