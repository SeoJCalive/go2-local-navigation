"""통합 preflight observer의 rclpy 자원 소유권을 검증한다."""

from typing import Final

import pytest
import rclpy

from bringup.preflight_observer_node import IntegratedPreflightObserver


LOOPBACK_CYCLONEDDS_URI: Final = (
    "<CycloneDDS><Domain><General><Interfaces>"
    '<NetworkInterface name="lo" priority="default" multicast="false" />'
    "</Interfaces></General></Domain></CycloneDDS>"
)


def test_observer_shutdown_preserves_rclpy_subscription_registry(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Given: TF listener와 필수 topic subscription을 생성한 observer
    monkeypatch.setenv("CYCLONEDDS_URI", LOOPBACK_CYCLONEDDS_URI)
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "0")
    rclpy.init()
    node = IntegratedPreflightObserver()

    # When: rclpy가 node 소유 entity를 정리한다.
    try:
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    # Then: destroy_node가 내부 subscription registry 오류 없이 반환한다.
