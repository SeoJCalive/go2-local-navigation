"""통합 preflight observer의 rclpy 자원 소유권을 검증한다."""

import rclpy

from bringup.preflight_observer_node import IntegratedPreflightObserver


def test_observer_shutdown_preserves_rclpy_subscription_registry() -> None:
    # Given: TF listener와 필수 topic subscription을 생성한 observer
    rclpy.init()
    node = IntegratedPreflightObserver()

    # When: rclpy가 node 소유 entity를 정리한다.
    try:
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    # Then: destroy_node가 내부 subscription registry 오류 없이 반환한다.
