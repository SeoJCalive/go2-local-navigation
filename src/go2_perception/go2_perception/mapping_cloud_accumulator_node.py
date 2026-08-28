"""검증된 cloud를 odom frame으로 시간 보정해 짧게 누적한다.

선택된 TF tree의 ``odom → base → sensor``를 cloud stamp에서 조회한다. 출력은
mapping 전용 PointCloud2이며 command·service·물리 제어 interface를 만들지 않는다.
"""

from typing import Final

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from go2_perception.mapping_cloud_accumulator import (
    MappingCloudRetryQueue,
    MappingCloudWindow,
    MappingCloudWindowError,
    compact_xyz_cloud,
    format_mapping_cloud_accounting,
)
from go2_perception.mapping_cloud_gate_node import OUTPUT_TOPIC as VALIDATED_TOPIC
from go2_perception.obstacle_candidate_node import POINT_CLOUD_QOS


DEFAULT_OUTPUT_TOPIC: Final = "/go2_mapping/cloud_accumulated"
DEFAULT_TARGET_FRAME: Final = "odom"
DEFAULT_FRAME_LIMIT: Final = 10
DEFAULT_EMIT_EVERY: Final = 1
DEFAULT_INPUT_QOS_DEPTH: Final = 64
DEFAULT_RETRY_QUEUE_CAPACITY: Final = 64
RETRY_PERIOD_SECONDS: Final = 0.01


class MappingCloudAccumulatorNode(Node):
    """Cloud stamp의 TF로 변환한 뒤 고정 길이 window를 publish한다."""

    def __init__(self) -> None:
        super().__init__("go2_mapping_cloud_accumulator")
        frame_limit = int(
            self.declare_parameter("frame_limit", DEFAULT_FRAME_LIMIT).value
        )
        emit_every = int(
            self.declare_parameter("emit_every", DEFAULT_EMIT_EVERY).value
        )
        input_qos_depth = int(
            self.declare_parameter(
                "input_qos_depth",
                DEFAULT_INPUT_QOS_DEPTH,
            ).value
        )
        target_frame = str(
            self.declare_parameter("target_frame", DEFAULT_TARGET_FRAME).value
        )
        output_topic = str(
            self.declare_parameter("output_topic", DEFAULT_OUTPUT_TOPIC).value
        )
        retry_queue_capacity = int(
            self.declare_parameter(
                "retry_queue_capacity",
                DEFAULT_RETRY_QUEUE_CAPACITY,
            ).value
        )
        self._window = MappingCloudWindow(frame_limit, target_frame, emit_every)
        self._target_frame = target_frame
        self._retry_queue = MappingCloudRetryQueue[PointCloud2](
            retry_queue_capacity,
            emit_every,
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            POINT_CLOUD_QOS,
        )
        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=input_qos_depth,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            PointCloud2,
            VALIDATED_TOPIC,
            self._on_cloud,
            input_qos,
        )
        self._retry_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(
            RETRY_PERIOD_SECONDS,
            self._process_pending_clouds,
            clock=self._retry_clock,
        )
        self.get_logger().info(
            f"accumulating {frame_limit} validated clouds in {target_frame}, "
            f"emitting every {emit_every}, input depth {input_qos_depth}, "
            f"retry queue capacity {retry_queue_capacity}; "
            f"publishing {output_topic}"
        )

    def _on_cloud(self, message: PointCloud2) -> None:
        result = self._retry_queue.enqueue(message, _stamp_nanoseconds(message))
        if result.drop_reason is not None:
            self.get_logger().warning(
                f"mapping cloud accumulation dropped: {result.drop_reason}"
            )

    def _process_pending_clouds(self) -> None:
        while (pending := self._retry_queue.head()) is not None:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    pending.cloud.header.frame_id,
                    Time.from_msg(pending.cloud.header.stamp),
                )
                transformed = do_transform_cloud(
                    compact_xyz_cloud(pending.cloud),
                    transform,
                )
                accumulated = self._window.add(transformed)
            except TransformException as error:
                if _is_future_extrapolation(error):
                    self._retry_queue.mark_head_future_waited()
                    return
                self._retry_queue.drop_head_unrecoverable()
                self.get_logger().warning(
                    f"mapping cloud accumulation dropped: TF lookup failed: {error}"
                )
                continue
            except (AssertionError, MappingCloudWindowError, TypeError, ValueError) as error:
                self._retry_queue.drop_head_unrecoverable()
                self.get_logger().warning(f"mapping cloud accumulation dropped: {error}")
                continue
            processed = self._retry_queue.take_head_for_processing()
            if processed is None:
                continue
            if accumulated is None:
                continue
            self._publisher.publish(accumulated)
            self._retry_queue.record_output_publish(_stamp_nanoseconds(accumulated))


def _stamp_nanoseconds(message: PointCloud2) -> int:
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def _is_future_extrapolation(error: TransformException) -> bool:
    return "extrapolation into the future" in str(error).lower()


def _emit_terminal_accounting(node: MappingCloudAccumulatorNode) -> None:
    accounting = node._retry_queue.accounting(node._window.partial_frame_count)
    print(format_mapping_cloud_accounting(accounting), flush=True)


def main(args: list[str] | None = None) -> None:
    """비동작 mapping cloud accumulator를 ROS 종료까지 실행한다."""
    rclpy.init(args=args)
    node = MappingCloudAccumulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("mapping cloud accumulator interrupted")
    except ExternalShutdownException:
        return
    finally:
        _emit_terminal_accounting(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
