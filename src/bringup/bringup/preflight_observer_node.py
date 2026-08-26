"""
전체 비동작 stack의 topic·TF·graph·motion 안전 상태를 관찰한다.

필수 입력과 파생 출력은 subscribe·graph 조회만 하며 command를 publish하지 않는다.
지정 시간이 끝나면 observer JSON을 저장하고 process가 종료되어 launch shutdown을
유발한다. host 자원과 teardown은 외부 `preflight_runner_node.py`가 추가한다.
"""

from datetime import datetime
from pathlib import Path
from time import monotonic

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from bringup.preflight_accumulator import TopicAccumulator
from bringup.preflight_assessments import (
    assess_graph,
    assess_safety,
    assess_transforms,
    collect_environment,
    project_publisher_count,
)
from bringup.preflight_configuration import (
    COMMAND_TOPICS,
    EXPECTED_NODES,
    REQUIRED_TRANSFORMS,
    STARTUP_GRACE_SECONDS,
    TOPIC_CONTRACTS,
)
from bringup.preflight_metrics import (
    assess_stationary_pose,
    assess_topic,
    overall_status,
)
from bringup.preflight_report import (
    GraphObservation,
    ObserverReport,
    SafetyObservation,
    TransformObservation,
    write_observer_report,
)
from bringup.preflight_subscriptions import (
    bind_preflight_subscriptions,
)
from bringup.preflight_types import (
    CheckResult,
    CheckStatus,
    ObservedMessage,
)


class IntegratedPreflightObserver(Node):
    """원문을 저장하지 않고 전체 비동작 stack의 합격 통계만 수집한다."""

    def __init__(self) -> None:
        super().__init__("go2_integrated_preflight_observer")
        self.duration_seconds = self.declare_parameter(
            "duration_sec", 30
        ).get_parameter_value().integer_value
        self.run_id = self.declare_parameter(
            "run_id", "preflight-unset"
        ).get_parameter_value().string_value
        self.run_label = self.declare_parameter(
            "run_label", "preflight"
        ).get_parameter_value().string_value
        report_value = self.declare_parameter(
            "report_path", "observer.json"
        ).get_parameter_value().string_value
        self.report_path = Path(report_value)
        self._started_at = datetime.now().astimezone()
        self._started_monotonic = monotonic()
        self._topics = {
            contract.topic: TopicAccumulator(contract)
            for contract in TOPIC_CONTRACTS
        }
        self._seen_nodes: set[str] = set()
        self._lost_nodes: set[str] = set()
        self._missing_streak = {node: 0 for node in EXPECTED_NODES}
        self._command_max_publishers = {topic: 0 for topic in COMMAND_TOPICS}
        self._gate_values: tuple[bool, bool] | None = None
        self._gate_future = None
        self._gate_client = self.create_client(
            GetParameters,
            "/go2_motion_adapter/get_parameters",
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._preflight_subscriptions = bind_preflight_subscriptions(self, self)
        self.create_timer(1.0, self._observe_graph)
        self.get_logger().info(
            f"integrated preflight observer started: run_id={self.run_id} "
            f"duration_sec={self.duration_seconds}"
        )

    def observe_topic(self, topic: str, sample: ObservedMessage) -> None:
        """변환된 subscription 표본을 해당 accumulator에 전달한다."""
        self._topics[topic].observe(sample)

    def _observe_graph(self) -> None:
        type_map = {
            topic: tuple(types)
            for topic, types in self.get_topic_names_and_types()
        }
        for contract in TOPIC_CONTRACTS:
            self._topics[contract.topic].observe_graph(
                type_map.get(contract.topic, ()),
                len(self.get_publishers_info_by_topic(contract.topic)),
            )
        current_nodes = {
            f"/{name}" if namespace == "/" else f"{namespace}/{name}"
            for name, namespace in self.get_node_names_and_namespaces()
        }
        self._seen_nodes.update(current_nodes.intersection(EXPECTED_NODES))
        if monotonic() - self._started_monotonic >= STARTUP_GRACE_SECONDS:
            for node_name in EXPECTED_NODES:
                if node_name in current_nodes:
                    self._missing_streak[node_name] = 0
                else:
                    self._missing_streak[node_name] += 1
                    if self._missing_streak[node_name] >= 3:
                        self._lost_nodes.add(node_name)
        for topic in COMMAND_TOPICS:
            endpoints = tuple(self.get_publishers_info_by_topic(topic))
            self._command_max_publishers[topic] = max(
                self._command_max_publishers[topic],
                project_publisher_count(endpoints, EXPECTED_NODES),
            )
        self._observe_gate()

    def _observe_gate(self) -> None:
        if self._gate_values is not None:
            return
        if self._gate_future is None and self._gate_client.service_is_ready():
            request = GetParameters.Request()
            request.names = ["output_enabled", "physical_validation_approved"]
            self._gate_future = self._gate_client.call_async(request)
            return
        if self._gate_future is None or not self._gate_future.done():
            return
        if self._gate_future.exception() is not None:
            return
        response = self._gate_future.result()
        if (
            len(response.values) == 2
            and all(
                value.type == ParameterType.PARAMETER_BOOL
                for value in response.values
            )
        ):
            self._gate_values = (
                response.values[0].bool_value,
                response.values[1].bool_value,
            )

    def finalize(self) -> CheckStatus:
        """마지막 graph를 반영하고 observer JSON을 저장한다."""
        self._observe_graph()
        topic_summaries = tuple(
            self._topics[contract.topic].summary()
            for contract in TOPIC_CONTRACTS
        )
        topic_checks = tuple(
            check
            for summary in topic_summaries
            for check in assess_topic(summary)
        )
        odometry_summary = next(
            summary
            for summary in topic_summaries
            if summary.contract.topic == "/odom"
        )
        environment, environment_check = collect_environment()
        graph = GraphObservation(
            expected_nodes=EXPECTED_NODES,
            seen_nodes=tuple(sorted(self._seen_nodes)),
            lost_nodes=tuple(sorted(self._lost_nodes)),
        )
        transforms = tuple(
            TransformObservation(
                parent_frame=parent,
                child_frame=child,
                available=self._tf_buffer.can_transform(parent, child, Time()),
            )
            for parent, child in REQUIRED_TRANSFORMS
        )
        gate_values = self._gate_values or (None, None)
        safety = SafetyObservation(
            output_enabled=gate_values[0],
            physical_validation_approved=gate_values[1],
            sport_request_max_publishers=self._command_max_publishers[
                "/api/sport/request"
            ],
            lowcmd_max_publishers=self._command_max_publishers["/lowcmd"],
        )
        checks: tuple[CheckResult, ...] = (
            environment_check,
            assess_graph(graph),
            *assess_transforms(transforms),
            *assess_safety(safety),
            *topic_checks,
            assess_stationary_pose(odometry_summary),
        )
        status = overall_status(checks)
        completed_at = datetime.now().astimezone()
        write_observer_report(
            ObserverReport(
                schema_version=1,
                record_kind="integrated_non_actuating_preflight_observer",
                run_id=self.run_id,
                run_label=self.run_label,
                target="go2_agx",
                started_at=self._started_at.isoformat(),
                completed_at=completed_at.isoformat(),
                requested_duration_seconds=self.duration_seconds,
                actual_duration_seconds=monotonic() - self._started_monotonic,
                physical_motion=False,
                command_publication=False,
                overall_status=status,
                checks=checks,
                environment=environment,
                graph=graph,
                transforms=transforms,
                safety=safety,
                topics=topic_summaries,
            ),
            self.report_path,
        )
        self.get_logger().info(
            f"integrated preflight observer completed: status={status.value} "
            f"report={self.report_path}"
        )
        return status


def main(args: list[str] | None = None) -> None:
    """지정된 정지 관찰 시간을 실행하고 결과에 따라 종료한다."""
    rclpy.init(args=args)
    node = IntegratedPreflightObserver()
    deadline = monotonic() + node.duration_seconds
    try:
        while rclpy.ok() and monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        status = node.finalize()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if status is CheckStatus.FAIL:
        raise SystemExit(2)
