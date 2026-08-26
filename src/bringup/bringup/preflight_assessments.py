"""환경·graph·TF·motion 안전 관찰값을 독립 check로 판정한다."""

import os
from pathlib import Path
from typing import Protocol

from ament_index_python.packages import get_package_prefix

from bringup.preflight_configuration import RUNTIME_WRAPPER_PATH
from bringup.preflight_report import (
    EnvironmentObservation,
    GraphObservation,
    SafetyObservation,
    TransformObservation,
)
from bringup.preflight_types import CheckResult, CheckStatus


class PublisherEndpoint(Protocol):
    """command publisher owner 판정에 필요한 ROS endpoint 필드다."""

    node_name: str
    node_namespace: str


def project_publisher_count(
    endpoints: tuple[PublisherEndpoint, ...],
    project_nodes: tuple[str, ...],
) -> int:
    """Bare DDS endpoint를 제외하고 통합 stack node의 publisher만 센다."""
    owners = (
        (
            f"/{endpoint.node_name}"
            if endpoint.node_namespace == "/"
            else f"{endpoint.node_namespace.rstrip('/')}/{endpoint.node_name}"
        )
        for endpoint in endpoints
    )
    return sum(owner in project_nodes for owner in owners)


def collect_environment() -> tuple[EnvironmentObservation, CheckResult]:
    """환경값과 Ethernet 상태를 읽어 wrapper 적용 증거를 반환한다."""
    interface = os.environ.get("GO2_AGX_INTERFACE", "")
    operstate_path = Path(f"/sys/class/net/{interface}/operstate")
    operstate = (
        operstate_path.read_text(encoding="utf-8").strip()
        if operstate_path.is_file()
        else "missing"
    )
    observation = EnvironmentObservation(
        rmw_implementation=os.environ.get("RMW_IMPLEMENTATION", ""),
        ros_domain_id=os.environ.get("ROS_DOMAIN_ID", ""),
        go2_interface=interface,
        cyclonedds_uri=os.environ.get("CYCLONEDDS_URI", ""),
        interface_operstate=operstate,
        bringup_prefix=get_package_prefix("bringup"),
        runtime_wrapper_exists=RUNTIME_WRAPPER_PATH.is_file(),
    )
    passed = (
        observation.rmw_implementation == "rmw_cyclonedds_cpp"
        and observation.ros_domain_id == "0"
        and observation.go2_interface == "eno1"
        and 'name="eno1"' in observation.cyclonedds_uri
        and observation.interface_operstate == "up"
        and observation.bringup_prefix.endswith(
            "/go2_local_navigation/install/bringup"
        )
        and observation.runtime_wrapper_exists
    )
    return observation, CheckResult(
        check_id="environment.runtime_wrapper_overlay",
        status=CheckStatus.PASS if passed else CheckStatus.FAIL,
        detail=(
            f"rmw={observation.rmw_implementation}; "
            f"domain={observation.ros_domain_id}; "
            f"interface={observation.go2_interface}; "
            f"operstate={observation.interface_operstate}; "
            f"bringup_prefix={observation.bringup_prefix}; "
            f"wrapper_exists={observation.runtime_wrapper_exists}"
        ),
    )


def assess_graph(observation: GraphObservation) -> CheckResult:
    """필수 node가 startup 뒤 모두 관찰되고 소실되지 않았는지 판정한다."""
    missing = tuple(
        node
        for node in observation.expected_nodes
        if node not in observation.seen_nodes
    )
    passed = not missing and not observation.lost_nodes
    return CheckResult(
        check_id="graph.required_project_nodes",
        status=CheckStatus.PASS if passed else CheckStatus.FAIL,
        detail=f"missing={missing}; lost={observation.lost_nodes}",
    )


def assess_transforms(
    observations: tuple[TransformObservation, ...],
) -> tuple[CheckResult, ...]:
    """각 필수 TF 연결을 별도 check로 보존한다."""
    return tuple(
        CheckResult(
            check_id=(
                f"tf.{observation.parent_frame}_to_"
                f"{observation.child_frame}"
            ),
            status=(
                CheckStatus.PASS
                if observation.available
                else CheckStatus.FAIL
            ),
            detail=f"available={observation.available}",
        )
        for observation in observations
    )


def assess_safety(
    observation: SafetyObservation,
) -> tuple[CheckResult, ...]:
    """두 motion gate와 프로젝트 command publisher 부재를 강제 판정한다."""
    gates_closed = (
        observation.output_enabled is False
        and observation.physical_validation_approved is False
    )
    publishers_absent = (
        observation.sport_request_max_publishers == 0
        and observation.lowcmd_max_publishers == 0
    )
    return (
        CheckResult(
            check_id="safety.motion_gates_closed",
            status=CheckStatus.PASS if gates_closed else CheckStatus.FAIL,
            detail=(
                f"output_enabled={observation.output_enabled}; "
                "physical_validation_approved="
                f"{observation.physical_validation_approved}"
            ),
        ),
        CheckResult(
            check_id="safety.command_publishers_absent",
            status=(
                CheckStatus.PASS
                if publishers_absent
                else CheckStatus.FAIL
            ),
            detail=(
                "sport_request_max_project_publishers="
                f"{observation.sport_request_max_publishers}; "
                "lowcmd_max_project_publishers="
                f"{observation.lowcmd_max_publishers}"
            ),
        ),
    )
