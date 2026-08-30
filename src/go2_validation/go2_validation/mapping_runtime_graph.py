
"""Mapping observer가 공유하는 QoS와 ROS graph path helper다."""
from typing import Final, Protocol

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


RAW_CLOUD_QOS: Final = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)
RAW_ODOMETRY_QOS: Final = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1000,
    durability=DurabilityPolicy.VOLATILE,
)
STREAM_QOS: Final = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    durability=DurabilityPolicy.VOLATILE,
)
MAP_QOS: Final = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
LOCAL_TF_OWNER_NODES: Final = frozenset(
    {"/go2_odometry_adapter", "/robot_state_publisher"}
)
RUNTIME_NODES: Final = frozenset(
    {
        "/base_to_utlidar_lidar_static_tf",
        "/go2_mapping_cloud_gate",
        "/go2_mapping_cloud_accumulator",
        "/go2_odometry_adapter",
        "/pointcloud_to_laserscan",
        "/robot_state_publisher",
        "/rosbag2_player",
        "/slam_toolbox",
    }
)


class GraphEndpoint(Protocol):
    """ROS graph endpoint가 노출하는 node identity다."""

    node_name: str
    node_namespace: str


class MappingGraphNode(Protocol):
    """Mapping graph inspection에 필요한 Node의 최소 surface다."""

    def get_node_names_and_namespaces(self) -> list[tuple[str, str]]: ...

    def get_publishers_info_by_topic(self, topic_name: str) -> list[GraphEndpoint]: ...


def endpoint_path(name: str, namespace: str) -> str:
    """ROS name과 namespace를 canonical absolute node path로 결합한다."""
    return f"/{name}" if namespace == "/" else f"{namespace.rstrip('/')}/{name}"


def node_paths(node: MappingGraphNode) -> set[str]:
    """현재 graph node names를 canonical path set으로 투영한다."""
    return {
        endpoint_path(name, namespace)
        for name, namespace in node.get_node_names_and_namespaces()
    }


def global_tf_owner_nodes(node: MappingGraphNode) -> set[str]:
    """Local TF owner를 제외한 `/tf` publisher node를 반환한다."""
    return {
        path
        for endpoint in node.get_publishers_info_by_topic("/tf")
        if (path := endpoint_path(endpoint.node_name, endpoint.node_namespace))
        not in LOCAL_TF_OWNER_NODES
    }
