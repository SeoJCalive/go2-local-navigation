
"""SLAM Toolbox save·serialize·deserialize service 호출을 소유한다.
모든 호출은 같은 observer node에서 bounded wall time으로 완료하며 occupancy와 pose
graph 파일이 검증된 뒤에만 runtime execution에 반환한다.
"""

from dataclasses import dataclass
from pathlib import Path
from time import monotonic

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import DeserializePoseGraph, SaveMap, SerializePoseGraph

from go2_validation.mapping_artifacts import (
    SavedMappingArtifacts,
    normalize_occupancy_image_reference,
    validate_saved_mapping_artifacts,
)


@dataclass(frozen=True, slots=True)
class SlamServiceError(Exception):
    """SLAM lifecycle 또는 저장 service가 bounded contract를 완료하지 못했다."""

    reason_code: str
    detail: str = ""

    def __str__(self) -> str:
        return f"{self.reason_code}: {self.detail}" if self.detail else self.reason_code


class SlamServiceClient:
    """한 MappingRuntimeObserver에 연결된 세 SLAM service client 집합이다."""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._save = node.create_client(SaveMap, "/slam_toolbox/save_map")
        self._serialize = node.create_client(
            SerializePoseGraph,
            "/slam_toolbox/serialize_map",
        )
        self._deserialize = node.create_client(
            DeserializePoseGraph,
            "/slam_toolbox/deserialize_map",
        )

    def wait_until_ready(self, timeout_seconds: float) -> bool:
        """세 service가 하나의 wall-time deadline 안에 발견되는지 확인한다."""
        deadline = monotonic() + timeout_seconds
        for client in (
            self._save,
            self._serialize,
            self._deserialize,
        ):
            remaining = deadline - monotonic()
            if remaining <= 0 or not client.wait_for_service(timeout_sec=remaining):
                return False
        return True

    def save_serialize_reload(
        self,
        artifact_root: Path,
        timeout_seconds: float,
    ) -> SavedMappingArtifacts:
        """Occupancy와 pose graph를 저장·검증하고 pose graph를 다시 읽는다."""
        occupancy_prefix = artifact_root / "occupancy"
        pose_graph_prefix = artifact_root / "pose_graph"
        save_request = SaveMap.Request()
        save_request.name.data = str(occupancy_prefix)
        save_future = self._save.call_async(save_request)
        if not self._wait_future(save_future, timeout_seconds):
            raise SlamServiceError("slam_save_map_timeout")
        save_response = save_future.result()
        if save_response is None or save_response.result != SaveMap.Response.RESULT_SUCCESS:
            result = None if save_response is None else save_response.result
            raise SlamServiceError("slam_save_map_failed", str(result))

        serialize_request = SerializePoseGraph.Request()
        serialize_request.filename = str(pose_graph_prefix)
        serialize_future = self._serialize.call_async(serialize_request)
        if not self._wait_future(serialize_future, timeout_seconds):
            raise SlamServiceError("slam_serialize_timeout")
        serialize_response = serialize_future.result()
        if (
            serialize_response is None
            or serialize_response.result
            != SerializePoseGraph.Response.RESULT_SUCCESS
        ):
            result = None if serialize_response is None else serialize_response.result
            raise SlamServiceError("slam_serialize_failed", str(result))

        normalize_occupancy_image_reference(artifact_root)
        artifacts = validate_saved_mapping_artifacts(artifact_root)
        deserialize_request = DeserializePoseGraph.Request()
        deserialize_request.filename = str(pose_graph_prefix)
        deserialize_request.match_type = DeserializePoseGraph.Request.START_AT_FIRST_NODE
        deserialize_future = self._deserialize.call_async(deserialize_request)
        if not self._wait_future(deserialize_future, timeout_seconds):
            raise SlamServiceError("slam_deserialize_timeout")
        if deserialize_future.result() is None:
            raise SlamServiceError("slam_deserialize_response_absent")
        return artifacts

    def _wait_future(self, future, timeout_seconds: float) -> bool:
        deadline = monotonic() + timeout_seconds
        while rclpy.ok() and monotonic() < deadline and not future.done():
            rclpy.spin_once(self._node, timeout_sec=0.1)
        return future.done()
