from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField

from go2_perception.obstacle_candidate_node import ObstacleCandidateNode
from go2_perception.perception_contract import SOURCE_FRAME_ID


class _Logger:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def warning(self, message: str) -> None:
        self.warnings.append(message)


class _TransformBuffer:
    def lookup_transform(self, *_args) -> TransformStamped:
        transform = TransformStamped()
        transform.transform.rotation.w = 1.0
        return transform


class _Publisher:
    def __init__(self) -> None:
        self.messages: list[PointCloud2] = []

    def publish(self, message: PointCloud2) -> None:
        self.messages.append(message)


class _ObstacleNodeHarness:
    def __init__(self) -> None:
        self._tf_buffer = _TransformBuffer()
        self._publisher = _Publisher()
        self._logger = _Logger()

    def get_logger(self) -> _Logger:
        return self._logger


def test_given_malformed_cloud_when_received_then_candidate_node_suppresses_without_crashing() -> None:
    # Given: the same zero-point-step layout observed in the runtime fault log.
    message = PointCloud2()
    message.header.frame_id = SOURCE_FRAME_ID
    message.height = 1
    message.width = 1
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.point_step = 0
    message.row_step = 0
    harness = _ObstacleNodeHarness()

    # When: the real callback reaches sensor_msgs_py decoding.
    ObstacleCandidateNode._on_cloud(harness, message)

    # Then: no candidate is published and the malformed sample remains observable.
    assert harness._publisher.messages == []
    assert len(harness._logger.warnings) == 1
    assert "malformed" in harness._logger.warnings[0]
