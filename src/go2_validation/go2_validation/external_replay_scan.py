
"""Raw DDS MCAP의 CDR·시간 구간·frame·평면 odometry를 한 번에 조사한다."""
from dataclasses import dataclass
from pathlib import Path

from go2_validation.external_replay_contract import (
    CLOUD_SOURCE,
    ODOMETRY_SOURCE,
    ContractConflict,
)
from go2_validation.external_replay_converter import RosCdrCanonicalizer, SourceInventory
from go2_validation.external_replay_rosbag import (
    inspect_source_inventory,
    iter_selected_messages,
)
from go2_validation.external_replay_window import MessageRecord


@dataclass(frozen=True, slots=True)
class ExternalSourceScan:
    """후보 선택과 provenance 결과에 필요한 source 관찰값이다."""

    inventory: SourceInventory
    cloud_log_times_ns: tuple[int, ...]
    odometry: tuple[MessageRecord, ...]
    interval_start_ns: int
    interval_end_ns: int
    cloud_frames: tuple[str, ...]
    odometry_frames: tuple[str, ...]
    odometry_child_frames: tuple[str, ...]


def scan_external_source(path: Path) -> ExternalSourceScan:
    """Storage 전체 읽기와 target Humble CDR round-trip을 모두 통과시킨다."""
    try:
        from nav_msgs.msg import Odometry
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2
    except ImportError as error:
        raise ContractConflict("ros_typesupport_unavailable", str(error)) from error
    inventory = inspect_source_inventory(path)
    canonicalizer = RosCdrCanonicalizer()
    cloud_times: list[int] = []
    odometry_records: list[MessageRecord] = []
    cloud_frames: set[str] = set()
    odometry_frames: set[str] = set()
    child_frames: set[str] = set()
    selected_times: list[int] = []
    for raw in iter_selected_messages(path, None, None):
        canonical = canonicalizer.canonicalize(raw)
        selected_times.append(raw.log_time_ns)
        match raw.source_topic:
            case "rt/utlidar/cloud":
                message = deserialize_message(canonical.payload, PointCloud2)
                cloud_times.append(raw.log_time_ns)
                cloud_frames.add(message.header.frame_id)
            case "rt/utlidar/robot_odom":
                message = deserialize_message(canonical.payload, Odometry)
                odometry_frames.add(message.header.frame_id)
                child_frames.add(message.child_frame_id)
                odometry_records.append(
                    MessageRecord(
                        channel=ODOMETRY_SOURCE,
                        log_time_ns=raw.log_time_ns,
                        sequence=raw.sequence,
                        planar_xy=(
                            message.pose.pose.position.x,
                            message.pose.pose.position.y,
                        ),
                    )
                )
            case unexpected:
                raise ContractConflict("unselected_source_topic", unexpected)
    if not selected_times:
        raise ContractConflict("selected_message_absent")
    count_by_topic = dict(inventory.channel_counts)
    if count_by_topic.get(CLOUD_SOURCE, 0) != len(cloud_times):
        raise ContractConflict("cloud_inventory_count_mismatch")
    if count_by_topic.get(ODOMETRY_SOURCE, 0) != len(odometry_records):
        raise ContractConflict("odometry_inventory_count_mismatch")
    return ExternalSourceScan(
        inventory=inventory,
        cloud_log_times_ns=tuple(cloud_times),
        odometry=tuple(odometry_records),
        interval_start_ns=min(selected_times),
        interval_end_ns=max(selected_times) + 1,
        cloud_frames=tuple(sorted(cloud_frames)),
        odometry_frames=tuple(sorted(odometry_frames)),
        odometry_child_frames=tuple(sorted(child_frames)),
    )
