from pathlib import Path
import json
import struct
from typing import Final

from geometry_msgs.msg import TransformStamped
import pytest
from sensor_msgs.msg import PointField
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


PACKAGE_ROOT: Final = Path(__file__).parents[1]
CONFIG_PATH: Final = PACKAGE_ROOT / "config/mapping_scan.yaml"


def _cloud(stamp_nanoseconds: int, x_values: tuple[float, ...]) -> PointCloud2:
    header = Header()
    header.frame_id = "odom"
    header.stamp.sec = stamp_nanoseconds // 1_000_000_000
    header.stamp.nanosec = stamp_nanoseconds % 1_000_000_000
    return point_cloud2.create_cloud_xyz32(
        header,
        [(x_value, 0.0, 0.0) for x_value in x_values],
    )


def _padded_mixed_cloud() -> PointCloud2:
    header = Header()
    header.frame_id = "utlidar_lidar"
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.UINT16, count=1),
    ]
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = 1
    cloud.fields = fields
    cloud.is_bigendian = False
    cloud.point_step = 32
    cloud.row_step = 32
    cloud.data = struct.pack("<fffH18x", 1.0, 2.0, 3.0, 7)
    cloud.is_dense = True
    return cloud


def test_given_two_odom_clouds_when_window_is_two_then_latest_output_contains_both() -> None:
    from go2_perception.mapping_cloud_accumulator import MappingCloudWindow

    window = MappingCloudWindow(frame_limit=2, target_frame="odom")

    first = window.add(_cloud(1_000_000_000, (1.0, 2.0)))
    second = window.add(_cloud(2_000_000_000, (3.0,)))

    assert first.width == 2
    assert second.width == 3
    assert second.header.frame_id == "odom"
    assert second.header.stamp.sec == 2
    points = point_cloud2.read_points_list(
        second,
        field_names=("x", "y", "z"),
        skip_nans=True,
    )
    assert tuple(point.x for point in points) == (1.0, 2.0, 3.0)


def test_given_three_clouds_when_window_is_two_then_oldest_cloud_is_evicted() -> None:
    from go2_perception.mapping_cloud_accumulator import MappingCloudWindow

    window = MappingCloudWindow(frame_limit=2, target_frame="odom")
    window.add(_cloud(1_000_000_000, (1.0,)))
    window.add(_cloud(2_000_000_000, (2.0,)))

    output = window.add(_cloud(3_000_000_000, (3.0,)))

    points = point_cloud2.read_points_list(
        output,
        field_names=("x", "y", "z"),
        skip_nans=True,
    )
    assert tuple(point.x for point in points) == (2.0, 3.0)


def test_given_sliding_ten_frame_profile_when_clouds_arrive_then_every_input_emits_latest_window() -> None:
    from go2_perception.mapping_cloud_accumulator import MappingCloudWindow

    # Given: 기존 DimOS sliding profile과 같은 10-frame, every-input cadence
    window = MappingCloudWindow(frame_limit=10, target_frame="odom", emit_every=1)

    # When: 12개 source-order cloud를 처리한다.
    outputs = tuple(
        window.add(_cloud(index, (float(index),))) for index in range(1, 13)
    )

    # Then: 모든 입력이 출력되고 11·12번째는 최신 10 frame만 포함한다.
    assert all(output is not None for output in outputs)
    assert tuple(output.width for output in outputs if output is not None) == (
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 10
    )
    final_points = point_cloud2.read_points_list(
        outputs[-1], field_names=("x", "y", "z"), skip_nans=True
    )
    assert tuple(point.x for point in final_points) == tuple(
        float(index) for index in range(3, 13)
    )


def test_given_emit10_profile_when_twenty_clouds_arrive_then_batches_do_not_overlap() -> None:
    from go2_perception.mapping_cloud_accumulator import MappingCloudWindow

    # Given: 10개 처리마다 한 번 내보내는 replay-only batch
    window = MappingCloudWindow(frame_limit=10, target_frame="odom", emit_every=10)

    # When: source-order cloud 20개를 처리한다.
    outputs = tuple(
        window.add(_cloud(index, (float(index),))) for index in range(1, 21)
    )

    # Then: 1..9와 11..19는 출력하지 않고 두 batch는 서로 겹치지 않는다.
    assert all(output is None for output in outputs[:9])
    assert all(output is None for output in outputs[10:19])
    emitted = (outputs[9], outputs[19])
    assert all(output is not None for output in emitted)
    emitted_points = tuple(
        tuple(
            point.x
            for point in point_cloud2.read_points_list(
                output, field_names=("x", "y", "z"), skip_nans=True
            )
        )
        for output in emitted
        if output is not None
    )
    assert emitted_points == (
        tuple(float(index) for index in range(1, 11)),
        tuple(float(index) for index in range(11, 21)),
    )
    assert window.partial_frame_count == 0


def test_given_padded_mixed_cloud_when_xyz_is_compacted_then_tf_transform_succeeds() -> (
    None
):
    source = _padded_mixed_cloud()
    transform = TransformStamped()
    transform.header.frame_id = "odom"
    transform.transform.translation.x = 1.0
    transform.transform.rotation.w = 1.0

    with pytest.raises(AssertionError):
        do_transform_cloud(source, transform)

    from go2_perception.mapping_cloud_accumulator import compact_xyz_cloud

    transformed = do_transform_cloud(compact_xyz_cloud(source), transform)

    assert tuple(field.name for field in transformed.fields) == ("x", "y", "z")
    assert transformed.point_step == 12
    points = point_cloud2.read_points_list(
        transformed,
        field_names=("x", "y", "z"),
        skip_nans=True,
    )
    assert tuple(points[0]) == (2.0, 2.0, 3.0)


def test_given_mapping_profiles_when_loaded_then_raw_and_dimos_accumulation_are_separate() -> None:
    from go2_perception.mapping_scan_profiles import load_mapping_scan_profile

    raw = load_mapping_scan_profile(CONFIG_PATH, "raw_single", "onboard")
    accumulated = load_mapping_scan_profile(
        CONFIG_PATH,
        "dimos_odom_accumulated",
        "external_replay",
    )
    emit10 = load_mapping_scan_profile(
        CONFIG_PATH,
        "dimos_odom_accumulated_emit10",
        "external_replay",
    )

    assert raw.accumulator_enabled is False
    assert raw.frame_limit == 1
    assert raw.converter_input_topic == "/go2_mapping/cloud_validated"
    assert accumulated.accumulator_enabled is True
    assert accumulated.frame_limit == 10
    assert accumulated.emit_every == 1
    assert accumulated.input_qos_depth == 64
    assert accumulated.retry_queue_capacity == 64
    assert accumulated.accumulator_target_frame == "odom"
    assert accumulated.converter_input_topic == "/go2_mapping/cloud_accumulated"
    assert accumulated.scope == "dimos_external_replay_only"
    assert accumulated.source_commit == (
        "b4cd9789cc68876adf87ff40a404677f127d69bf"
    )
    assert emit10.frame_limit == 10
    assert emit10.emit_every == 10
    assert emit10.input_qos_depth == 64
    assert emit10.retry_queue_capacity == 64
    assert emit10.source_commit == accumulated.source_commit


def test_given_package_metadata_when_read_then_accumulator_node_is_installable() -> None:
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    package_source = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")

    assert "mapping_cloud_accumulator_node.py" in tuple(
        path.name for path in (PACKAGE_ROOT / "go2_perception").iterdir()
    )
    assert '"mapping_cloud_accumulator = "' in setup_source
    assert '"go2_perception.mapping_cloud_accumulator_node:main"' in setup_source
    assert "<exec_depend>tf2_sensor_msgs</exec_depend>" in package_source


def test_given_future_head_when_transform_becomes_available_then_it_recovers() -> None:
    from go2_perception.mapping_cloud_accumulator import MappingCloudRetryQueue

    # Given
    queue = MappingCloudRetryQueue[str](capacity=2)
    queue.enqueue("future-cloud", 10)

    # When
    queue.mark_head_future_waited()
    processed = queue.take_head_for_processing()

    # Then
    assert processed is not None
    assert processed.cloud == "future-cloud"
    assert queue.accounting().future_waited == 1
    assert queue.accounting().recovered_after_retry == 1
    assert queue.accounting().processed == 1


def test_given_future_head_when_later_cloud_arrives_then_source_order_is_preserved() -> None:
    from go2_perception.mapping_cloud_accumulator import MappingCloudRetryQueue

    # Given
    queue = MappingCloudRetryQueue[str](capacity=3)
    queue.enqueue("first", 10)
    queue.enqueue("second", 20)

    # When
    queue.mark_head_future_waited()

    # Then
    assert queue.head() is not None
    assert queue.head().cloud == "first"
    assert queue.take_head_for_processing().cloud == "first"
    assert queue.take_head_for_processing().cloud == "second"


def test_given_full_queue_when_new_cloud_arrives_then_overflow_is_accounted() -> None:
    from go2_perception.mapping_cloud_accumulator import MappingCloudRetryQueue

    # Given
    queue = MappingCloudRetryQueue[str](capacity=1)
    queue.enqueue("kept", 10)

    # When
    result = queue.enqueue("overflow", 20)

    # Then
    assert result.drop_reason == "queue_capacity_exceeded"
    assert queue.head().cloud == "kept"
    assert queue.accounting().dropped_overflow == 1


def test_given_unrecoverable_head_when_dropped_then_it_cannot_publish() -> None:
    from go2_perception.mapping_cloud_accumulator import MappingCloudRetryQueue

    # Given
    queue = MappingCloudRetryQueue[str](capacity=1)
    queue.enqueue("bad-transform", 10)

    # When
    dropped = queue.drop_head_unrecoverable()

    # Then
    assert dropped is not None
    assert queue.head() is None
    assert queue.accounting().dropped_unrecoverable == 1
    assert queue.accounting().processed == 0


def test_given_reordered_or_duplicate_source_when_enqueued_then_output_stamps_stay_monotonic() -> None:
    from go2_perception.mapping_cloud_accumulator import MappingCloudRetryQueue

    # Given
    queue = MappingCloudRetryQueue[str](capacity=3)
    queue.enqueue("newest", 20)
    first_output = queue.take_head_for_processing()

    # When
    duplicate = queue.enqueue("duplicate", 20)
    reordered = queue.enqueue("older", 10)

    # Then
    assert first_output is not None
    assert first_output.source_stamp_nanoseconds == 20
    assert duplicate.drop_reason == "source_stamp_not_increasing"
    assert reordered.drop_reason == "source_stamp_not_increasing"
    assert queue.take_head_for_processing() is None
    assert queue.accounting().output_stamp_regression_count == 0


def test_given_terminal_accounting_when_formatted_then_prefix_and_compact_json_are_stable() -> None:
    from go2_perception.mapping_cloud_accumulator import (
        MAPPING_CLOUD_ACCOUNTING_PREFIX,
        MappingCloudRetryQueue,
        format_mapping_cloud_accounting,
    )

    # Given
    queue = MappingCloudRetryQueue[str](capacity=1)
    queue.enqueue("pending", 10)

    # When
    line = format_mapping_cloud_accounting(queue.accounting())
    payload = json.loads(line.removeprefix(MAPPING_CLOUD_ACCOUNTING_PREFIX))

    # Then
    assert line.startswith("MAPPING_CLOUD_ACCOUNTING {")
    assert " " not in line.removeprefix(MAPPING_CLOUD_ACCOUNTING_PREFIX)
    assert set(payload) == {
        "received",
        "future_waited",
        "recovered_after_retry",
        "processed",
        "output_published",
        "dropped_unrecoverable",
        "dropped_overflow",
        "pending_at_shutdown",
        "partial_frames_not_emitted",
        "emit_every",
        "output_stamp_regression_count",
    }
    assert "published" not in payload
    assert payload["pending_at_shutdown"] == 1
