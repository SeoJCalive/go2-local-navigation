from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from go2_validation.external_replay_contract import (
    Channel,
    ContractConflict,
    OdometrySemantic,
    PointCloudSemantic,
    canonical_channels,
    validate_counts,
    validate_semantic_round_trip,
)
from go2_validation.external_replay_window import (
    MessageRecord,
    choose_short_window,
    short_window_candidates,
)
from go2_validation.external_replay_converter import enforce_determinism, enforce_output_cap
from go2_validation.external_replay_converter import (
    CanonicalMessage,
    RawMessage,
    RosCdrCanonicalizer,
    convert_messages,
    output_tree_checksum,
)
from go2_validation.external_replay_acquisition_runner import AcquisitionResult
from go2_validation.external_replay_conversion_runner import convert_external_replay
from go2_validation.external_replay_manifest import load_conversion_spec
from go2_validation.external_replay_rosbag import (
    QOS_PROFILES,
    Rosbag2CanonicalWriter,
    inspect_source_inventory,
    iter_selected_messages,
)
from go2_validation.external_replay_scan import scan_external_source


PROJECT_ROOT = Path(__file__).parents[3]
CONFIG_PATH = PROJECT_ROOT / "src/go2_validation/config/external_replay_sources.yaml"


def _odom(second: int, x: float, sequence: int) -> MessageRecord:
    return MessageRecord(
        channel="rt/utlidar/robot_odom",
        log_time_ns=second * 1_000_000_000,
        sequence=sequence,
        planar_xy=(x, 0.0),
    )


def test_canonical_channels_reject_duplicate_selected_channel() -> None:
    # Given: duplicate source channels for one selected DDS topic.
    channels = (
        Channel(1, "rt/utlidar/cloud", "sensor_msgs::msg::dds_::PointCloud2_"),
        Channel(2, "rt/utlidar/cloud", "sensor_msgs::msg::dds_::PointCloud2_"),
        Channel(3, "rt/utlidar/robot_odom", "nav_msgs::msg::dds_::Odometry_"),
    )

    # When: selected channels are canonicalized.
    with pytest.raises(ContractConflict) as raised:
        canonical_channels(channels)

    # Then: aliases cannot silently select one source.
    assert raised.value.reason == "duplicate_selected_channel"


def test_canonical_channels_reject_wrong_schema() -> None:
    # Given: both names exist but one schema is incompatible.
    channels = (
        Channel(1, "rt/utlidar/cloud", "wrong"),
        Channel(2, "rt/utlidar/robot_odom", "nav_msgs::msg::dds_::Odometry_"),
    )

    # When: schema identity is checked.
    with pytest.raises(ContractConflict) as raised:
        canonical_channels(channels)

    # Then: no output mapping is returned.
    assert raised.value.reason == "selected_schema_mismatch"


def test_short_window_selects_highest_score_then_earliest() -> None:
    # Given: two valid 120-second candidates with equal rounded path scores.
    messages = tuple(
        _odom(second, float(second % 2), second) for second in range(241)
    )
    cloud_times = tuple(second * 1_000_000_000 for second in range(241))

    # When: selection evaluates deterministic integer-second windows.
    selected = choose_short_window(
        messages,
        cloud_times,
        interval_start_ns=0,
        interval_end_ns=241_000_000_000,
        minimum_cloud_count=120,
        minimum_odometry_count=120,
    )

    # Then: tie-break chooses the earliest start.
    assert selected.start_ns == 0
    assert selected.end_ns == 120_000_000_000
    assert selected.path_score == 119.0


def test_short_window_candidate_manifest_preserves_all_eligible_starts() -> None:
    messages = tuple(_odom(second, float(second), second) for second in range(121))
    cloud_times = tuple(second * 1_000_000_000 for second in range(121))

    candidates = short_window_candidates(
        messages,
        cloud_times,
        interval_start_ns=0,
        interval_end_ns=121_000_000_000,
        minimum_cloud_count=120,
        minimum_odometry_count=120,
    )

    assert tuple(item.start_ns for item in candidates) == (0, 1_000_000_000)
    assert all(item.cloud_count == 120 for item in candidates)
    assert all(item.odometry_count == 120 for item in candidates)


def test_semantic_round_trip_rejects_field_mutation() -> None:
    # Given: a PointCloud2 semantic snapshot and a changed data payload.
    original = PointCloudSemantic(
        header=(10, "utlidar_lidar"),
        height=1,
        width=1,
        fields=(("x", 0, 7, 1),),
        is_bigendian=False,
        point_step=4,
        row_step=4,
        data=b"abcd",
        is_dense=True,
    )

    # When: deserialize/serialize/deserialze semantics are compared.
    with pytest.raises(ContractConflict) as raised:
        validate_semantic_round_trip(original, replace(original, data=b"abce"))

    # Then: any field mutation is a conflict.
    assert raised.value.reason == "cdr_semantic_mismatch"


def test_odometry_round_trip_accepts_all_equal_fields() -> None:
    # Given: a complete odometry semantic snapshot.
    semantic = OdometrySemantic(
        header=(12, "odom"),
        child_frame_id="base_link",
        pose=(1.0,) * 7,
        pose_covariance=(0.0,) * 36,
        twist=(2.0,) * 6,
        twist_covariance=(0.0,) * 36,
    )

    # When/Then: equality across every represented field is accepted.
    validate_semantic_round_trip(semantic, semantic)


def test_count_mismatch_is_conflict() -> None:
    # Given: counts that differ from pinned full-source inventory.
    # When: full counts are checked.
    with pytest.raises(ContractConflict) as raised:
        validate_counts(cloud_count=17_775, odometry_count=173_616)

    # Then: the external lane cannot be promoted.
    assert raised.value.reason == "selected_count_mismatch"


def test_determinism_rejects_order_or_checksum_difference() -> None:
    # Given: repeated short conversions with different ordered manifests.
    # When: repeatability is enforced.
    with pytest.raises(ContractConflict) as raised:
        enforce_determinism(("a", "b"), "hash", ("b", "a"), "hash")

    # Then: nondeterminism blocks promotion.
    assert raised.value.reason == "short_conversion_nondeterministic"


def test_output_cap_rejects_oversize_output() -> None:
    # Given: a full output one byte over the configured cap.
    # When: promotion checks its size.
    with pytest.raises(ContractConflict) as raised:
        enforce_output_cap(3_864_397_507, 3_864_397_506)

    # Then: oversize output is a conflict.
    assert raised.value.reason == "full_output_byte_cap_exceeded"


class _TimestampMutatingCanonicalizer:
    def canonicalize(self, message: RawMessage) -> CanonicalMessage:
        return CanonicalMessage(
            output_topic="/utlidar/cloud",
            output_type="sensor_msgs/msg/PointCloud2",
            log_time_ns=message.log_time_ns + 1,
            sequence=message.sequence,
            payload=message.payload,
        )


class _Writer:
    def write(self, message: CanonicalMessage) -> None:
        del message

    def finish(self) -> None:
        return None


class _IdentityCanonicalizer:
    def canonicalize(self, message: RawMessage) -> CanonicalMessage:
        return CanonicalMessage(
            output_topic="/utlidar/cloud",
            output_type="sensor_msgs/msg/PointCloud2",
            log_time_ns=message.log_time_ns,
            sequence=message.sequence,
            payload=message.payload,
        )


def test_conversion_rejects_log_timestamp_mutation() -> None:
    # Given: one selected source message and a faulty CDR adapter.
    messages = (RawMessage("rt/utlidar/cloud", 10, 1, b"cdr"),)

    # When: canonicalization changes the source log timestamp.
    with pytest.raises(ContractConflict) as raised:
        convert_messages(messages, _TimestampMutatingCanonicalizer(), _Writer())

    # Then: no writer output can be accepted as canonical.
    assert raised.value.reason == "source_log_timestamp_mutated"


def test_conversion_rejects_nondeterministic_source_order() -> None:
    # Given: source messages arrive outside the total ordering contract.
    messages = (
        RawMessage("rt/utlidar/cloud", 11, 1, b"second"),
        RawMessage("rt/utlidar/cloud", 10, 1, b"first"),
    )

    # When: conversion checks source order before writing.
    with pytest.raises(ContractConflict) as raised:
        convert_messages(messages, _IdentityCanonicalizer(), _Writer())

    # Then: input nondeterminism is a conflict.
    assert raised.value.reason == "source_message_order_nondeterministic"


def test_mcap_crc_corruption_is_conflict(tmp_path: Path) -> None:
    # Given: an MCAP with both selected channels and one corrupted chunk payload.
    mcap_writer = pytest.importorskip("mcap.writer")
    CompressionType = mcap_writer.CompressionType
    Writer = mcap_writer.Writer
    path = tmp_path / "source.mcap"
    with path.open("wb") as output:
        writer = Writer(output, compression=CompressionType.NONE, enable_data_crcs=True)
        writer.start()
        cloud_schema = writer.register_schema(
            "sensor_msgs::msg::dds_::PointCloud2_", "", b""
        )
        odom_schema = writer.register_schema(
            "nav_msgs::msg::dds_::Odometry_", "", b""
        )
        cloud = writer.register_channel("rt/utlidar/cloud", "cdr", cloud_schema)
        odom = writer.register_channel("rt/utlidar/robot_odom", "cdr", odom_schema)
        writer.add_message(cloud, 1, b"cloud-payload", 1, 1)
        writer.add_message(odom, 2, b"odom-payload", 2, 1)
        writer.finish()
    path.write_bytes(path.read_bytes().replace(b"cloud-payload", b"cloue-payload"))

    # When: inventory reads every chunk with CRC validation.
    with pytest.raises(ContractConflict) as raised:
        inspect_source_inventory(path)

    # Then: corrupted source data cannot enter conversion.
    assert raised.value.reason == "mcap_crc_or_format_failure"


def test_rosbag2_adapter_preserves_canonical_topics_and_log_timestamps(
    tmp_path: Path,
) -> None:
    pytest.importorskip("mcap")
    rosbag2_py = pytest.importorskip("rosbag2_py")
    serialization = pytest.importorskip("rclpy.serialization")
    point_cloud_module = pytest.importorskip("sensor_msgs.msg")
    odometry_module = pytest.importorskip("nav_msgs.msg")
    source_path = tmp_path / "raw"
    source_writer = rosbag2_py.SequentialWriter()
    source_writer.open(
        rosbag2_py.StorageOptions(uri=str(source_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    source_writer.create_topic(
        rosbag2_py.TopicMetadata(
            name="rt/utlidar/cloud",
            type="sensor_msgs::msg::dds_::PointCloud2_",
            serialization_format="cdr",
        )
    )
    source_writer.create_topic(
        rosbag2_py.TopicMetadata(
            name="rt/utlidar/robot_odom",
            type="nav_msgs::msg::dds_::Odometry_",
            serialization_format="cdr",
        )
    )
    cloud = point_cloud_module.PointCloud2()
    cloud.header.frame_id = "utlidar_lidar"
    cloud.height = 1
    cloud.width = 0
    odometry = odometry_module.Odometry()
    odometry.header.frame_id = "odom"
    odometry.child_frame_id = "base"
    source_writer.write(
        "rt/utlidar/cloud",
        serialization.serialize_message(cloud),
        10,
    )
    source_writer.write(
        "rt/utlidar/robot_odom",
        serialization.serialize_message(odometry),
        20,
    )
    source_writer.close()

    inventory = inspect_source_inventory(source_path)
    source_scan = scan_external_source(source_path)
    messages = tuple(iter_selected_messages(source_path, None, None))
    destination = tmp_path / "canonical"
    output_writer = Rosbag2CanonicalWriter(destination)
    manifest = convert_messages(messages, RosCdrCanonicalizer(), output_writer)
    repeat_root = tmp_path / "repeat"
    repeat_root.mkdir()
    repeat_destination = repeat_root / "canonical"
    repeat_manifest = convert_messages(
        iter_selected_messages(source_path, None, None),
        RosCdrCanonicalizer(),
        Rosbag2CanonicalWriter(repeat_destination),
    )

    assert inventory.channel_counts == (
        ("rt/utlidar/cloud", 1),
        ("rt/utlidar/robot_odom", 1),
    )
    assert source_scan.interval_start_ns == 10
    assert source_scan.interval_end_ns == 21
    assert source_scan.cloud_log_times_ns == (10,)
    assert source_scan.odometry[0].planar_xy == (0.0, 0.0)
    assert len(manifest) == 2
    assert repeat_manifest == manifest
    assert output_tree_checksum(repeat_destination) == output_tree_checksum(destination)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(destination), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_metadata = reader.get_all_topics_and_types()
    assert tuple(
        sorted((item.name, item.type) for item in topic_metadata)
    ) == (
        ("/utlidar/cloud", "sensor_msgs/msg/PointCloud2"),
        ("/utlidar/robot_odom", "nav_msgs/msg/Odometry"),
    )
    assert (reader.read_next()[0], reader.read_next()[0]) == (
        "/utlidar/cloud",
        "/utlidar/robot_odom",
    )
    qos_profiles = {item.name: item.offered_qos_profiles for item in topic_metadata}
    assert all("reliability: 1" in value for value in qos_profiles.values())
    assert all("depth: 1" in value for value in qos_profiles.values())
    assert all("durability: 2" in value for value in qos_profiles.values())


def test_deferred_acquisition_produces_explicit_absent_conversion(
    tmp_path: Path,
) -> None:
    acquisition = AcquisitionResult(
        status="deferred",
        source_id="dimos_go2_indoor",
        reason_code="network_prerequisite_absent",
        detail="dns",
        artifact_absent=True,
        archive_path=None,
        archive_size_bytes=None,
        archive_sha256=None,
        extracted_path=None,
        extracted_size_bytes=None,
        extracted_sha256=None,
    )

    result = convert_external_replay(
        load_conversion_spec(CONFIG_PATH),
        acquisition,
        tmp_path / "derived",
    )

    assert result.status == "deferred"
    assert result.artifact_absent
    assert result.short_bag_path is None
    assert result.full_bag_path is None
    assert not (tmp_path / "derived").exists()


def test_given_canonical_qos_when_parsed_by_humble_then_all_rmw_fields_exist() -> None:
    # Given: the QoS YAML embedded in canonical topic metadata.
    required_fields = (
        "history:",
        "depth:",
        "reliability:",
        "durability:",
        "deadline:",
        "lifespan:",
        "liveliness:",
        "liveliness_lease_duration:",
        "avoid_ros_namespace_conventions:",
    )

    # When: Humble's required rmw profile keys are checked.
    missing = tuple(field for field in required_fields if field not in QOS_PROFILES)

    # Then: rosbag2 player will not reject an incomplete profile node.
    assert missing == ()
