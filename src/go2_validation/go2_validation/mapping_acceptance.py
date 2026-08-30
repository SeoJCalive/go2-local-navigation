
"""Todo 12 mapping runtime 관찰값을 ROS와 분리해 판정한다.
입력 message 수, SLAM service·TF owner, 저장 artifact와 process teardown을
불변 값으로 받아 합격 또는 실패 사유를 만든다. 이 모듈은 ROS graph를 열거나
파일을 생성하지 않는다.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Final

from go2_validation.mapping_cloud_accounting import (
    CLOUD_ACCOUNTING_EMIT_CADENCES,
    MappingCloudAccounting,
)
from go2_validation.mapping_scan_quality import (
    MappingScanQualityObservation,
    empty_mapping_scan_quality_observation,
)
from go2_validation.mapping_tf_continuity import (
    MAXIMUM_TRANSLATION_STEP_M,
    MAXIMUM_YAW_STEP_RAD,
    MappingTfContinuityObservation,
    empty_mapping_tf_continuity_observation,
)
from go2_validation.mapping_pose_continuity import MappingCorrectionContinuityObservation


EXPECTED_ARTIFACT_NAMES: Final = (
    "occupancy.pgm",
    "occupancy.yaml",
    "pose_graph.data",
    "pose_graph.posegraph",
)
MINIMUM_MAP_CORRECTION_SAMPLES: Final = 2
class MappingVariant(str, Enum):
    """서로 다른 provenance를 유지하는 Todo 12 replay 종류다."""

    PROJECT_STATIONARY = "project_stationary"
    EXTERNAL_DYNAMIC_SHORT = "external_dynamic_short"
    EXTERNAL_DYNAMIC_FULL = "external_dynamic_full"


class MappingStatus(str, Enum):
    """한 mapping variant의 폐쇄된 terminal 상태다."""

    PASSED = "passed"
    FAILED = "failed"


@dataclass(frozen=True, slots=True)
class MappingStreamObservation:
    """Bag 전체 소비와 SLAM input/output message 관찰값이다."""

    expected_cloud_count: int
    observed_cloud_count: int
    expected_odometry_count: int
    observed_odometry_count: int
    scan_count: int
    odom_count: int
    map_count: int
    map_frames: tuple[str, ...]
    map_has_cells: bool
    scan_quality: MappingScanQualityObservation = field(
        default_factory=empty_mapping_scan_quality_observation
    )


@dataclass(frozen=True, slots=True)
class MappingOwnershipObservation:
    """SLAM service, clock, global TF와 command graph 최대값이다."""

    slam_services_ready: bool
    clock_publisher_max: int
    clock_progressed: bool
    clock_stalled: bool
    global_edges: tuple[tuple[str, str], ...]
    global_owner_nodes: tuple[str, ...]
    command_publisher_max: int
    control_node_max: int


@dataclass(frozen=True, slots=True)
class MappingArtifactObservation:
    """Occupancy·pose graph 저장과 reload 결과다."""

    occupancy_saved: bool
    pose_graph_saved: bool
    pose_graph_reloaded: bool
    checksums: tuple[tuple[str, str], ...]


@dataclass(frozen=True, slots=True)
class MappingProcessObservation:
    """Owned player·launch와 teardown 이후 잔류 상태다."""

    player_exit_code: int
    launch_exit_code: int
    residual_nodes: tuple[str, ...]
    residual_processes: tuple[str, ...]
    teardown_clock_publishers: int
    teardown_global_owner_nodes: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class MappingObservation:
    """한 variant의 provenance와 네 acceptance 관찰 영역이다."""

    variant: MappingVariant
    provenance: str
    source_checksum: str
    replay_checksum: str
    streams: MappingStreamObservation
    ownership: MappingOwnershipObservation
    artifacts: MappingArtifactObservation
    process: MappingProcessObservation
    sensor_tf_profile: str
    continuity: MappingTfContinuityObservation
    map_correction_continuity: MappingCorrectionContinuityObservation
    scan_projection_profile: str = "raw_single"
    coarse_search_angle_offset: float = 0.349
    use_response_expansion: bool = True
    do_loop_closing: bool = True
    expected_intrinsic_untransformable_cloud_count: int = 0
    cloud_accounting: MappingCloudAccounting | None = None
    odometry_continuity: MappingTfContinuityObservation = field(
        default_factory=empty_mapping_tf_continuity_observation
    )


@dataclass(frozen=True, slots=True)
class MappingResult:
    """한 variant의 terminal verdict와 재현 가능한 실패 check다."""

    variant: MappingVariant
    status: MappingStatus
    provenance: str
    source_checksum: str
    replay_checksum: str
    sensor_tf_profile: str
    scan_projection_profile: str
    coarse_search_angle_offset: float
    use_response_expansion: bool
    do_loop_closing: bool
    expected_intrinsic_untransformable_cloud_count: int
    failed_checks: tuple[str, ...]


def assess_mapping(observation: MappingObservation) -> MappingResult:
    """모든 Todo 12 합격 조건을 독립 check 이름으로 판정한다."""
    streams = observation.streams
    ownership = observation.ownership
    artifacts = observation.artifacts
    process = observation.process
    artifact_names = tuple(name for name, _checksum in artifacts.checksums)
    checks = (
        ("cloud_count", streams.observed_cloud_count == streams.expected_cloud_count),
        (
            "odometry_count",
            streams.observed_odometry_count == streams.expected_odometry_count,
        ),
        ("scan_received", streams.scan_count > 0),
        ("odom_received", streams.odom_count > 0),
        ("map_received", streams.map_count > 0),
        ("map_frame", streams.map_frames == ("map",)),
        ("map_cells", streams.map_has_cells),
        ("slam_services", ownership.slam_services_ready),
        ("clock_owner", ownership.clock_publisher_max == 1),
        ("clock_progress", ownership.clock_progressed),
        ("clock_stall", not ownership.clock_stalled),
        ("global_tf_edge", ownership.global_edges == (("map", "odom"),)),
        ("global_tf_owner", ownership.global_owner_nodes == ("/slam_toolbox",)),
        (
            "map_correction_samples",
            observation.map_correction_continuity.sample_count
            >= MINIMUM_MAP_CORRECTION_SAMPLES,
        ),
        (
            "map_correction_translation_step",
            observation.map_correction_continuity.maximum_translation_step_m
            <= MAXIMUM_TRANSLATION_STEP_M,
        ),
        (
            "map_correction_yaw_step",
            observation.map_correction_continuity.maximum_yaw_step_rad
            <= MAXIMUM_YAW_STEP_RAD,
        ),
        (
            "map_correction_unaligned",
            observation.map_correction_continuity.unaligned_sample_count == 0,
        ),
        (
            "map_correction_stamp_regression",
            observation.map_correction_continuity.regressive_stamp_count == 0,
        ),
        ("command_publishers", ownership.command_publisher_max == 0),
        ("control_nodes", ownership.control_node_max == 0),
        ("occupancy_saved", artifacts.occupancy_saved),
        ("pose_graph_saved", artifacts.pose_graph_saved),
        ("pose_graph_reloaded", artifacts.pose_graph_reloaded),
        ("artifact_checksums", artifact_names == EXPECTED_ARTIFACT_NAMES),
        ("player_exit", process.player_exit_code == 0),
        ("launch_exit", process.launch_exit_code == 0),
        ("residual_nodes", not process.residual_nodes),
        ("residual_processes", not process.residual_processes),
        ("teardown_clock", process.teardown_clock_publishers == 0),
        ("teardown_global_tf", not process.teardown_global_owner_nodes),
        ("source_checksum", bool(observation.source_checksum)),
        ("replay_checksum", bool(observation.replay_checksum)),
        *_cloud_accounting_checks(observation),
    )
    failed = tuple(check_id for check_id, passed in checks if not passed)
    return MappingResult(
        variant=observation.variant,
        status=MappingStatus.PASSED if not failed else MappingStatus.FAILED,
        provenance=observation.provenance,
        source_checksum=observation.source_checksum,
        replay_checksum=observation.replay_checksum,
        sensor_tf_profile=observation.sensor_tf_profile,
        scan_projection_profile=observation.scan_projection_profile,
        coarse_search_angle_offset=observation.coarse_search_angle_offset,
        use_response_expansion=observation.use_response_expansion,
        do_loop_closing=observation.do_loop_closing,
        expected_intrinsic_untransformable_cloud_count=(
            observation.expected_intrinsic_untransformable_cloud_count
        ),
        failed_checks=failed,
    )


def _cloud_accounting_checks(
    observation: MappingObservation,
) -> tuple[tuple[str, bool], ...]:
    """누적 scan profile에만 terminal accumulator accounting gate를 더한다."""
    expected_emit_every = CLOUD_ACCOUNTING_EMIT_CADENCES.get(
        observation.scan_projection_profile
    )
    if expected_emit_every is None:
        return ()
    accounting = observation.cloud_accounting
    if accounting is None:
        return (("cloud_accounting", False),)
    expected_cloud_count = observation.streams.expected_cloud_count
    input_conserved = (
        accounting.processed
        + accounting.dropped_unrecoverable
        + accounting.dropped_overflow
        + accounting.pending_at_shutdown
        == accounting.received
    )
    output_conserved = (
        accounting.output_published * accounting.emit_every
        + accounting.partial_frames_not_emitted
        == accounting.processed
    )
    return (
        ("cloud_accounting_received", accounting.received == expected_cloud_count),
        ("cloud_accounting_input_conservation", input_conserved),
        ("cloud_accounting_output_conservation", output_conserved),
        ("cloud_accounting_emit_every", accounting.emit_every == expected_emit_every),
        (
            "cloud_accounting_dropped_unrecoverable",
            accounting.dropped_unrecoverable
            == observation.expected_intrinsic_untransformable_cloud_count,
        ),
        ("cloud_accounting_dropped_overflow", accounting.dropped_overflow == 0),
        (
            "cloud_accounting_pending_at_shutdown",
            accounting.pending_at_shutdown == 0,
        ),
        (
            "cloud_accounting_output_stamp_regression",
            accounting.output_stamp_regression_count == 0,
        ),
        (
            "cloud_accounting_future_recovery",
            accounting.pending_at_shutdown != 0
            or accounting.future_waited == accounting.recovered_after_retry,
        ),
    )
