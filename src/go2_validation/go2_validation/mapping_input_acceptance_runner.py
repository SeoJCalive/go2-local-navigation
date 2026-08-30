
"""Pure verdict boundary for the domain-62 mapping-input acceptance runner.
The ROS process owner supplies one immutable observation per sequential variant.
This module only decides the contract outcome, so Todo 6 observer helpers and
Todo 9's conversion-result reader can attach without duplicating ROS graph code.
"""

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Final

DOMAIN_ID: Final = 62
LASER_SCAN_TYPE: Final = "sensor_msgs/msg/LaserScan"
BASE_FRAME: Final = "base"


class MappingInputVariant(str, Enum):
    PROJECT_STATIONARY = "project_stationary"
    EXTERNAL_DYNAMIC_SHORT = "external_dynamic_short"


class ExternalReplayStatus(str, Enum):
    PASSED = "passed"
    DEFERRED = "deferred"
    CONFLICT = "conflict"


class MappingInputStatus(str, Enum):
    PASSED = "passed"
    DEFERRED = "deferred"
    CONFLICT = "conflict"
    FAILED = "failed"


@dataclass(frozen=True, slots=True)
class ExternalShortReplayBoundary:
    status: str
    provenance: str
    short_bag_path: str | None
    source_checksum: str | None


@dataclass(frozen=True, slots=True)
class ExternalShortReplay:
    status: ExternalReplayStatus
    provenance: str
    short_bag_path: Path | None
    source_checksum: str | None


@dataclass(frozen=True, slots=True)
class ExternalReplayBoundaryError(Exception):
    field_name: str
    value: str | None

    def __str__(self) -> str:
        return f"invalid external short replay {self.field_name}: {self.value!r}"


@dataclass(frozen=True, slots=True)
class MappingInputRunSpec:
    variant: MappingInputVariant
    launch_files: tuple[str, ...]
    domain_id: int = DOMAIN_ID
    loopback_only: bool = True
    use_sim_time: bool = True
    playback_rate: float = 1.0
    player_owns_clock: bool = True
    readiness_required: bool = True
    global_map_to_odom_owners: int = 0


@dataclass(frozen=True, slots=True)
class MappingInputObservation:
    variant: MappingInputVariant
    scan_message_type: str
    scan_frame_id: str
    scan_stamps_monotonic: bool
    scan_ranges_finite_or_infinite: bool
    scan_minimum_rate_met: bool
    clock_publishers: int
    global_map_to_odom_owners: int
    command_publishers: int
    domain_id: int
    loopback_only: bool
    odom_overlaps_scan_clock: bool
    source_checksum: str = "test-source-checksum"


@dataclass(frozen=True, slots=True)
class MappingInputResult:
    variant: MappingInputVariant
    status: MappingInputStatus
    provenance: str
    source_checksum: str | None
    failed_checks: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class MappingInputSummary:
    overall_status: MappingInputStatus
    variants: tuple[MappingInputResult, ...]


def parse_external_short_replay(
    boundary: ExternalShortReplayBoundary,
) -> ExternalShortReplay:
    if boundary.provenance != "external_dynamic":
        raise ExternalReplayBoundaryError("provenance", boundary.provenance)
    match boundary.status:
        case "passed":
            if boundary.short_bag_path is None:
                raise ExternalReplayBoundaryError("short_bag_path", None)
            if boundary.source_checksum is None:
                raise ExternalReplayBoundaryError("source_checksum", None)
            return ExternalShortReplay(
                status=ExternalReplayStatus.PASSED,
                provenance=boundary.provenance,
                short_bag_path=Path(boundary.short_bag_path),
                source_checksum=boundary.source_checksum,
            )
        case "deferred":
            return ExternalShortReplay(
                status=ExternalReplayStatus.DEFERRED,
                provenance=boundary.provenance,
                short_bag_path=None,
                source_checksum=None,
            )
        case "conflict":
            return ExternalShortReplay(
                status=ExternalReplayStatus.CONFLICT,
                provenance=boundary.provenance,
                short_bag_path=None,
                source_checksum=None,
            )
        case invalid_status:
            raise ExternalReplayBoundaryError("status", invalid_status)


def build_run_specs(
    external_replay: ExternalShortReplay,
) -> tuple[MappingInputRunSpec, ...]:
    stationary = MappingInputRunSpec(
        variant=MappingInputVariant.PROJECT_STATIONARY,
        launch_files=("go2_mapping_scan.launch.py",),
    )
    if external_replay.status is not ExternalReplayStatus.PASSED:
        return (stationary,)
    return (
        stationary,
        MappingInputRunSpec(
            variant=MappingInputVariant.EXTERNAL_DYNAMIC_SHORT,
            launch_files=(
                "go2_mapping_scan.launch.py",
                "go2_odometry_adapter.launch.py",
            ),
        ),
    )


def assess_mapping_input(
    observation: MappingInputObservation,
    external_status: ExternalReplayStatus,
) -> MappingInputResult:
    match observation.variant:
        case MappingInputVariant.PROJECT_STATIONARY:
            return _result_for_observation(observation)
        case MappingInputVariant.EXTERNAL_DYNAMIC_SHORT:
            match external_status:
                case ExternalReplayStatus.DEFERRED:
                    return MappingInputResult(
                        variant=observation.variant,
                        status=MappingInputStatus.DEFERRED,
                        provenance="external_dynamic",
                        source_checksum=None,
                        failed_checks=(),
                    )
                case ExternalReplayStatus.CONFLICT:
                    return MappingInputResult(
                        variant=observation.variant,
                        status=MappingInputStatus.CONFLICT,
                        provenance="external_dynamic",
                        source_checksum=observation.source_checksum,
                        failed_checks=("external_conversion_conflict",),
                    )
                case ExternalReplayStatus.PASSED:
                    return _result_for_observation(observation)
                case unreachable:
                    from typing import assert_never

                    assert_never(unreachable)
        case unreachable:
            from typing import assert_never

            assert_never(unreachable)


def summarize_variants(
    variants: tuple[MappingInputResult, ...],
) -> MappingInputSummary:
    statuses = tuple(result.status for result in variants)
    if MappingInputStatus.CONFLICT in statuses or MappingInputStatus.FAILED in statuses:
        return MappingInputSummary(MappingInputStatus.FAILED, variants)
    stationary_passed = any(
        result.variant is MappingInputVariant.PROJECT_STATIONARY
        and result.status is MappingInputStatus.PASSED
        for result in variants
    )
    status = (
        MappingInputStatus.PASSED if stationary_passed else MappingInputStatus.FAILED
    )
    return MappingInputSummary(status, variants)


def _result_for_observation(
    observation: MappingInputObservation,
) -> MappingInputResult:
    failed_checks = _failed_checks(observation)
    return MappingInputResult(
        variant=observation.variant,
        status=MappingInputStatus.PASSED
        if not failed_checks
        else MappingInputStatus.FAILED,
        provenance=(
            "project_stationary"
            if observation.variant is MappingInputVariant.PROJECT_STATIONARY
            else "external_dynamic"
        ),
        source_checksum=observation.source_checksum,
        failed_checks=failed_checks,
    )


def _failed_checks(observation: MappingInputObservation) -> tuple[str, ...]:
    checks = (
        ("scan_message_type", observation.scan_message_type == LASER_SCAN_TYPE),
        ("scan_frame", observation.scan_frame_id == BASE_FRAME),
        ("scan_stamp_monotonic", observation.scan_stamps_monotonic),
        ("scan_ranges", observation.scan_ranges_finite_or_infinite),
        ("scan_minimum_rate", observation.scan_minimum_rate_met),
        ("clock_owner", observation.clock_publishers == 1),
        ("global_map_to_odom_owner", observation.global_map_to_odom_owners == 0),
        ("command_publishers", observation.command_publishers == 0),
        ("domain", observation.domain_id == DOMAIN_ID),
        ("loopback", observation.loopback_only),
        ("source_checksum", bool(observation.source_checksum)),
        (
            "odom_clock_overlap",
            observation.variant is MappingInputVariant.PROJECT_STATIONARY
            or observation.odom_overlaps_scan_clock,
        ),
    )
    return tuple(check_name for check_name, passed in checks if not passed)


def main(args: list[str] | None = None) -> None:
    """Delegate ROS process ownership while this module keeps the pure verdict API."""
    from go2_validation.mapping_input_runtime import main as runtime_main

    runtime_main(args)
