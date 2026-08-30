
"""검증된 raw DDS MCAP을 canonical short·full rosbag2 fixture로 변환한다."""
from dataclasses import dataclass
from pathlib import Path
import tempfile
from typing import NoReturn

from go2_validation.external_replay_acquisition import verify_file
from go2_validation.external_replay_acquisition_runner import AcquisitionResult
from go2_validation.external_replay_contract import (
    AcquisitionConflict,
    ContractConflict,
    validate_counts,
)
from go2_validation.external_replay_converter import (
    RosCdrCanonicalizer,
    convert_messages,
    enforce_determinism,
    enforce_output_cap,
    output_tree_checksum,
    output_tree_size,
    promote_output,
)
from go2_validation.external_replay_manifest import ConversionSpec
from go2_validation.external_replay_rosbag import (
    Rosbag2CanonicalWriter,
    iter_selected_messages,
)
from go2_validation.external_replay_scan import ExternalSourceScan, scan_external_source
from go2_validation.external_replay_conversion_result import (
    ConversionFailure,
    ConversionResult,
    InventoryRow,
    failed_conversion,
)
from go2_validation.external_replay_window import (
    ShortWindow,
    choose_short_window,
    short_window_candidates,
)


@dataclass(frozen=True, slots=True)
class ConversionEvidence:
    source_path: Path
    source_checksum: str
    output_root: Path
    scan: ExternalSourceScan
    candidates: tuple[ShortWindow, ...]
    selected: ShortWindow
    short_checksum: str
    repeat_checksum: str
    short_size: int
    full_checksum: str
    full_size: int


def convert_external_replay(
    spec: ConversionSpec,
    acquisition: AcquisitionResult,
    output_root: Path,
) -> ConversionResult:
    """Acquisition 상태를 보존하고 두 output을 한 directory로 atomic 승격한다."""
    match acquisition.status:
        case "deferred":
            return failed_conversion(
                acquisition,
                ConversionFailure("deferred", acquisition.reason_code, acquisition.detail),
            )
        case "conflict":
            return failed_conversion(
                acquisition,
                ConversionFailure("conflict", acquisition.reason_code, acquisition.detail),
            )
        case "passed":
            pass
        case unreachable:
            _assert_never(unreachable)
    try:
        return _convert_passed(spec, acquisition, output_root)
    except (AcquisitionConflict, ContractConflict) as error:
        return failed_conversion(
            acquisition,
            ConversionFailure("conflict", error.reason, error.detail or None),
        )
    except OSError as error:
        return failed_conversion(
            acquisition,
            ConversionFailure("conflict", "derived_local_io_failure", str(error)),
        )


def _convert_passed(
    spec: ConversionSpec,
    acquisition: AcquisitionResult,
    output_root: Path,
) -> ConversionResult:
    source_path, source_checksum = _validated_source(spec, acquisition, output_root)
    scan = scan_external_source(source_path)
    cloud_count = len(scan.cloud_log_times_ns)
    odometry_count = len(scan.odometry)
    if (cloud_count, odometry_count) != (
        spec.expected_cloud_count,
        spec.expected_odometry_count,
    ):
        raise ContractConflict(
            "selected_count_mismatch",
            f"cloud={cloud_count},odometry={odometry_count}",
        )
    validate_counts(cloud_count, odometry_count)
    candidates = short_window_candidates(
        scan.odometry,
        scan.cloud_log_times_ns,
        scan.interval_start_ns,
        scan.interval_end_ns,
        spec.short_minimum_cloud_count,
        spec.short_minimum_odometry_count,
    )
    selected = choose_short_window(
        scan.odometry,
        scan.cloud_log_times_ns,
        scan.interval_start_ns,
        scan.interval_end_ns,
        spec.short_minimum_cloud_count,
        spec.short_minimum_odometry_count,
    )
    stage_root = output_root.parent / "staging"
    stage_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="convert-", dir=stage_root) as raw:
        workspace = Path(raw)
        bundle = workspace / "derived"
        bundle.mkdir()
        short_path = bundle / "short"
        short_manifest = _convert_window(source_path, short_path, selected)
        short_checksum = output_tree_checksum(short_path)
        repeat_path = workspace / "repeat" / "short"
        repeat_manifest = _convert_window(source_path, repeat_path, selected)
        repeat_checksum = output_tree_checksum(repeat_path)
        enforce_determinism(
            short_manifest,
            short_checksum,
            repeat_manifest,
            repeat_checksum,
        )
        full_path = bundle / "full"
        full_manifest = _convert_window(source_path, full_path, None)
        short_counts = _manifest_counts(short_manifest)
        full_counts = _manifest_counts(full_manifest)
        if short_counts != (selected.cloud_count, selected.odometry_count):
            raise ContractConflict("short_output_count_mismatch")
        if full_counts != (cloud_count, odometry_count):
            raise ContractConflict("full_output_count_mismatch")
        short_size = output_tree_size(short_path)
        full_size = output_tree_size(full_path)
        enforce_output_cap(full_size, spec.full_output_cap_bytes)
        full_checksum = output_tree_checksum(full_path)
        promote_output(
            bundle,
            output_root,
            spec.full_output_cap_bytes + short_size,
        )
    return _passed_result(
        acquisition,
        ConversionEvidence(
            source_path=source_path,
            source_checksum=source_checksum,
            output_root=output_root,
            scan=scan,
            candidates=candidates,
            selected=selected,
            short_checksum=short_checksum,
            repeat_checksum=repeat_checksum,
            short_size=short_size,
            full_checksum=full_checksum,
            full_size=full_size,
        ),
    )


def _convert_window(
    source_path: Path,
    destination: Path,
    window: ShortWindow | None,
) -> tuple[str, ...]:
    start_ns = None if window is None else window.start_ns
    end_ns = None if window is None else window.end_ns
    return convert_messages(
        iter_selected_messages(source_path, start_ns, end_ns),
        RosCdrCanonicalizer(),
        Rosbag2CanonicalWriter(destination),
    )


def _validated_source(
    spec: ConversionSpec,
    acquisition: AcquisitionResult,
    output_root: Path,
) -> tuple[Path, str]:
    if acquisition.source_id != spec.source.source_id:
        raise ContractConflict("acquisition_source_id_mismatch")
    if acquisition.extracted_path is None or acquisition.extracted_sha256 is None:
        raise ContractConflict("acquisition_artifact_fields_absent")
    path = Path(acquisition.extracted_path)
    if path.resolve().parent != (output_root.parent / "source").resolve():
        raise ContractConflict("acquisition_source_outside_custody", str(path))
    digest = verify_file(
        path,
        spec.source.extracted_size_bytes,
        acquisition.extracted_sha256,
    )
    return path, digest


def _manifest_counts(manifest: tuple[str, ...]) -> tuple[int, int]:
    cloud = sum(":/utlidar/cloud:" in row for row in manifest)
    odometry = sum(":/utlidar/robot_odom:" in row for row in manifest)
    return cloud, odometry


def _inventory(scan: ExternalSourceScan) -> tuple[InventoryRow, ...]:
    counts = dict(scan.inventory.channel_counts)
    return tuple(
        InventoryRow(channel.topic, channel.schema, counts.get(channel.topic, 0))
        for channel in scan.inventory.channels
    )


def _passed_result(
    acquisition: AcquisitionResult,
    evidence: ConversionEvidence,
) -> ConversionResult:
    return ConversionResult(
        status="passed",
        source_id=acquisition.source_id,
        provenance="external_dynamic",
        reason_code=None,
        detail=None,
        artifact_absent=False,
        source_path=str(evidence.source_path),
        source_checksum=evidence.source_checksum,
        interval_start_ns=evidence.scan.interval_start_ns,
        interval_end_ns=evidence.scan.interval_end_ns,
        inventory=_inventory(evidence.scan),
        cloud_frames=evidence.scan.cloud_frames,
        odometry_frames=evidence.scan.odometry_frames,
        odometry_child_frames=evidence.scan.odometry_child_frames,
        candidate_windows=evidence.candidates,
        selected_window=evidence.selected,
        short_bag_path=str(evidence.output_root / "short"),
        short_checksum=evidence.short_checksum,
        short_repeat_checksum=evidence.repeat_checksum,
        short_size_bytes=evidence.short_size,
        short_cloud_count=evidence.selected.cloud_count,
        short_odometry_count=evidence.selected.odometry_count,
        full_bag_path=str(evidence.output_root / "full"),
        full_checksum=evidence.full_checksum,
        full_size_bytes=evidence.full_size,
        full_cloud_count=len(evidence.scan.cloud_log_times_ns),
        full_odometry_count=len(evidence.scan.odometry),
    )


def _assert_never(value: NoReturn) -> NoReturn:
    raise AssertionError(f"unreachable acquisition status: {value!r}")
