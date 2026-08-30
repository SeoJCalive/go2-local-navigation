
"""Stationary와 external-short mapping ingress를 순차 실행하고 JSON을 기록한다."""
from dataclasses import asdict
from datetime import datetime
import json
import os
from pathlib import Path

from bringup.mode_observer import ExecutionMode, ModeEnvironment, assess_mode_environment
from bringup.preflight_result import write_document
from bringup.preflight_types import CheckStatus
from go2_validation.external_replay_converter import output_tree_checksum
from go2_validation.mapping_input_acceptance_runner import (
    ExternalReplayBoundaryError,
    ExternalReplayStatus,
    ExternalShortReplay,
    ExternalShortReplayBoundary,
    MappingInputResult,
    MappingInputStatus,
    MappingInputVariant,
    parse_external_short_replay,
    summarize_variants,
)
from go2_validation.mapping_input_execution import (
    MappingRuntimeError,
    MappingVariantExecution,
    run_mapping_variant,
)


def read_external_short_replay(path: Path) -> ExternalShortReplay:
    """Todo 9 result에서 status·provenance·short path·source hash만 파싱한다."""
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except OSError as error:
        raise ExternalReplayBoundaryError("result_path", str(path)) from error
    except json.JSONDecodeError as error:
        raise ExternalReplayBoundaryError("result_json", str(error)) from error
    if not isinstance(document, dict):
        raise ExternalReplayBoundaryError("result_root", None)
    return parse_external_short_replay(
        ExternalShortReplayBoundary(
            status=_optional_string(document.get("status")) or "",
            provenance=_optional_string(document.get("provenance")) or "",
            short_bag_path=_optional_string(document.get("short_bag_path")),
            source_checksum=_optional_string(document.get("source_checksum")),
        )
    )


def _optional_string(value) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str):
        raise ExternalReplayBoundaryError("result_field", str(value))
    return value


def _environment_is_valid() -> bool:
    check = assess_mode_environment(
        ExecutionMode.SCAN_REPLAY,
        ModeEnvironment(
            rmw_implementation=os.environ.get("RMW_IMPLEMENTATION", ""),
            ros_domain_id=os.environ.get("ROS_DOMAIN_ID", ""),
            go2_interface=os.environ.get("GO2_AGX_INTERFACE", ""),
            cyclonedds_uri=os.environ.get("CYCLONEDDS_URI", ""),
        ),
    )
    return check.status is CheckStatus.PASS and 'name="lo"' in os.environ.get(
        "CYCLONEDDS_URI", ""
    )


def _external_result(replay: ExternalShortReplay) -> MappingInputResult:
    match replay.status:
        case ExternalReplayStatus.DEFERRED:
            return MappingInputResult(
                MappingInputVariant.EXTERNAL_DYNAMIC_SHORT,
                MappingInputStatus.DEFERRED,
                replay.provenance,
                None,
                (),
            )
        case ExternalReplayStatus.CONFLICT:
            return MappingInputResult(
                MappingInputVariant.EXTERNAL_DYNAMIC_SHORT,
                MappingInputStatus.CONFLICT,
                replay.provenance,
                None,
                ("external_conversion_conflict",),
            )
        case ExternalReplayStatus.PASSED:
            raise MappingRuntimeError("external_passed_without_execution")


def execute_mapping_inputs(
    project_root: Path,
    stationary_bag: Path,
    external_replay: ExternalShortReplay,
    log_root: Path,
) -> tuple[MappingVariantExecution, ...] | tuple[MappingVariantExecution, MappingInputResult]:
    """Stationary를 항상 먼저 실행하고 available external short만 이어서 실행한다."""
    stationary = run_mapping_variant(
        MappingInputVariant.PROJECT_STATIONARY,
        stationary_bag,
        output_tree_checksum(stationary_bag),
        log_root,
    )
    if external_replay.status is not ExternalReplayStatus.PASSED:
        return (stationary, _external_result(external_replay))
    if external_replay.short_bag_path is None or external_replay.source_checksum is None:
        raise MappingRuntimeError("external_passed_fields_absent")
    external_path = external_replay.short_bag_path
    if not external_path.is_absolute():
        external_path = project_root / external_path
    external = run_mapping_variant(
        MappingInputVariant.EXTERNAL_DYNAMIC_SHORT,
        external_path,
        external_replay.source_checksum,
        log_root,
    )
    return (stationary, external)


def _document(rows, output_path: Path) -> tuple[dict, MappingInputStatus]:
    results = tuple(row.result if isinstance(row, MappingVariantExecution) else row for row in rows)
    summary = summarize_variants(results)
    variants = []
    for row in rows:
        if isinstance(row, MappingVariantExecution):
            variants.append(
                {
                    "result": asdict(row.result),
                    "observation": asdict(row.observation),
                    "player_exit_code": row.player_exit_code,
                    "launch_exit_codes": list(row.launch_exit_codes),
                    "residual_nodes": list(row.residual_nodes),
                    "residual_processes": list(row.residual_processes),
                    "logs": [str(path) for path in row.log_paths],
                }
            )
        else:
            variants.append({"result": asdict(row), "observation": None})
    return (
        {
            "schema_version": 1,
            "record_kind": "mapping_input_acceptance_result",
            "recorded_at": datetime.now().astimezone().isoformat(),
            "overall": summary.overall_status.value,
            "domain_id": 62,
            "loopback_only": True,
            "physical_motion": False,
            "command_publication": False,
            "output_path": str(output_path),
            "variants": variants,
        },
        summary.overall_status,
    )


def main(args: list[str] | None = None) -> None:
    """ROS parameter를 읽고 readiness-first replay와 atomic report를 실행한다."""
    from ament_index_python.packages import get_package_prefix
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=args)
    node = Node("go2_mapping_input_acceptance_runner")
    project_root = Path(get_package_prefix("go2_validation")).parents[1]
    external_path = Path(
        str(
            node.declare_parameter(
                "external_manifest",
                str(project_root / "data/external/dimos_go2_indoor/runs/conversion.json"),
            ).value
        )
    )
    output_path = Path(
        str(
            node.declare_parameter(
                "output_path",
                str(project_root / "data/runs/mapping_input/stage12-ingress.json"),
            ).value
        )
    )
    stationary_bag = Path(
        str(
            node.declare_parameter(
                "stationary_bag",
                str(project_root / "data/bags/go2_stationary_raw_20260826_1829"),
            ).value
        )
    )
    exit_code = 2
    try:
        if not _environment_is_valid():
            raise MappingRuntimeError("mapping_environment_mismatch")
        rows = execute_mapping_inputs(
            project_root,
            stationary_bag,
            read_external_short_replay(external_path),
            output_path.with_suffix(""),
        )
        document, status = _document(rows, output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        write_document(document, output_path)
        exit_code = 0 if status is MappingInputStatus.PASSED else 2
    except (
        ExternalReplayBoundaryError,
        MappingRuntimeError,
        OSError,
        ValueError,
    ) as error:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        write_document(
            {
                "schema_version": 1,
                "record_kind": "mapping_input_acceptance_result",
                "recorded_at": datetime.now().astimezone().isoformat(),
                "overall": "failed",
                "domain_id": 62,
                "reason_code": str(error),
            },
            output_path,
        )
        node.get_logger().error(f"mapping input acceptance failed: {error}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
