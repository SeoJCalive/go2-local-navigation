
"""Stationary와 external-full mapping을 순차 실행해 Todo 12 JSON을 기록한다.
Domain 63·loopback 전제, input checksum·metadata와 variant 순서를 소유한다. 실제
launch·service lifecycle은 mapping_runtime_execution에 위임한다.
"""

from dataclasses import asdict
from datetime import datetime
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node

from bringup.mode_observer import ExecutionMode, ModeEnvironment, assess_mode_environment
from bringup.preflight_result import JsonDocument, write_document
from bringup.preflight_types import CheckStatus
from go2_validation.external_replay_converter import output_tree_checksum
from go2_validation.mapping_acceptance import MappingStatus, MappingVariant
from go2_validation.mapping_artifacts import MappingArtifactError
from go2_validation.mapping_runtime_data import (
    ExternalFullStatus,
    MappingRuntimeDataError,
    assert_external_metadata_matches,
    read_bag_expectation,
    read_external_full_replay,
)
from go2_validation.mapping_runtime_execution import (
    MappingRuntimeError,
    MappingVariantExecution,
    MappingVariantSpec,
    run_mapping_variant,
)
from go2_validation.mapping_slam_services import SlamServiceError
from go2_validation.typing_compat import assert_never


def _environment_is_valid() -> bool:
    check = assess_mode_environment(
        ExecutionMode.MAPPING,
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


def _execution_document(execution: MappingVariantExecution) -> JsonDocument:
    return {
        "result": asdict(execution.result),
        "observation": asdict(execution.observation),
        "artifact_paths": [str(path) for path in execution.artifact_paths],
        "log_paths": [str(path) for path in execution.log_paths],
        "physical_execution": False,
        "ground_truth": False,
    }


def _write_execution(
    execution: MappingVariantExecution,
    run_directory: Path,
) -> None:
    document = {
        "schema_version": 1,
        "record_kind": "mapping_variant_acceptance_result",
        "recorded_at": datetime.now().astimezone().isoformat(),
        **_execution_document(execution),
    }
    write_document(document, run_directory / "result.json")


def execute_mapping_acceptance(
    project_root: Path,
    stationary_bag: Path,
    conversion_result: Path,
    output_root: Path,
) -> tuple[MappingVariantExecution, ...] | tuple[MappingVariantExecution, dict]:
    """Stationary를 먼저 통과시킨 뒤 available external-full을 실행한다."""
    stationary_expectation = read_bag_expectation(stationary_bag)
    stationary_checksum = output_tree_checksum(stationary_bag)
    stationary = run_mapping_variant(
        MappingVariantSpec(
            variant=MappingVariant.PROJECT_STATIONARY,
            bag_path=stationary_bag,
            provenance="project_stationary",
            source_checksum=stationary_checksum,
            replay_checksum=stationary_checksum,
            expectation=stationary_expectation,
            sensor_tf_profile="project_default",
            execution_mode="onboard",
            continuity_profile="replay_enforce",
        ),
        output_root / MappingVariant.PROJECT_STATIONARY.value,
    )
    _write_execution(
        stationary,
        output_root / MappingVariant.PROJECT_STATIONARY.value,
    )
    if stationary.result.status is not MappingStatus.PASSED:
        return (stationary,)
    external = read_external_full_replay(conversion_result)
    match external.status:
        case ExternalFullStatus.DEFERRED:
            return (
                stationary,
                {
                    "variant": MappingVariant.EXTERNAL_DYNAMIC_FULL.value,
                    "status": "deferred",
                    "provenance": "external_dynamic",
                    "artifact_absent": True,
                },
            )
        case ExternalFullStatus.CONFLICT:
            raise MappingRuntimeDataError(
                "external_conversion_conflict",
                str(conversion_result),
            )
        case ExternalFullStatus.PASSED:
            if (
                external.bag_path is None
                or external.source_checksum is None
                or external.replay_checksum is None
            ):
                raise MappingRuntimeDataError(
                    "external_passed_fields_absent",
                    str(conversion_result),
                )
            external_path = external.bag_path
            if not external_path.is_absolute():
                external_path = project_root / external_path
            external_expectation = read_bag_expectation(external_path)
            assert_external_metadata_matches(external, external_expectation)
            actual_checksum = output_tree_checksum(external_path)
            if actual_checksum != external.replay_checksum:
                raise MappingRuntimeDataError(
                    "external_replay_checksum_mismatch",
                    actual_checksum,
                )
            execution = run_mapping_variant(
                MappingVariantSpec(
                    variant=MappingVariant.EXTERNAL_DYNAMIC_FULL,
                    bag_path=external_path,
                    provenance=external.provenance,
                    source_checksum=external.source_checksum,
                    replay_checksum=external.replay_checksum,
                    expectation=external_expectation,
                    sensor_tf_profile="dimos_replay",
                    execution_mode="external_replay",
                    continuity_profile="replay_enforce",
                ),
                output_root / MappingVariant.EXTERNAL_DYNAMIC_FULL.value,
            )
            _write_execution(
                execution,
                output_root / MappingVariant.EXTERNAL_DYNAMIC_FULL.value,
            )
            return (stationary, execution)
        case unreachable:
            assert_never(unreachable)


def _summary_document(rows, output_path: Path) -> tuple[JsonDocument, bool]:
    variants = []
    passed = True
    for row in rows:
        if isinstance(row, MappingVariantExecution):
            variants.append(_execution_document(row))
            passed = passed and row.result.status is MappingStatus.PASSED
        else:
            variants.append(row)
            passed = passed and row.get("status") == "deferred"
    return (
        {
            "schema_version": 1,
            "record_kind": "mapping_acceptance_result",
            "recorded_at": datetime.now().astimezone().isoformat(),
            "overall": "passed" if passed else "failed",
            "domain_id": 63,
            "loopback_only": True,
            "playback_rate": 1.0,
            "physical_execution": False,
            "command_publication": False,
            "output_path": str(output_path),
            "variants": variants,
        },
        passed,
    )


def main(args: list[str] | None = None) -> None:
    """ROS parameter를 읽고 Todo 12 runtime과 atomic summary를 실행한다."""
    rclpy.init(args=args)
    node = Node("go2_mapping_acceptance_runner")
    project_root = Path(get_package_prefix("go2_validation")).parents[1]
    output_root = Path(
        str(
            node.declare_parameter(
                "output_root",
                str(project_root / "data/runs/mapping"),
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
    conversion_result = Path(
        str(
            node.declare_parameter(
                "external_manifest",
                str(
                    project_root
                    / "data/external/dimos_go2_indoor/runs/conversion.json"
                ),
            ).value
        )
    )
    output_path = output_root / "stage12.json"
    exit_code = 2
    try:
        if not _environment_is_valid():
            raise MappingRuntimeError("mapping_environment_mismatch")
        output_root.mkdir(parents=True, exist_ok=True)
        rows = execute_mapping_acceptance(
            project_root,
            stationary_bag,
            conversion_result,
            output_root,
        )
        document, passed = _summary_document(rows, output_path)
        write_document(document, output_path)
        exit_code = 0 if passed else 2
    except (
        MappingArtifactError,
        MappingRuntimeDataError,
        MappingRuntimeError,
        SlamServiceError,
        OSError,
        ValueError,
    ) as error:
        output_root.mkdir(parents=True, exist_ok=True)
        write_document(
            {
                "schema_version": 1,
                "record_kind": "mapping_acceptance_result",
                "recorded_at": datetime.now().astimezone().isoformat(),
                "overall": "failed",
                "domain_id": 63,
                "reason_code": str(error),
            },
            output_path,
        )
        node.get_logger().error(f"mapping acceptance failed: {error}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
