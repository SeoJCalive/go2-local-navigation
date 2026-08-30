"""
동일 external-short bag을 두 static TF profile로 순차 mapping한다.

Domain 63 loopback과 simulated clock에서 project 기본값을 먼저 재생하고 DimOS
source-aligned profile을 이어서 재생한다. 각 run은 지도·pose graph와 연속성 결과를
별도 경로에 남기며 command·control interface를 시작하지 않는다.

"""
from dataclasses import asdict
from datetime import datetime
import os
from pathlib import Path
from typing import Final

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node

from bringup.mode_observer import ExecutionMode, ModeEnvironment, assess_mode_environment
from bringup.preflight_result import JsonDocument, write_document
from bringup.preflight_types import CheckStatus
from go2_validation.mapping_acceptance import MappingStatus, MappingVariant
from go2_validation.mapping_artifacts import MappingArtifactError
from go2_validation.mapping_runtime_data import MappingRuntimeDataError
from go2_validation.mapping_runtime_execution import (
    MappingRuntimeError,
    MappingVariantExecution,
    MappingVariantSpec,
    run_mapping_variant,
)
from go2_validation.mapping_slam_services import SlamServiceError
from go2_validation.mapping_tf_profile_ab_input import (
    MappingTfProfileAbInput,
    load_mapping_tf_profile_ab_input,
)


BASELINE_PROFILE: Final = "project_default"
CANDIDATE_PROFILE: Final = "dimos_replay"
CONTINUITY_CHECKS: Final = frozenset(
    {
        "map_correction_samples",
        "map_correction_translation_step",
        "map_correction_yaw_step",
        "map_correction_unaligned",
        "map_correction_stamp_regression",
    }
)


def run_mapping_tf_profile_ab(
    replay: MappingTfProfileAbInput,
    run_directory: Path,
) -> tuple[MappingVariantExecution, MappingVariantExecution]:
    """두 profile을 동일 입력과 mapping parameter로 순차 실행한다."""
    executions: list[MappingVariantExecution] = []
    for profile_id in (BASELINE_PROFILE, CANDIDATE_PROFILE):
        execution = run_mapping_variant(
            MappingVariantSpec(
                variant=MappingVariant.EXTERNAL_DYNAMIC_SHORT,
                bag_path=replay.bag_path,
                provenance=replay.provenance,
                source_checksum=replay.source_checksum,
                replay_checksum=replay.replay_checksum,
                expectation=replay.expectation,
                sensor_tf_profile=profile_id,
                execution_mode="external_replay",
                continuity_profile="replay_enforce",
            ),
            run_directory / profile_id,
        )
        write_mapping_ab_execution(
            execution,
            run_directory / profile_id / "result.json",
        )
        executions.append(execution)
    return executions[0], executions[1]


def mapping_ab_execution_document(
    execution: MappingVariantExecution,
) -> JsonDocument:
    return {
        "result": asdict(execution.result),
        "observation": asdict(execution.observation),
        "artifact_paths": [str(path) for path in execution.artifact_paths],
        "log_paths": [str(path) for path in execution.log_paths],
        "physical_execution": False,
        "ground_truth": False,
    }


def write_mapping_ab_execution(
    execution: MappingVariantExecution,
    output_path: Path,
) -> None:
    write_document(
        {
            "schema_version": 2,
            "record_kind": "mapping_tf_profile_variant_result",
            "recorded_at": datetime.now().astimezone().isoformat(),
            **mapping_ab_execution_document(execution),
        },
        output_path,
    )


def _summary_document(
    baseline: MappingVariantExecution,
    candidate: MappingVariantExecution,
) -> tuple[JsonDocument, bool]:
    baseline_failed_continuity = bool(
        CONTINUITY_CHECKS.intersection(baseline.result.failed_checks)
    )
    candidate_passed = candidate.result.status is MappingStatus.PASSED
    toggle_confirmed = baseline_failed_continuity and candidate_passed
    return (
        {
            "schema_version": 2,
            "record_kind": "mapping_tf_profile_ab_result",
            "recorded_at": datetime.now().astimezone().isoformat(),
            "overall": "passed" if toggle_confirmed else "failed",
            "domain_id": 63,
            "loopback_only": True,
            "playback_rate": 1.0,
            "physical_execution": False,
            "command_publication": False,
            "toggle_confirmed": toggle_confirmed,
            "baseline": mapping_ab_execution_document(baseline),
            "candidate": mapping_ab_execution_document(candidate),
        },
        toggle_confirmed,
    )


def mapping_environment_is_valid() -> bool:
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


def main(args: list[str] | None = None) -> None:
    """ROS parameter를 읽고 두 120초 replay와 atomic summary를 실행한다."""
    rclpy.init(args=args)
    node = Node("go2_mapping_tf_profile_ab_runner")
    project_root = Path(get_package_prefix("go2_validation")).parents[1]
    manifest_path = Path(
        str(
            node.declare_parameter(
                "external_manifest",
                str(project_root / "data/external/dimos_go2_indoor/runs/conversion.json"),
            ).value
        )
    )
    output_root = Path(
        str(
            node.declare_parameter(
                "output_root",
                str(project_root / "data/runs/mapping_tf_ab"),
            ).value
        )
    )
    run_label = str(
        node.declare_parameter(
            "run_label",
            datetime.now().astimezone().strftime("%Y%m%d_%H%M%S"),
        ).value
    )
    run_directory = output_root / run_label
    summary_path = run_directory / "summary.json"
    exit_code = 2
    try:
        if not mapping_environment_is_valid():
            raise MappingRuntimeError("mapping_tf_ab_environment_mismatch")
        baseline, candidate = run_mapping_tf_profile_ab(
            load_mapping_tf_profile_ab_input(project_root, manifest_path),
            run_directory,
        )
        document, passed = _summary_document(baseline, candidate)
        write_document(document, summary_path)
        exit_code = 0 if passed else 2
    except (
        MappingArtifactError,
        MappingRuntimeDataError,
        MappingRuntimeError,
        SlamServiceError,
        OSError,
        ValueError,
    ) as error:
        run_directory.mkdir(parents=True, exist_ok=True)
        write_document(
            {
                "schema_version": 2,
                "record_kind": "mapping_tf_profile_ab_result",
                "recorded_at": datetime.now().astimezone().isoformat(),
                "overall": "failed",
                "domain_id": 63,
                "reason_code": str(error),
            },
            summary_path,
        )
        node.get_logger().error(f"mapping TF profile A/B failed: {error}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
