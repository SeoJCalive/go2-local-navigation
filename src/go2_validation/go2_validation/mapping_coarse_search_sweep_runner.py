"""
DimOS short bag에서 emit3 profile의 coarse search 후보를 순차 비교한다.

동일한 bag·TF·scan·SLAM 통제값을 유지하고 coarse search angle offset만 바꾼다.
각 후보는 Domain 63 loopback에서 지도 저장·재로딩과 연속성 gate까지 독립 실행하며,
command·control interface나 실제 Go2 연결을 사용하지 않는다. 기본 후보는 7개이며,
재현성 확인이나 좁은 탐색은 ROS parameter로 후보 목록을 주입한다.
"""

from datetime import datetime
from math import isfinite
from pathlib import Path
from typing import Final

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node

from bringup.preflight_result import JsonDocument, write_document
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
from go2_validation.mapping_tf_profile_ab_runner import (
    mapping_ab_execution_document,
    mapping_environment_is_valid,
)
from go2_validation.mapping_tf_continuity import (
    MAXIMUM_TRANSLATION_STEP_M,
    MAXIMUM_YAW_STEP_RAD,
)


COARSE_SEARCH_ANGLE_OFFSETS: Final = (
    0.0698,
    0.1047,
    0.1396,
    0.1745,
    0.2094,
    0.2443,
    0.2792,
)
SENSOR_TF_PROFILE: Final = "dimos_replay"
SCAN_PROJECTION_PROFILE: Final = "dimos_odom_accumulated_emit3"


def build_mapping_coarse_search_specs(
    replay: MappingTfProfileAbInput,
    angle_offsets: tuple[float, ...] = COARSE_SEARCH_ANGLE_OFFSETS,
) -> tuple[MappingVariantSpec, ...]:
    """Coarse 범위만 다른 external replay 실행 계약을 만든다."""
    return tuple(
        MappingVariantSpec(
            variant=MappingVariant.EXTERNAL_DYNAMIC_SHORT,
            bag_path=replay.bag_path,
            provenance=replay.provenance,
            source_checksum=replay.source_checksum,
            replay_checksum=replay.replay_checksum,
            expectation=replay.expectation,
            sensor_tf_profile=SENSOR_TF_PROFILE,
            scan_projection_profile=SCAN_PROJECTION_PROFILE,
            execution_mode="external_replay",
            continuity_profile="replay_enforce",
            coarse_search_angle_offset=offset,
            use_response_expansion=False,
            do_loop_closing=True,
            expected_intrinsic_untransformable_cloud_count=1,
        )
        for offset in angle_offsets
    )


def parse_angle_offsets(raw_value: str) -> tuple[float, ...]:
    """ROS parameter의 comma-separated radian 후보를 검증하고 파싱한다."""
    if not raw_value.strip():
        return COARSE_SEARCH_ANGLE_OFFSETS
    tokens = tuple(token.strip() for token in raw_value.split(","))
    if not tokens or any(not token for token in tokens):
        raise MappingRuntimeError(
            "mapping_coarse_search_offsets_invalid",
            "comma-separated radian values are required",
        )
    try:
        offsets = tuple(float(token) for token in tokens)
    except ValueError as error:
        raise MappingRuntimeError(
            "mapping_coarse_search_offsets_invalid",
            raw_value,
        ) from error
    if (
        not offsets
        or len(set(offsets)) != len(offsets)
        or any(not isfinite(offset) or offset < 0.0 for offset in offsets)
    ):
        raise MappingRuntimeError(
            "mapping_coarse_search_offsets_invalid",
            "offsets must be unique finite non-negative radians",
        )
    return offsets


def run_mapping_coarse_search_sweep(
    replay: MappingTfProfileAbInput,
    run_directory: Path,
    angle_offsets: tuple[float, ...] = COARSE_SEARCH_ANGLE_OFFSETS,
) -> tuple[MappingVariantExecution, ...]:
    """후보를 순차 실행하고 후보별 원시 결과를 고유 경로에 저장한다."""
    executions: list[MappingVariantExecution] = []
    for spec in build_mapping_coarse_search_specs(replay, angle_offsets):
        candidate_directory = run_directory / (
            f"coarse_{spec.coarse_search_angle_offset:.4f}".replace(".", "p")
        )
        execution = run_mapping_variant(spec, candidate_directory)
        write_document(
            {
                "schema_version": 2,
                "record_kind": "mapping_coarse_search_candidate_result",
                "recorded_at": datetime.now().astimezone().isoformat(),
                **mapping_ab_execution_document(execution),
            },
            candidate_directory / "result.json",
        )
        executions.append(execution)
    return tuple(executions)


def mapping_coarse_search_sweep_document(
    executions: tuple[MappingVariantExecution, ...],
) -> JsonDocument:
    """완료된 후보 결과와 공통 acceptance 기준을 한 summary로 투영한다."""
    return {
        "schema_version": 2,
        "record_kind": "mapping_coarse_search_sweep_result",
        "recorded_at": datetime.now().astimezone().isoformat(),
        "overall": "completed",
        "domain_id": 63,
        "loopback_only": True,
        "playback_rate": 1.0,
        "physical_execution": False,
        "command_publication": False,
        "sensor_tf_profile": SENSOR_TF_PROFILE,
        "scan_projection_profile": SCAN_PROJECTION_PROFILE,
        "candidate_count": len(executions),
        "angle_offsets_rad": [
            execution.observation.coarse_search_angle_offset
            for execution in executions
        ],
        "passed_candidate_count": sum(
            execution.result.status is MappingStatus.PASSED
            for execution in executions
        ),
        "acceptance_thresholds": {
            "maximum_translation_step_m": MAXIMUM_TRANSLATION_STEP_M,
            "maximum_yaw_step_rad": MAXIMUM_YAW_STEP_RAD,
        },
        "candidates": [
            mapping_ab_execution_document(execution) for execution in executions
        ],
    }


def main(args: list[str] | None = None) -> None:
    """ROS parameter를 읽고 candidate-only replay와 summary를 실행한다."""
    rclpy.init(args=args)
    node = Node("go2_mapping_coarse_search_sweep_runner")
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
                str(project_root / "data/runs/mapping_coarse_sweep"),
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
            raise MappingRuntimeError("mapping_coarse_sweep_environment_mismatch")
        angle_offsets = parse_angle_offsets(
            str(node.declare_parameter("angle_offsets_rad", "").value)
        )
        executions = run_mapping_coarse_search_sweep(
            load_mapping_tf_profile_ab_input(project_root, manifest_path),
            run_directory,
            angle_offsets,
        )
        write_document(mapping_coarse_search_sweep_document(executions), summary_path)
        exit_code = 0
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
                "record_kind": "mapping_coarse_search_sweep_result",
                "recorded_at": datetime.now().astimezone().isoformat(),
                "overall": "failed",
                "domain_id": 63,
                "physical_execution": False,
                "command_publication": False,
                "reason_code": str(error),
            },
            summary_path,
        )
        node.get_logger().error(f"mapping coarse search sweep failed: {error}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
