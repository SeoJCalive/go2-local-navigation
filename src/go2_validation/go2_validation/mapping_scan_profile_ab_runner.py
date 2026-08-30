
"""같은 DimOS TF에서 raw scan과 odometry 보정 누적 scan을 A/B한다.
외부 short bag, SLAM 설정과 TF profile은 고정한다. scan projection profile과
profile-scoped coarse search angle offset만 바꾸며 두 실행 모두 command·control
interface를 시작하지 않는다.
"""

from dataclasses import asdict, dataclass
from datetime import datetime
import os
from pathlib import Path
from typing import Final

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node

from bringup.preflight_result import write_document
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
from go2_validation.mapping_tf_profile_ab_runner import (
    MappingTfProfileAbInput,
    load_mapping_tf_profile_ab_input,
    mapping_ab_execution_document,
    mapping_environment_is_valid,
    write_mapping_ab_execution,
)


SENSOR_TF_PROFILE: Final = "dimos_replay"
BASELINE_SCAN_PROFILE: Final = "raw_single"
CANDIDATE_SCAN_PROFILE: Final = "dimos_odom_accumulated_emit3"
BASELINE_COARSE_SEARCH_ANGLE_OFFSET: Final = 0.349
CANDIDATE_COARSE_SEARCH_ANGLE_OFFSET: Final = 0.1745
CONTINUITY_CHECKS: Final = frozenset(
    {
        "map_correction_samples",
        "map_correction_translation_step",
        "map_correction_yaw_step",
        "map_correction_unaligned",
        "map_correction_stamp_regression",
    }
)


@dataclass(frozen=True, slots=True)
class MappingScanProfilePair:
    """한 A/B run의 실제 baseline·candidate scan profile ID다."""

    baseline: str
    candidate: str


def run_mapping_scan_profile_ab(
    replay: MappingTfProfileAbInput,
    run_directory: Path,
    profiles: MappingScanProfilePair,
) -> tuple[MappingVariantExecution, MappingVariantExecution]:
    """TF·bag·SLAM은 고정하고 scan profile·profile별 coarse bound를 순차 실행한다."""
    executions: list[MappingVariantExecution] = []
    for scan_profile, coarse_search_angle_offset in (
        (profiles.baseline, BASELINE_COARSE_SEARCH_ANGLE_OFFSET),
        (profiles.candidate, CANDIDATE_COARSE_SEARCH_ANGLE_OFFSET),
    ):
        execution = run_mapping_variant(
            MappingVariantSpec(
                variant=MappingVariant.EXTERNAL_DYNAMIC_SHORT,
                bag_path=replay.bag_path,
                provenance=replay.provenance,
                source_checksum=replay.source_checksum,
                replay_checksum=replay.replay_checksum,
                expectation=replay.expectation,
                sensor_tf_profile=SENSOR_TF_PROFILE,
                scan_projection_profile=scan_profile,
                execution_mode="external_replay",
                continuity_profile="replay_enforce",
                coarse_search_angle_offset=coarse_search_angle_offset,
                use_response_expansion=False,
                do_loop_closing=True,
                expected_intrinsic_untransformable_cloud_count=1,
            ),
            run_directory / scan_profile,
        )
        write_mapping_ab_execution(
            execution,
            run_directory / scan_profile / "result.json",
        )
        executions.append(execution)
    return executions[0], executions[1]


def _summary_document(
    baseline: MappingVariantExecution,
    candidate: MappingVariantExecution,
    profiles: MappingScanProfilePair,
):
    baseline_failed_continuity = bool(
        CONTINUITY_CHECKS.intersection(baseline.result.failed_checks)
    )
    candidate_passed = candidate.result.status is MappingStatus.PASSED
    baseline_density = baseline.observation.streams.scan_quality
    candidate_density = candidate.observation.streams.scan_quality
    density_improved = (
        candidate_density.median_valid_beams > baseline_density.median_valid_beams
    )
    toggle_confirmed = baseline_failed_continuity and candidate_passed
    return (
        {
            "schema_version": 2,
            "record_kind": "mapping_scan_profile_ab_result",
            "recorded_at": datetime.now().astimezone().isoformat(),
            "overall": "passed" if toggle_confirmed else "failed",
            "comparison_dimension": "scan_projection_and_coarse_search_bound",
            "domain_id": 63,
            "loopback_only": True,
            "playback_rate": 1.0,
            "physical_execution": False,
            "command_publication": False,
            "sensor_tf_profile": SENSOR_TF_PROFILE,
            "baseline_scan_profile": profiles.baseline,
            "candidate_scan_profile": profiles.candidate,
            "scan_density_improved": density_improved,
            "toggle_confirmed": toggle_confirmed,
            "baseline_scan_quality": asdict(baseline_density),
            "candidate_scan_quality": asdict(candidate_density),
            "baseline": mapping_ab_execution_document(baseline),
            "candidate": mapping_ab_execution_document(candidate),
        },
        toggle_confirmed,
    )


def main(args: list[str] | None = None) -> None:
    """TF·bag·SLAM은 고정하고 scan profile·profile별 coarse bound A/B를 실행한다."""
    rclpy.init(args=args)
    node = Node("go2_mapping_scan_profile_ab_runner")
    project_root = Path(get_package_prefix("go2_validation")).parents[1]
    manifest_path = Path(
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
    output_root = Path(
        str(
            node.declare_parameter(
                "output_root",
                str(project_root / "data/runs/mapping_scan_ab"),
            ).value
        )
    )
    run_label = str(
        node.declare_parameter(
            "run_label",
            datetime.now().astimezone().strftime("%Y%m%d_%H%M%S"),
        ).value
    )
    profiles = MappingScanProfilePair(
        baseline=str(
            node.declare_parameter(
                "baseline_scan_profile",
                BASELINE_SCAN_PROFILE,
            ).value
        ),
        candidate=str(
            node.declare_parameter(
                "candidate_scan_profile",
                CANDIDATE_SCAN_PROFILE,
            ).value
        ),
    )
    run_directory = output_root / run_label
    summary_path = run_directory / "summary.json"
    exit_code = 2
    try:
        if not mapping_environment_is_valid():
            raise MappingRuntimeError("mapping_scan_ab_environment_mismatch")
        if (
            not profiles.baseline
            or not profiles.candidate
            or profiles.baseline == profiles.candidate
        ):
            raise MappingRuntimeError("mapping_scan_ab_profiles_invalid")
        baseline, candidate = run_mapping_scan_profile_ab(
            load_mapping_tf_profile_ab_input(project_root, manifest_path),
            run_directory,
            profiles,
        )
        document, passed = _summary_document(baseline, candidate, profiles)
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
                "record_kind": "mapping_scan_profile_ab_result",
                "recorded_at": datetime.now().astimezone().isoformat(),
                "overall": "failed",
                "domain_id": 63,
                "baseline_scan_profile": profiles.baseline,
                "candidate_scan_profile": profiles.candidate,
                "reason_code": str(error),
            },
            summary_path,
        )
        node.get_logger().error(f"mapping scan profile A/B failed: {error}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
