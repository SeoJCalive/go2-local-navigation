"""
한 Domain 63 mapping variant의 launch·player·저장·teardown을 소유한다.

Subscriber와 SLAM service readiness 뒤에만 1.0배속 player를 시작한다. 저장 파일은
고유 partial directory에서 검증·reload한 뒤 같은 filesystem의 artifacts directory로
원자 승격한다.

"""
from contextlib import ExitStack
from dataclasses import dataclass
import os
from pathlib import Path
import shutil
import subprocess
from tempfile import mkdtemp
from time import monotonic

import rclpy

from bringup.preflight_host import scan_residual_processes
from go2_validation.mapping_acceptance import (
    MappingArtifactObservation,
    MappingObservation,
    MappingProcessObservation,
    MappingResult,
    MappingVariant,
    assess_mapping,
)
from go2_validation.mapping_cloud_accounting import (
    MappingCloudAccountingError,
    mapping_cloud_accounting_for_profile,
)
from go2_validation.mapping_command_builders import (
    MappingLaunchConfiguration,
    isfinite_positive,
    mapping_bag_play_command,
    mapping_launch_command,
)
from go2_validation.mapping_runtime_data import BagExpectation
from go2_validation.mapping_runtime_observer import MappingRuntimeObserver
from go2_validation.mapping_player_services import (
    MappingPlayerServices,
    MappingRuntimeError,
)
from go2_validation.mapping_slam_services import SlamServiceClient
from go2_validation.offline_process import spin_for, spin_until, stop_owned_process


@dataclass(frozen=True, slots=True)
class MappingVariantSpec:
    """한 replay의 경로·provenance·identity와 전체 소비 기대값이다."""

    variant: MappingVariant
    bag_path: Path
    provenance: str
    source_checksum: str
    replay_checksum: str
    expectation: BagExpectation
    sensor_tf_profile: str
    scan_projection_profile: str = "raw_single"
    execution_mode: str = "onboard"
    continuity_profile: str = "onboard_observe"
    coarse_search_angle_offset: float = 0.349
    use_response_expansion: bool = True
    do_loop_closing: bool = True
    expected_intrinsic_untransformable_cloud_count: int = 0
    playback_rate: float = 1.0


@dataclass(frozen=True, slots=True)
class MappingVariantExecution:
    """Pure verdict, raw observation과 local-only runtime artifact 경로다."""

    result: MappingResult
    observation: MappingObservation
    artifact_paths: tuple[Path, ...]
    log_paths: tuple[Path, ...]


def run_mapping_variant(
    spec: MappingVariantSpec,
    run_directory: Path,
) -> MappingVariantExecution:
    """한 variant의 readiness, full replay, save/reload와 teardown을 완료한다."""
    if not spec.bag_path.is_dir() or spec.bag_path.is_symlink():
        raise MappingRuntimeError("mapping_bag_path_invalid", str(spec.bag_path))
    try:
        run_directory.mkdir(parents=True, exist_ok=False)
    except FileExistsError as error:
        raise MappingRuntimeError("mapping_run_directory_exists", str(run_directory)) from error
    observer = MappingRuntimeObserver(spec.variant.value)
    try:
        return _run_owned_mapping(spec, run_directory, observer)
    finally:
        observer.destroy_node()


def _run_owned_mapping(
    spec: MappingVariantSpec,
    run_directory: Path,
    observer: MappingRuntimeObserver,
) -> MappingVariantExecution:
    services = SlamServiceClient(observer)
    launch_log_path = run_directory / "launch.log"
    player_log_path = run_directory / "player.log"
    launch_process: subprocess.Popen[str] | None = None
    player_process: subprocess.Popen[str] | None = None
    launch_exit_code = -1
    player_exit_code = -1
    artifacts = None
    slam_services_ready = False
    with ExitStack() as stack:
        try:
            launch_output = stack.enter_context(
                launch_log_path.open("w", encoding="utf-8")
            )
            launch_process = subprocess.Popen(
                mapping_launch_command(
                    MappingLaunchConfiguration(
                        sensor_tf_profile=spec.sensor_tf_profile,
                        scan_projection_profile=spec.scan_projection_profile,
                        execution_mode=spec.execution_mode,
                        continuity_profile=spec.continuity_profile,
                        coarse_search_angle_offset=spec.coarse_search_angle_offset,
                        use_response_expansion=spec.use_response_expansion,
                        do_loop_closing=spec.do_loop_closing,
                    )
                ),
                stdout=launch_output,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            ready = spin_until(observer, observer.ready_for_player, 30.0)
            if not ready or launch_process.poll() is not None:
                raise MappingRuntimeError("mapping_readiness_failed")
            if not services.wait_until_ready(30.0):
                raise MappingRuntimeError("mapping_slam_services_unavailable")
            slam_services_ready = True
            player_output = stack.enter_context(
                player_log_path.open("w", encoding="utf-8")
            )
            player_process = subprocess.Popen(
                mapping_bag_play_command(spec.bag_path, spec.playback_rate),
                stdout=player_output,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            deadline = MappingPlayerServices(observer).synchronize_start(
                readiness_timeout_seconds=30.0,
                resume_timeout_seconds=30.0,
                playback_timeout_seconds=spec.expectation.playback_timeout_seconds,
            )
            while (
                rclpy.ok()
                and monotonic() < deadline
                and player_process.poll() is None
            ):
                rclpy.spin_once(observer, timeout_sec=0.05)
                observer.observe_graph()
                if observer.clock_stalled_during_playback():
                    raise MappingRuntimeError("mapping_clock_stalled")
            if player_process.poll() is None:
                raise MappingRuntimeError("mapping_player_timeout", spec.variant.value)
            player_exit_code = player_process.wait()
            if player_exit_code != 0:
                raise MappingRuntimeError(
                    "mapping_player_nonzero_exit",
                    str(player_exit_code),
                )
            spin_for(observer, 3.0)
            observer.observe_graph()
            if observer.map_count <= 0:
                raise MappingRuntimeError("mapping_map_not_received")
            partial_root = Path(
                mkdtemp(prefix=".artifacts-", dir=str(run_directory))
            )
            try:
                artifacts = services.save_serialize_reload(partial_root, 30.0)
                artifact_root = run_directory / "artifacts"
                if artifact_root.exists():
                    raise MappingRuntimeError(
                        "mapping_artifact_directory_exists",
                        str(artifact_root),
                    )
                os.replace(partial_root, artifact_root)
            finally:
                if partial_root.exists():
                    shutil.rmtree(partial_root)
        finally:
            if player_process is not None:
                player_exit_code = stop_owned_process(player_process)
            if launch_process is not None:
                launch_exit_code = stop_owned_process(launch_process)
    if artifacts is None:
        raise MappingRuntimeError("mapping_artifacts_absent")
    spin_until(observer, observer.teardown_complete, 30.0)
    observer.observe_graph()
    artifact_root = run_directory / "artifacts"
    artifact_paths = tuple(artifact_root / path.name for path in artifacts.paths)
    try:
        cloud_accounting = mapping_cloud_accounting_for_profile(
            launch_log_path,
            spec.scan_projection_profile,
        )
    except MappingCloudAccountingError as error:
        raise MappingRuntimeError(error.reason_code, error.detail) from error
    observation = MappingObservation(
        variant=spec.variant,
        provenance=spec.provenance,
        source_checksum=spec.source_checksum,
        replay_checksum=spec.replay_checksum,
        streams=observer.stream_observation(spec.expectation),
        ownership=observer.ownership_observation(slam_services_ready),
        artifacts=MappingArtifactObservation(
            occupancy_saved=True,
            pose_graph_saved=True,
            pose_graph_reloaded=True,
            checksums=artifacts.checksums,
        ),
        process=MappingProcessObservation(
            player_exit_code=player_exit_code,
            launch_exit_code=launch_exit_code,
            residual_nodes=observer.residual_nodes(),
            residual_processes=scan_residual_processes(os.getpid()),
            teardown_clock_publishers=observer.teardown_clock_publishers(),
            teardown_global_owner_nodes=observer.teardown_global_owner_nodes(),
        ),
        sensor_tf_profile=spec.sensor_tf_profile,
        continuity=observer.continuity_observation(),
        map_correction_continuity=observer.map_correction_continuity_observation(),
        scan_projection_profile=spec.scan_projection_profile,
        coarse_search_angle_offset=spec.coarse_search_angle_offset,
        use_response_expansion=spec.use_response_expansion,
        do_loop_closing=spec.do_loop_closing,
        expected_intrinsic_untransformable_cloud_count=(
            spec.expected_intrinsic_untransformable_cloud_count
        ),
        cloud_accounting=cloud_accounting,
        odometry_continuity=observer.odometry_continuity_observation(),
    )
    return MappingVariantExecution(
        result=assess_mapping(observation),
        observation=observation,
        artifact_paths=artifact_paths,
        log_paths=(launch_log_path, player_log_path),
    )
