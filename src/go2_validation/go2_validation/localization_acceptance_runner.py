"""Domain 64 saved-map localization 실행을 검증하고 JSON으로 기록한다."""

import os
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from bringup.mode_observer import (
    ExecutionMode,
    ModeEnvironment,
    assess_mode_environment,
)
from bringup.preflight_result import write_document
from bringup.preflight_types import CheckStatus
from rclpy.node import Node

from go2_validation.localization_acceptance import LocalizationStatus
from go2_validation.localization_runtime_execution import (
    LocalizationRuntimeError,
    run_saved_map_localization,
)
from go2_validation.mapping_player_services import MappingRuntimeError
from go2_validation.mapping_runtime_data import MappingRuntimeDataError


def _environment_is_valid() -> bool:
    check = assess_mode_environment(
        ExecutionMode.LOCALIZATION,
        ModeEnvironment(
            rmw_implementation=os.environ.get("RMW_IMPLEMENTATION", ""),
            ros_domain_id=os.environ.get("ROS_DOMAIN_ID", ""),
            go2_interface=os.environ.get("GO2_AGX_INTERFACE", ""),
            cyclonedds_uri=os.environ.get("CYCLONEDDS_URI", ""),
        ),
    )
    return check.status is CheckStatus.PASS and 'name="lo"' in os.environ.get(
        "CYCLONEDDS_URI",
        "",
    )


def main(args: list[str] | None = None) -> None:
    """ROS parameter를 읽고 한 bounded localization replay를 실행한다."""
    rclpy.init(args=args)
    node = Node("saved_map_localization_acceptance_runner")
    project_root = Path(get_package_prefix("go2_validation")).parents[1]
    output_root = Path(
        str(
            node.declare_parameter(
                "output_root",
                str(project_root / "data/runs/localization"),
            ).value
        )
    )
    run_label = str(node.declare_parameter("run_label", "stage12-domain64").value)
    map_path = Path(
        str(
            node.declare_parameter(
                "map_path",
                str(
                    project_root
                    / "data/runs/mapping/project_stationary/artifacts/occupancy.yaml"
                ),
            ).value
        )
    )
    bag_path = Path(
        str(
            node.declare_parameter(
                "bag_path",
                str(project_root / "data/bags/go2_stationary_raw_20260826_1829"),
            ).value
        )
    )
    run_directory = output_root / run_label
    output_path = run_directory / "result.json"
    exit_code = 2
    try:
        if not _environment_is_valid():
            raise LocalizationRuntimeError("localization_environment_mismatch")
        execution = run_saved_map_localization(map_path, bag_path, run_directory)
        write_document(
            {
                "schema_version": 1,
                "record_kind": "saved_map_localization_acceptance_result",
                "recorded_at": datetime.now().astimezone().isoformat(),
                "overall": execution.result.status.value,
                "domain_id": 64,
                "loopback_only": True,
                "physical_execution": False,
                "command_publication": False,
                "map_accuracy_ground_truth": False,
                "map_path": str(map_path),
                "map_checksum": execution.map_checksum,
                "bag_path": str(bag_path),
                "bag_checksum": execution.bag_checksum,
                "result": asdict(execution.result),
                "observation": asdict(execution.observation),
                "log_paths": [str(path) for path in execution.log_paths],
            },
            output_path,
        )
        exit_code = (
            0 if execution.result.status is LocalizationStatus.PASSED else 2
        )
    except (
        LocalizationRuntimeError,
        MappingRuntimeDataError,
        MappingRuntimeError,
        OSError,
        ValueError,
    ) as error:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        write_document(
            {
                "schema_version": 1,
                "record_kind": "saved_map_localization_acceptance_result",
                "recorded_at": datetime.now().astimezone().isoformat(),
                "overall": "failed",
                "domain_id": 64,
                "reason_code": str(error),
            },
            output_path,
        )
        node.get_logger().error(f"saved-map localization failed: {error}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
