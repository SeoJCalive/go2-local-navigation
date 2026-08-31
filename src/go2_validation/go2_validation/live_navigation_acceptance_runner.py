"""Domain 0 live localization·no-goal Nav2를 bounded 실행하고 JSON으로 기록한다."""

import os
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from bringup.mode_observer import ExecutionMode, ModeEnvironment, assess_mode_environment
from bringup.preflight_result import write_document
from bringup.preflight_types import CheckStatus
from rclpy.node import Node

from go2_validation.live_navigation_acceptance import LiveNavigationStatus
from go2_validation.live_navigation_runtime import (
    LiveNavigationRuntimeError,
    run_live_navigation,
)


def _environment_is_valid() -> bool:
    check = assess_mode_environment(
        ExecutionMode.LIVE_SHADOW,
        ModeEnvironment(
            rmw_implementation=os.environ.get("RMW_IMPLEMENTATION", ""),
            ros_domain_id=os.environ.get("ROS_DOMAIN_ID", ""),
            go2_interface=os.environ.get("GO2_AGX_INTERFACE", ""),
            cyclonedds_uri=os.environ.get("CYCLONEDDS_URI", ""),
        ),
    )
    return (
        check.status is CheckStatus.PASS
        and os.environ.get("ROS_LOCALHOST_ONLY", "0") != "1"
    )


def main(args: list[str] | None = None) -> None:
    """ROS parameter를 읽고 한 번의 실제-input no-goal 관찰을 실행한다."""
    rclpy.init(args=args)
    node = Node("go2_live_navigation_acceptance_runner")
    project_root = Path(get_package_prefix("go2_validation")).parents[1]
    output_root = Path(
        str(
            node.declare_parameter(
                "output_root",
                str(project_root / "data/runs/live_navigation"),
            ).value
        )
    )
    run_label = str(
        node.declare_parameter("run_label", "todo12l2-domain0-live").value
    )
    duration_seconds = float(node.declare_parameter("duration_sec", 60.0).value)
    map_path = Path(
        str(
            node.declare_parameter(
                "map_path",
                str(
                    project_root
                    / "data/runs/live_mapping/"
                    "20260831_142854_todo12l_domain0_live_mapping/"
                    "artifacts/occupancy.yaml"
                ),
            ).value
        )
    )
    started_at = datetime.now().astimezone()
    run_directory = output_root / f"{started_at:%Y%m%d_%H%M%S}_{run_label}"
    output_path = run_directory / "result.json"
    exit_code = 2
    try:
        if not _environment_is_valid():
            raise LiveNavigationRuntimeError("live_environment_mismatch")
        execution = run_live_navigation(
            map_path,
            run_directory,
            duration_seconds,
        )
        write_document(
            {
                "schema_version": 1,
                "record_kind": "domain0_live_navigation_acceptance_result",
                "recorded_at": started_at.isoformat(),
                "overall": execution.result.status.value,
                "domain_id": 0,
                "use_sim_time": False,
                "loopback_only": False,
                "physical_execution": False,
                "action_goal_sent": False,
                "command_publication": False,
                "localization_accuracy_ground_truth": False,
                "map_path": str(map_path),
                "map_checksum": execution.map_checksum,
                "duration_seconds": duration_seconds,
                "result": asdict(execution.result),
                "observation": asdict(execution.observation),
                "log_path": str(execution.log_path),
            },
            output_path,
        )
        exit_code = 0 if execution.result.status is LiveNavigationStatus.PASSED else 2
    except (LiveNavigationRuntimeError, OSError, ValueError) as error:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        write_document(
            {
                "schema_version": 1,
                "record_kind": "domain0_live_navigation_acceptance_result",
                "recorded_at": started_at.isoformat(),
                "overall": "failed",
                "domain_id": 0,
                "physical_execution": False,
                "action_goal_sent": False,
                "command_publication": False,
                "reason_code": str(error),
            },
            output_path,
        )
        node.get_logger().error(f"live navigation acceptance failed: {error}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
