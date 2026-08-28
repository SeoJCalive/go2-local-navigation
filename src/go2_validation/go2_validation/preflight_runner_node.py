"""
비동작 preflight launch·Jetson telemetry·clean teardown을 한 번에 실행한다.

`ros2 run go2_validation integrated_preflight`의 entry point다. 관찰 중 motion 관련
publish를 수행하지 않으며, 자동 산출물은 project `data/runs/preflight`에 저장한다.

"""
from dataclasses import asdict
from datetime import datetime
import json
import os
import subprocess
from time import monotonic, time

import rclpy
from rclpy.node import Node

from bringup.preflight_assessments import project_publisher_count
from bringup.preflight_configuration import EXPECTED_NODES
from bringup.preflight_host import (
    TeardownObservation,
    collect_kernel_observation,
    read_passive_trip_c,
    scan_residual_processes,
    stop_owned_process,
)
from bringup.preflight_metrics import overall_status
from bringup.preflight_result import (
    ReportFormatError,
    load_document,
    read_observer_status,
    write_document,
)
from bringup.preflight_resources import assess_resources, parse_tegrastats
from bringup.preflight_types import CheckResult, CheckStatus
from go2_validation.preflight_runner_configuration import (
    ConfigurationError,
    RunConfiguration,
    parse_configuration,
)


POSTFLIGHT_DISCOVERY_SECONDS = 3.0


def _execute(node: Node, configuration: RunConfiguration) -> int:
    started_at = datetime.now().astimezone()
    run_id = f"{started_at:%Y%m%d_%H%M%S}_{configuration.run_label}"
    run_directory = configuration.output_root / run_id
    run_directory.mkdir(parents=True, exist_ok=False)
    observer_path = run_directory / "observer.json"
    launch_log_path = run_directory / "launch.log"
    tegrastats_path = run_directory / "tegrastats.log"
    result_path = run_directory / "result.json"
    launch_command = [
        "ros2",
        "launch",
        "go2_validation",
        "go2_integrated_preflight.launch.py",
        f"duration_sec:={configuration.duration_seconds}",
        f"run_id:={run_id}",
        f"run_label:={configuration.run_label}",
        f"report_path:={observer_path}",
    ]
    node.get_logger().info(
        f"integrated preflight run started: run_id={run_id} "
        f"duration_sec={configuration.duration_seconds}"
    )
    launch_timed_out = False
    start_epoch_seconds = time()
    with launch_log_path.open("w", encoding="utf-8") as launch_log, \
            tegrastats_path.open("w", encoding="utf-8") as tegrastats_log:
        tegrastats_process = subprocess.Popen(
            ["tegrastats", "--interval", "5000"],
            stdout=tegrastats_log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        launch_process = subprocess.Popen(
            launch_command,
            stdout=launch_log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        try:
            launch_return_code = launch_process.wait(
                timeout=configuration.duration_seconds + 120
            )
        except subprocess.TimeoutExpired:
            launch_timed_out = True
            launch_return_code = stop_owned_process(launch_process)
        finally:
            stop_owned_process(tegrastats_process)
    discovery_deadline = monotonic() + POSTFLIGHT_DISCOVERY_SECONDS
    while rclpy.ok() and monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    residual_nodes = tuple(
        sorted(
            {
                f"/{name}" if namespace == "/" else f"{namespace}/{name}"
                for name, namespace in node.get_node_names_and_namespaces()
            }.intersection(EXPECTED_NODES)
        )
    )
    teardown = TeardownObservation(
        launch_return_code=launch_return_code,
        launch_timed_out=launch_timed_out,
        residual_nodes=residual_nodes,
        residual_processes=scan_residual_processes(os.getpid()),
        sport_request_publishers=project_publisher_count(
            tuple(node.get_publishers_info_by_topic("/api/sport/request")),
            EXPECTED_NODES,
        ),
        lowcmd_publishers=project_publisher_count(
            tuple(node.get_publishers_info_by_topic("/lowcmd")),
            EXPECTED_NODES,
        ),
    )
    telemetry_lines = tuple(
        tegrastats_path.read_text(encoding="utf-8").splitlines()
    )
    resource_summary = parse_tegrastats(telemetry_lines)
    passive_trip = read_passive_trip_c()
    kernel = collect_kernel_observation(start_epoch_seconds)
    trip_check = CheckResult(
        check_id="resources.passive_trip_source",
        status=CheckStatus.PASS if passive_trip is not None else CheckStatus.WARN,
        detail=f"passive_trip_c={passive_trip}",
    )
    resource_checks = assess_resources(
        resource_summary,
        passive_trip_c=passive_trip or 70.0,
        kernel=kernel,
    )
    launch_check = CheckResult(
        check_id="teardown.launch_exit",
        status=(
            CheckStatus.PASS
            if launch_return_code == 0 and not launch_timed_out
            else CheckStatus.FAIL
        ),
        detail=(
            f"return_code={launch_return_code}; timed_out={launch_timed_out}"
        ),
    )
    residual_check = CheckResult(
        check_id="teardown.residual_graph_process",
        status=(
            CheckStatus.PASS
            if not teardown.residual_nodes and not teardown.residual_processes
            else CheckStatus.FAIL
        ),
        detail=(
            f"nodes={teardown.residual_nodes}; "
            f"processes={teardown.residual_processes}"
        ),
    )
    command_check = CheckResult(
        check_id="teardown.command_publishers_absent",
        status=(
            CheckStatus.PASS
            if teardown.sport_request_publishers == 0
            and teardown.lowcmd_publishers == 0
            else CheckStatus.FAIL
        ),
        detail=(
            "sport_request_project_publishers="
            f"{teardown.sport_request_publishers}; "
            f"lowcmd_project_publishers={teardown.lowcmd_publishers}"
        ),
    )
    document = load_document(observer_path)
    observer_status = read_observer_status(document)
    additional_checks = (
        trip_check,
        *resource_checks,
        launch_check,
        residual_check,
        command_check,
    )
    final_status = overall_status(
        (
            CheckResult("observer.overall", observer_status, "observer.json"),
            *additional_checks,
        )
    )
    checks = document.get("checks")
    if not isinstance(checks, list):
        raise ReportFormatError("observer checks must be an array")
    checks.extend(asdict(check) for check in additional_checks)
    document["record_kind"] = "integrated_non_actuating_preflight_result"
    document["overall_status"] = final_status.value
    document["finalized_at"] = datetime.now().astimezone().isoformat()
    document["resources"] = asdict(resource_summary)
    document["kernel"] = asdict(kernel)
    document["teardown"] = asdict(teardown)
    document["artifacts"] = {
        "observer": observer_path.name,
        "launch_log": launch_log_path.name,
        "tegrastats_log": tegrastats_path.name,
    }
    write_document(document, result_path)
    node.get_logger().info(
        f"integrated preflight run completed: status={final_status.value} "
        f"result={result_path}"
    )
    return 1 if final_status is CheckStatus.FAIL else 0


def main(args: list[str] | None = None) -> None:
    """Runner parameter를 읽고 preflight·telemetry·teardown을 순서대로 실행한다."""
    rclpy.init(args=args)
    node = Node("go2_integrated_preflight_runner")
    exit_code = 2
    try:
        exit_code = _execute(node, parse_configuration(node))
    except (ConfigurationError, ReportFormatError, OSError, json.JSONDecodeError) as error:
        node.get_logger().error(f"integrated preflight runner failed: {error}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
