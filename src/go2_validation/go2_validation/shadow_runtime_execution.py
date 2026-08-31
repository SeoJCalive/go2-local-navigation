"""Domain 65 child lifecycle, bounded settle, observer와 verdict를 소유한다."""

import os
import subprocess
from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path
from typing import Final

import rclpy
from bringup.preflight_host import (
    scan_residual_processes,
)
from bringup.preflight_host import (
    stop_owned_process as stop_process_group,
)

from go2_validation.offline_process import spin_until, stop_owned_process
from go2_validation.shadow_action_runner import (
    ShadowActionError,
    run_navigation_action,
    shadow_fixture_command,
    shadow_launch_command,
)
from go2_validation.shadow_observer import ShadowRuntimeObserver
from go2_validation.shadow_runtime_model import ShadowTerminalEvidence
from go2_validation.shadow_scenarios import ShadowScenario, ShadowTerminalStatus
from go2_validation.shadow_verdict import (
    ShadowObservation,
    ShadowVerdict,
    assess_shadow_scenario,
)

READINESS_TIMEOUT_SECONDS: Final = 30.0
SETTLE_TIMEOUT_SECONDS: Final = 30.0


@dataclass(frozen=True, slots=True)
class ShadowRuntimeError(Exception):
    """Domain 65 child lifecycle 또는 fixture 경계가 깨진 이유다."""

    reason_code: str
    detail: str = ""

    def __str__(self) -> str:
        return f"{self.reason_code}: {self.detail}" if self.detail else self.reason_code


@dataclass(frozen=True, slots=True)
class ShadowRunResult:
    """시나리오 verdict, 원시 관찰과 local log 경로다."""

    scenario_id: str
    verdict: ShadowVerdict
    observation: ShadowObservation
    log_paths: tuple[Path, ...]


@dataclass(frozen=True, slots=True)
class _ChildExecution:
    terminal: ShadowTerminalStatus
    fixture_exit_code: int
    launch_exit_code: int
    log_paths: tuple[Path, ...]


@dataclass(frozen=True, slots=True)
class _ExecutionSpec:
    scenario: ShadowScenario
    map_path: Path
    run_directory: Path


def run_shadow_scenario(
    scenario: ShadowScenario,
    map_path: Path,
    run_directory: Path,
) -> ShadowRunResult:
    """한 시나리오의 ROS context를 생성하고 terminal·teardown까지 닫는다."""
    if not map_path.is_file() or map_path.is_symlink():
        raise ShadowRuntimeError("shadow_map_path_invalid", str(map_path))
    try:
        run_directory.mkdir(parents=True, exist_ok=False)
    except FileExistsError as error:
        raise ShadowRuntimeError(
            "shadow_run_directory_exists",
            str(run_directory),
        ) from error

    rclpy.init()
    observer = ShadowRuntimeObserver()
    try:
        child = _execute_children(
            observer,
            _ExecutionSpec(
                scenario=scenario,
                map_path=map_path,
                run_directory=run_directory,
            ),
        )
        spin_until(observer, observer.teardown_complete, SETTLE_TIMEOUT_SECONDS)
        observer.observe_graph()
        observation = observer.observation(
            ShadowTerminalEvidence(
                action_terminal=child.terminal,
                fixture_exit_code=child.fixture_exit_code,
                launch_exit_code=child.launch_exit_code,
                residual_nodes=observer.residual_nodes(),
                residual_processes=scan_residual_processes(os.getpid()),
            )
        )
        return ShadowRunResult(
            scenario_id=scenario.scenario_id,
            verdict=assess_shadow_scenario(scenario.scenario_id, observation),
            observation=observation,
            log_paths=child.log_paths,
        )
    finally:
        observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _execute_children(
    observer: ShadowRuntimeObserver,
    spec: _ExecutionSpec,
) -> _ChildExecution:
    fixture_log = spec.run_directory / "fixture.log"
    launch_log = spec.run_directory / "nav2-shadow.log"
    fixture_process: subprocess.Popen[str] | None = None
    launch_process: subprocess.Popen[str] | None = None
    fixture_exit_code = -1
    launch_exit_code = -1
    try:
        with ExitStack() as stack:
            fixture_output = stack.enter_context(
                fixture_log.open("w", encoding="utf-8")
            )
            launch_output = stack.enter_context(
                launch_log.open("w", encoding="utf-8")
            )
            fixture_process = subprocess.Popen(
                shadow_fixture_command(spec.scenario),
                stdout=fixture_output,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            launch_process = subprocess.Popen(
                shadow_launch_command(spec.map_path),
                stdout=launch_output,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            spin_until(
                observer,
                lambda: (
                    observer.ready_for_action()
                    or launch_process.poll() is not None
                    or fixture_process.poll() is not None
                ),
                READINESS_TIMEOUT_SECONDS,
            )
            if launch_process.poll() is not None:
                raise ShadowRuntimeError(
                    "shadow_launch_early_exit",
                    str(launch_process.returncode),
                )
            if fixture_process.poll() is not None:
                raise ShadowRuntimeError(
                    "shadow_fixture_early_exit",
                    str(fixture_process.returncode),
                )
            if not observer.ready_for_action():
                raise ShadowRuntimeError("shadow_readiness_timeout")
            observer.capture_lifecycle_states(READINESS_TIMEOUT_SECONDS)
            try:
                terminal = run_navigation_action(observer, spec.scenario)
            except ShadowActionError as error:
                raise ShadowRuntimeError(error.reason_code) from error
            observer.capture_lifecycle_states(READINESS_TIMEOUT_SECONDS)
    finally:
        if launch_process is not None:
            launch_exit_code = stop_owned_process(launch_process)
        if fixture_process is not None:
            fixture_exit_code = stop_process_group(fixture_process)
    return _ChildExecution(
        terminal=terminal,
        fixture_exit_code=fixture_exit_code,
        launch_exit_code=launch_exit_code,
        log_paths=(fixture_log, launch_log),
    )
