"""Domain 65 여섯 결과 요약과 loopback 환경 hard gate를 검증한다."""

import json
from pathlib import Path

from go2_validation.shadow_acceptance_runner import build_shadow_summary
from go2_validation.shadow_environment import (
    ShadowEnvironment,
    assess_shadow_environment,
)
from go2_validation.shadow_runtime_execution import ShadowRunResult
from go2_validation.shadow_scenarios import ShadowTerminalStatus, load_shadow_scenarios
from go2_validation.shadow_verdict import (
    ShadowObservation,
    ShadowStatus,
    assess_shadow_scenario,
)

PACKAGE_ROOT = Path(__file__).parents[1]


def _observation(terminal: ShadowTerminalStatus, *, path: bool, candidate: bool) -> ShadowObservation:
    return ShadowObservation(
        action_terminal=terminal,
        path_count=int(path),
        shadow_candidate_count=int(candidate),
        feedback_count=1,
        lifecycle_states=(
            ("behavior_server", "active"),
            ("bt_navigator", "active"),
            ("controller_server", "active"),
            ("map_server", "active"),
            ("planner_server", "active"),
        ),
        global_costmap_count=1,
        local_costmap_count=1,
        clock_publisher_max=1,
        clock_progressed=True,
        map_to_odom_owners=("/synthetic_navigation_fixture",),
        odom_to_base_owners=("/synthetic_navigation_fixture",),
        physical_command_publisher_max=0,
        control_node_max=0,
        unitree_node_max=0,
        fixture_exit_code=0,
        launch_exit_code=0,
        residual_nodes=(),
        residual_processes=(),
        teardown_clock_publishers=0,
        teardown_tf_owners=(),
    )


def test_given_six_terminal_observations_when_summary_built_then_rows_are_sequential_and_json_safe() -> None:
    # Given: one complete accepted observation per configured Domain 65 scenario
    scenarios = load_shadow_scenarios(PACKAGE_ROOT / "config" / "shadow_scenarios.yaml")
    observations = {
        "success": _observation(ShadowTerminalStatus.SUCCEEDED, path=True, candidate=True),
        "cancel": _observation(ShadowTerminalStatus.CANCELED, path=True, candidate=True),
        "blocked_goal": _observation(ShadowTerminalStatus.ABORTED, path=False, candidate=False),
        "outside_map_goal": _observation(ShadowTerminalStatus.ABORTED, path=False, candidate=False),
        "planner_failure": _observation(ShadowTerminalStatus.ABORTED, path=False, candidate=False),
        "no_progress": _observation(ShadowTerminalStatus.ABORTED, path=True, candidate=True),
    }
    results = tuple(
        ShadowRunResult(
            scenario_id=scenario.scenario_id,
            verdict=assess_shadow_scenario(
                scenario.scenario_id,
                observations[scenario.scenario_id],
            ),
            observation=observations[scenario.scenario_id],
            log_paths=(),
        )
        for scenario in scenarios
    )

    # When: the acceptance layer projects the ordered run results into a JSON summary.
    summary = build_shadow_summary(scenarios, results)

    # Then: six passed rows preserve manifest order and the safety status is passed.
    assert summary["overall"] == ShadowStatus.PASSED.value
    assert [row["scenario_id"] for row in summary["scenarios"]] == [
        scenario.scenario_id for scenario in scenarios
    ]
    assert all(row["status"] == ShadowStatus.PASSED.value for row in summary["scenarios"])
    assert all("observation" in row for row in summary["scenarios"])
    assert all("fixture_exit_code" in row["observation"] for row in summary["scenarios"])
    assert json.loads(json.dumps(summary))["overall"] == ShadowStatus.PASSED.value


def test_given_non_loopback_or_wall_time_when_environment_assessed_then_domain65_runner_is_rejected() -> None:
    # Given: Domain 65 environment candidates that violate loopback or simulated-time isolation
    baseline = ShadowEnvironment(
        65,
        "rmw_cyclonedds_cpp",
        "lo",
        '<NetworkInterface name="lo" multicast="false" />',
        True,
    )

    # When: the runner checks its hard execution boundary.
    wrong_domain = assess_shadow_environment(
        ShadowEnvironment(
            0,
            baseline.rmw_implementation,
            baseline.go2_interface,
            baseline.cyclonedds_uri,
            True,
        )
    )
    wall_time = assess_shadow_environment(
        ShadowEnvironment(
            65,
            baseline.rmw_implementation,
            baseline.go2_interface,
            baseline.cyclonedds_uri,
            False,
        )
    )
    wrong_rmw = assess_shadow_environment(
        ShadowEnvironment(
            65,
            "rmw_fastrtps_cpp",
            baseline.go2_interface,
            baseline.cyclonedds_uri,
            True,
        )
    )
    wrong_interface = assess_shadow_environment(
        ShadowEnvironment(
            65,
            baseline.rmw_implementation,
            "eno1",
            baseline.cyclonedds_uri,
            True,
        )
    )

    # Then: no scenario child may start outside Domain 65 loopback simulated time.
    assert assess_shadow_environment(baseline) is None
    assert wrong_domain == "shadow_domain_mismatch"
    assert wall_time == "shadow_sim_time_required"
    assert wrong_rmw == "shadow_rmw_mismatch"
    assert wrong_interface == "shadow_interface_mismatch"
