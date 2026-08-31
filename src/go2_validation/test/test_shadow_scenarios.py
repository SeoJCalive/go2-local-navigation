"""합성 scenario YAML의 여섯 terminal 계약과 timeout 여유를 검증한다."""

from pathlib import Path
from typing import Final

from go2_validation.shadow_scenarios import (
    ShadowTerminalStatus,
    load_shadow_scenarios,
)

PACKAGE_ROOT: Final = Path(__file__).parents[1]
SCENARIO_PATH: Final = PACKAGE_ROOT / "config" / "shadow_scenarios.yaml"


def test_given_shadow_manifest_when_loaded_then_all_six_terminal_contracts_are_typed() -> None:
    # Given: shared synthetic navigation scenario manifest
    scenarios = load_shadow_scenarios(SCENARIO_PATH)

    # When: each scenario is parsed at the validation boundary.
    by_id = {scenario.scenario_id: scenario for scenario in scenarios}

    # Then: the complete Domain 65 matrix retains its terminal and observable contract.
    assert set(by_id) == {
        "success",
        "cancel",
        "blocked_goal",
        "outside_map_goal",
        "planner_failure",
        "no_progress",
    }
    assert by_id["success"].expected_terminal is ShadowTerminalStatus.SUCCEEDED
    assert by_id["cancel"].expected_terminal is ShadowTerminalStatus.CANCELED
    assert by_id["blocked_goal"].expected_terminal is ShadowTerminalStatus.ABORTED
    assert by_id["outside_map_goal"].expected_terminal is ShadowTerminalStatus.ABORTED
    assert by_id["planner_failure"].expected_terminal is ShadowTerminalStatus.ABORTED
    assert by_id["no_progress"].expected_terminal is ShadowTerminalStatus.ABORTED
    assert by_id["success"].expects_path is True
    assert by_id["cancel"].expects_candidate is True
    assert by_id["blocked_goal"].expects_path is False
    assert by_id["outside_map_goal"].expects_candidate is False
    assert by_id["planner_failure"].expects_path is False
    assert by_id["no_progress"].expects_path is True


def test_given_five_meter_success_path_when_loaded_then_timeout_has_runtime_headroom() -> None:
    # Given: the shared synthetic navigation scenario manifest.
    scenarios = load_shadow_scenarios(SCENARIO_PATH)

    # When: the success scenario's runtime boundary is selected.
    success = next(
        scenario for scenario in scenarios if scenario.scenario_id == "success"
    )

    # Then: the acceptance window leaves headroom above the observed 30-second run.
    assert success.timeout_seconds >= 60
