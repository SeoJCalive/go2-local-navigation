"""Typed fault scenario parser tests."""

from pathlib import Path

import pytest
import yaml

from bringup.fault_contract import FaultConfigurationError, load_fault_scenarios
from bringup.fault_result import (
    FaultAcceptanceReport,
    FaultScenarioResult,
    fault_report_document,
)


SCENARIOS = Path(__file__).resolve().parents[1] / "config/fault_scenarios.yaml"


def test_given_fault_scenarios_when_loaded_then_each_is_immutable_and_complete() -> None:
    """The oracle accepts all required Wave 1 fault classes."""
    # Given: the installed-package source contract
    # When: it is parsed
    scenarios = load_fault_scenarios(SCENARIOS)

    # Then: its typed IDs cover each software fault boundary
    assert {scenario.fault_kind for scenario in scenarios} == {
        "malformed_layout", "empty_cloud", "nan_cloud", "stale_cloud",
        "tf_loss", "odom_regression", "odom_jump", "odom_loss",
        "process_exit", "launch_failure",
    }
    assert all(scenario.suppressed_outputs for scenario in scenarios)
    assert all(scenario.recovery_deadline_seconds > 0 for scenario in scenarios)


def test_given_duplicate_id_when_loaded_then_typed_configuration_error_is_raised(
    tmp_path: Path,
) -> None:
    """Duplicate scenario identifiers cannot silently override an oracle entry."""
    # Given: a duplicate scenario fixture
    fixture = tmp_path / "faults.yaml"
    fixture.write_text(
        "scenarios:\n"
        "  - id: duplicate\n"
        "    fault_kind: empty_cloud\n"
        "    suppressed_outputs: [/x]\n"
        "    reason_code: EMPTY_CLOUD\n"
        "    recovery_trigger: valid_cloud\n"
        "    recovery_deadline_seconds: 1\n"
        "    terminal_status: suppressed\n"
        "  - id: duplicate\n"
        "    fault_kind: nan_cloud\n"
        "    suppressed_outputs: [/x]\n"
        "    reason_code: NAN_CLOUD\n"
        "    recovery_trigger: valid_cloud\n"
        "    recovery_deadline_seconds: 1\n"
        "    terminal_status: suppressed\n"
    )

    # When / Then: parsing rejects it at the boundary
    with pytest.raises(FaultConfigurationError, match="duplicate_scenario_id"):
        load_fault_scenarios(fixture)


def test_given_missing_recovery_trigger_when_loaded_then_configuration_is_rejected(
    tmp_path: Path,
) -> None:
    fixture = tmp_path / "faults.yaml"
    fixture.write_text(
        "scenarios:\n"
        "  - id: missing-trigger\n"
        "    fault_kind: empty_cloud\n"
        "    suppressed_outputs: [/x]\n"
        "    reason_code: EMPTY_CLOUD\n"
        "    recovery_deadline_seconds: 1\n"
        "    terminal_status: suppressed\n"
    )

    with pytest.raises(FaultConfigurationError, match="missing_recovery_trigger"):
        load_fault_scenarios(fixture)


def test_given_fault_results_when_serialized_then_schema_is_stable() -> None:
    report = FaultAcceptanceReport(
        overall="passed",
        domain_id=61,
        command_publisher_count=0,
        motion_gates_closed=True,
        scenarios=(
            FaultScenarioResult(
                scenario_id="empty-cloud",
                status="passed",
                reason_code="EMPTY_CLOUD",
                suppressed_outputs=("/scan",),
                recovered_outputs=("/scan",),
                recovery_elapsed_nanoseconds=500_000_000,
                child_exit_code=0,
            ),
        ),
    )

    document = fault_report_document(report)

    assert document["schema_version"] == 1
    assert document["record_kind"] == "software_fault_acceptance_result"
    assert document["overall"] == "passed"
    assert document["scenarios"][0]["scenario_id"] == "empty-cloud"


@pytest.mark.parametrize(
    ("scenario_row", "reason"),
    [
        ("not-a-mapping", "invalid_scenario_row"),
        (
            {
                "id": "bad-outputs",
                "fault_kind": "empty_cloud",
                "suppressed_outputs": "/scan",
                "reason_code": "EMPTY_CLOUD",
                "recovery_trigger": "valid_cloud",
                "recovery_deadline_seconds": 1,
                "terminal_status": "recovered",
            },
            "invalid_suppressed_outputs",
        ),
    ],
)
def test_given_invalid_row_shape_when_loaded_then_boundary_rejects_it(
    tmp_path: Path,
    scenario_row: str | dict[str, str | int],
    reason: str,
) -> None:
    fixture = tmp_path / "faults.yaml"
    fixture.write_text(
        yaml.safe_dump({"scenarios": [scenario_row]}),
        encoding="utf-8",
    )

    with pytest.raises(FaultConfigurationError, match=reason):
        load_fault_scenarios(fixture)
