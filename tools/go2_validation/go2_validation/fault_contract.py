"""Software-only fault scenario YAML을 불변 계약으로 파싱한다."""

from dataclasses import dataclass
from pathlib import Path
from typing import Final

import yaml


KNOWN_REASONS: Final = frozenset(
    {
        "MALFORMED_LAYOUT",
        "EMPTY_CLOUD",
        "NAN_CLOUD",
        "STALE_CLOUD",
        "TF_UNAVAILABLE",
        "TIMESTAMP_REGRESSION",
        "ODOMETRY_JUMP",
        "STALE_ODOMETRY",
        "PROCESS_EXIT",
        "LAUNCH_FAILURE",
    }
)
KNOWN_FAULT_KINDS: Final = frozenset(
    {
        "malformed_layout",
        "empty_cloud",
        "nan_cloud",
        "stale_cloud",
        "tf_loss",
        "odom_regression",
        "odom_jump",
        "odom_loss",
        "process_exit",
        "launch_failure",
    }
)
KNOWN_TERMINAL_STATUSES: Final = frozenset({"suppressed", "recovered"})


@dataclass(frozen=True, slots=True)
class FaultScenario:
    """한 fault 주입의 입력·차단·복구 oracle이다."""

    scenario_id: str
    fault_kind: str
    suppressed_outputs: tuple[str, ...]
    reason_code: str
    recovery_trigger: str
    recovery_deadline_seconds: int
    terminal_status: str


@dataclass(frozen=True, slots=True)
class FaultConfigurationError(Exception):
    """Fault YAML이 실행 가능한 계약으로 파싱되지 않았음을 나타낸다."""

    reason_code: str

    def __str__(self) -> str:
        return self.reason_code


def load_fault_scenarios(path: Path) -> tuple[FaultScenario, ...]:
    """Load unique, complete scenarios from a YAML configuration boundary."""
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or not isinstance(
        document.get("scenarios"),
        list,
    ):
        raise FaultConfigurationError("missing_scenarios")
    scenarios: list[FaultScenario] = []
    for raw in document["scenarios"]:
        if not isinstance(raw, dict):
            raise FaultConfigurationError("invalid_scenario_row")
        scenarios.append(_parse_scenario(raw))
    identifiers = tuple(scenario.scenario_id for scenario in scenarios)
    if len(set(identifiers)) != len(identifiers):
        raise FaultConfigurationError("duplicate_scenario_id")
    return tuple(scenarios)


def _parse_scenario(raw: dict[str, str | int | list[str]]) -> FaultScenario:
    required = {
        "id",
        "fault_kind",
        "suppressed_outputs",
        "reason_code",
        "recovery_trigger",
        "recovery_deadline_seconds",
        "terminal_status",
    }
    missing = required.difference(raw)
    if "recovery_trigger" in missing:
        raise FaultConfigurationError("missing_recovery_trigger")
    if missing:
        raise FaultConfigurationError(f"missing_fields:{tuple(sorted(missing))}")
    raw_outputs = raw["suppressed_outputs"]
    if not isinstance(raw_outputs, list) or not all(
        isinstance(value, str) and value for value in raw_outputs
    ):
        raise FaultConfigurationError("invalid_suppressed_outputs")
    outputs = tuple(raw_outputs)
    deadline = raw["recovery_deadline_seconds"]
    scenario_id = _required_string(raw, "id")
    reason = _required_string(raw, "reason_code")
    fault_kind = _required_string(raw, "fault_kind")
    recovery_trigger = _required_string(raw, "recovery_trigger")
    terminal_status = _required_string(raw, "terminal_status")
    if not outputs:
        raise FaultConfigurationError("empty_suppressed_outputs")
    if not isinstance(deadline, int) or deadline <= 0:
        raise FaultConfigurationError("invalid_recovery_deadline")
    if reason not in KNOWN_REASONS:
        raise FaultConfigurationError("unknown_reason_code")
    if fault_kind not in KNOWN_FAULT_KINDS:
        raise FaultConfigurationError("unknown_fault_kind")
    if not recovery_trigger:
        raise FaultConfigurationError("empty_recovery_trigger")
    if terminal_status not in KNOWN_TERMINAL_STATUSES:
        raise FaultConfigurationError("unknown_terminal_status")
    return FaultScenario(
        scenario_id=scenario_id,
        fault_kind=fault_kind,
        suppressed_outputs=outputs,
        reason_code=reason,
        recovery_trigger=recovery_trigger,
        recovery_deadline_seconds=deadline,
        terminal_status=terminal_status,
    )


def _required_string(
    raw: dict[str, str | int | list[str]],
    field_name: str,
) -> str:
    value = raw[field_name]
    if not isinstance(value, str) or not value:
        raise FaultConfigurationError(f"invalid_string:{field_name}")
    return value
