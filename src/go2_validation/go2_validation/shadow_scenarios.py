"""Domain 65 시나리오 manifest를 불변 runtime 입력으로 파싱한다."""

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Final

import yaml
from bringup.preflight_result import JsonDocument, JsonValue


class ShadowTerminalStatus(str, Enum):
    SUCCEEDED = "SUCCEEDED"
    CANCELED = "CANCELED"
    ABORTED = "ABORTED"


@dataclass(frozen=True, slots=True)
class ShadowScenarioError(Exception):
    reason_code: str

    def __str__(self) -> str:
        return self.reason_code


@dataclass(frozen=True, slots=True)
class GridCell:
    x: int
    y: int


@dataclass(frozen=True, slots=True)
class ShadowScenario:
    scenario_id: str
    map_id: str
    map_relative_path: str
    start_cell: GridCell
    goal_cell: GridCell
    expected_terminal: ShadowTerminalStatus
    timeout_seconds: int
    expects_path: bool
    expects_candidate: bool


EXPECTED_SCENARIO_IDS: Final = frozenset(
    {
        "success",
        "cancel",
        "blocked_goal",
        "outside_map_goal",
        "planner_failure",
        "no_progress",
    }
)


def load_shadow_scenarios(path: Path) -> tuple[ShadowScenario, ...]:
    document: JsonValue = yaml.safe_load(path.read_text(encoding="utf-8"))
    try:
        root = _required_mapping(document, "root")
        maps = _required_mapping(root["maps"], "maps")
        raw_scenarios = _required_list(root["scenarios"], "scenarios")
        scenarios = tuple(
            _parse_scenario(_required_mapping(raw, "scenario"), maps)
            for raw in raw_scenarios
        )
    except (KeyError, ValueError) as error:
        raise ShadowScenarioError("shadow_scenario_manifest_invalid") from error
    if {scenario.scenario_id for scenario in scenarios} != EXPECTED_SCENARIO_IDS:
        raise ShadowScenarioError("shadow_scenario_ids_invalid")
    return scenarios


def shadow_scenario(path: Path, scenario_id: str) -> ShadowScenario:
    for scenario in load_shadow_scenarios(path):
        if scenario.scenario_id == scenario_id:
            return scenario
    raise ShadowScenarioError("shadow_scenario_unknown")


def _parse_scenario(
    raw: JsonDocument,
    maps: JsonDocument,
) -> ShadowScenario:
    start = _required_mapping(raw["start_cell"], "start_cell")
    goal = _required_mapping(raw["goal_cell"], "goal_cell")
    scenario_id = _required_string(raw["id"], "id")
    map_id = _required_string(raw["map_id"], "map_id")
    path_expectation = _required_string(raw["path_expectation"], "path_expectation")
    candidate_expectation = _required_string(
        raw["candidate_expectation"],
        "candidate_expectation",
    )
    if path_expectation not in {"present", "absent"}:
        raise ShadowScenarioError("shadow_scenario_path_expectation_invalid")
    if candidate_expectation not in {"present", "absent"}:
        raise ShadowScenarioError("shadow_scenario_candidate_expectation_invalid")
    return ShadowScenario(
        scenario_id=scenario_id,
        map_id=map_id,
        map_relative_path=_required_string(maps[map_id], "map_path"),
        start_cell=GridCell(
            x=_required_integer(start["x"], "start_x"),
            y=_required_integer(start["y"], "start_y"),
        ),
        goal_cell=GridCell(
            x=_required_integer(goal["x"], "goal_x"),
            y=_required_integer(goal["y"], "goal_y"),
        ),
        expected_terminal=ShadowTerminalStatus(
            _required_string(
                raw["expected_action_terminal_status"],
                "expected_action_terminal_status",
            )
        ),
        timeout_seconds=_required_integer(raw["timeout_sec"], "timeout_sec"),
        expects_path=path_expectation == "present",
        expects_candidate=candidate_expectation == "present",
    )


def _required_mapping(value: JsonValue, field_name: str) -> JsonDocument:
    if not isinstance(value, dict):
        raise ShadowScenarioError(f"shadow_scenario_{field_name}_mapping_invalid")
    return value


def _required_list(value: JsonValue, field_name: str) -> list[JsonValue]:
    if not isinstance(value, list):
        raise ShadowScenarioError(f"shadow_scenario_{field_name}_list_invalid")
    return value


def _required_string(value: JsonValue, field_name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ShadowScenarioError(f"shadow_scenario_{field_name}_string_invalid")
    return value


def _required_integer(value: JsonValue, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ShadowScenarioError(f"shadow_scenario_{field_name}_integer_invalid")
    return value
