"""Domain 65 fixture의 단일 clock·TF·synthetic velocity 계약을 정의한다."""

from dataclasses import dataclass
from typing import Final

from go2_validation.shadow_scenarios import (
    ShadowScenarioError,
    ShadowTerminalStatus,
)


@dataclass(frozen=True, slots=True)
class ShadowFixturePlan:
    expected_terminal: ShadowTerminalStatus
    integrates_shadow_velocity: bool
    clock_topic: str = "/clock"
    map_to_odom_owner: str = "fixture"
    odom_to_base_owner: str = "fixture"
    shadow_velocity_topic: str = "/go2_nav2/shadow_cmd_vel"


PLANS: Final = {
    "success": ShadowFixturePlan(ShadowTerminalStatus.SUCCEEDED, True),
    "cancel": ShadowFixturePlan(ShadowTerminalStatus.CANCELED, True),
    "blocked_goal": ShadowFixturePlan(ShadowTerminalStatus.ABORTED, True),
    "outside_map_goal": ShadowFixturePlan(ShadowTerminalStatus.ABORTED, True),
    "planner_failure": ShadowFixturePlan(ShadowTerminalStatus.ABORTED, True),
    "no_progress": ShadowFixturePlan(ShadowTerminalStatus.ABORTED, False),
}


def fixture_plan_for(scenario_id: str) -> ShadowFixturePlan:
    try:
        return PLANS[scenario_id]
    except KeyError as error:
        raise ShadowScenarioError("shadow_fixture_scenario_unknown") from error
