"""Domain 65 NavigateToPose action과 feedback-gated cancel만 소유한다."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from time import monotonic
from typing import TYPE_CHECKING, Final, Protocol, TypeVar

import rclpy
from action_msgs.msg import GoalStatus

from go2_validation.shadow_scenarios import ShadowScenario, ShadowTerminalStatus

if TYPE_CHECKING:
    from geometry_msgs.msg import PoseStamped

    from go2_validation.shadow_observer import ShadowRuntimeObserver


FutureResult = TypeVar("FutureResult")
TERMINAL_BY_STATUS: Final = {
    GoalStatus.STATUS_SUCCEEDED: ShadowTerminalStatus.SUCCEEDED,
    GoalStatus.STATUS_CANCELED: ShadowTerminalStatus.CANCELED,
    GoalStatus.STATUS_ABORTED: ShadowTerminalStatus.ABORTED,
}
NAVFN_MAP_ORIGIN: Final = -3.0
NAVFN_MAP_RESOLUTION: Final = 0.5


class CompletedFuture(Protocol[FutureResult]):
    """Bounded spin에 필요한 ROS future의 최소 surface다."""

    def done(self) -> bool: ...

    def result(self) -> FutureResult: ...


@dataclass(frozen=True, slots=True)
class ShadowActionError(Exception):
    """Action server, timeout 또는 terminal 계약 위반 reason code다."""

    reason_code: str

    def __str__(self) -> str:
        return self.reason_code


def shadow_launch_command(
    map_path: Path | str,
) -> tuple[str, ...]:
    """시나리오 map만 주입하는 Nav2 launch argv를 만든다."""
    return (
        "ros2",
        "launch",
        "go2_nav2",
        "go2_nav2_shadow.launch.py",
        f"map:={map_path}",
    )


def shadow_fixture_command(scenario: ShadowScenario) -> tuple[str, ...]:
    """합성 fixture에 시나리오 ID만 주입하는 argv를 만든다."""
    return (
        "ros2",
        "run",
        "go2_validation",
        "shadow_fixture",
        "--ros-args",
        "-p",
        f"scenario_id:={scenario.scenario_id}",
    )


def run_navigation_action(
    observer: ShadowRuntimeObserver,
    scenario: ShadowScenario,
) -> ShadowTerminalStatus:
    """한 NavigateToPose goal을 보내고 terminal status까지 bounded하게 기다린다."""
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient

    client = ActionClient(observer, NavigateToPose, "navigate_to_pose")
    try:
        if not client.wait_for_server(timeout_sec=float(scenario.timeout_seconds)):
            raise ShadowActionError("shadow_action_server_unavailable")
        goal = NavigateToPose.Goal()
        goal.pose = _goal_pose(scenario)
        goal.behavior_tree = _behavior_tree_path(scenario)
        goal_handle = _wait_future(
            observer,
            client.send_goal_async(
                goal,
                feedback_callback=observer.record_feedback,
            ),
            scenario.timeout_seconds,
        )
        if not goal_handle.accepted:
            return ShadowTerminalStatus.ABORTED
        if scenario.scenario_id == "cancel":
            _wait_until(
                observer,
                observer.cancel_is_ready,
                scenario.timeout_seconds,
            )
            _wait_future(
                observer,
                goal_handle.cancel_goal_async(),
                scenario.timeout_seconds,
            )
        action_result = _wait_future(
            observer,
            goal_handle.get_result_async(),
            scenario.timeout_seconds,
        )
        return _terminal_status(action_result.status)
    finally:
        client.destroy()


def _goal_pose(scenario: ShadowScenario) -> PoseStamped:
    from geometry_msgs.msg import PoseStamped

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = (
        NAVFN_MAP_ORIGIN + scenario.goal_cell.x * NAVFN_MAP_RESOLUTION
    )
    pose.pose.position.y = (
        NAVFN_MAP_ORIGIN + scenario.goal_cell.y * NAVFN_MAP_RESOLUTION
    )
    pose.pose.orientation.w = 1.0
    return pose


def _behavior_tree_path(scenario: ShadowScenario) -> str:
    if scenario.scenario_id != "planner_failure":
        return ""
    from ament_index_python.packages import get_package_share_directory

    return str(
        Path(get_package_share_directory("go2_nav2"))
        / "behavior_trees/navigate_to_pose_shadow_missing_planner.xml"
    )


def _wait_future(
    observer: ShadowRuntimeObserver,
    future: CompletedFuture[FutureResult],
    timeout_seconds: int,
) -> FutureResult:
    deadline = monotonic() + timeout_seconds
    while rclpy.ok() and not future.done() and monotonic() < deadline:
        rclpy.spin_once(observer, timeout_sec=0.05)
    if not future.done():
        raise ShadowActionError("shadow_action_timeout")
    return future.result()


def _terminal_status(status: int) -> ShadowTerminalStatus:
    try:
        return TERMINAL_BY_STATUS[status]
    except KeyError as error:
        raise ShadowActionError("shadow_action_terminal_unknown") from error


def cancel_is_permitted(
    feedback_seen: bool,
    path_seen: bool,
    shadow_candidate_seen: bool,
) -> bool:
    """Cancel gate의 세 관찰 조건을 순수 boolean으로 판정한다."""
    return feedback_seen and path_seen and shadow_candidate_seen


def _wait_until(
    observer: ShadowRuntimeObserver,
    predicate: Callable[[], bool],
    timeout_seconds: int,
) -> None:
    deadline = monotonic() + timeout_seconds
    while rclpy.ok() and not predicate() and monotonic() < deadline:
        rclpy.spin_once(observer, timeout_sec=0.05)
    if not predicate():
        raise ShadowActionError("shadow_cancel_observation_timeout")
