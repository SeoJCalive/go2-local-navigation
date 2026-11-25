"""Evaluate installed no-goal observer composition without starting nodes."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from launch_test_support import evaluate


def test_live_observer_composes_localization_and_navigation():
    _, nodes, includes = evaluate("go2_nav2", "go2_nav2_live_observer.launch.py")
    assert {n["package"] for n in nodes} == {
        "nav2_planner", "nav2_controller", "nav2_bt_navigator",
        "nav2_behaviors", "nav2_lifecycle_manager"}
    assert len(includes) == 1
    assert "go2_saved_map_localization" in includes[0]["path"]
    manager = next(n for n in nodes if n["package"] == "nav2_lifecycle_manager")
    assert set(manager["parameters"][-1]["node_names"]) == {
        "planner_server", "controller_server", "behavior_server", "bt_navigator"}
