"""Check sim rotation settings against D01 response and Humble 1.1.20 semantics.

The measured negative response needs more than 0.3 rad/s; both signs respond at
0.5. Configuration checks protect that contract; real qpos heading convergence
and collision handling are covered by the separate isolated acceptance run.
BT action RPC waits must also cover the controller's cancel polling cycle;
the acceptance run checks actual scheduling, not just this budget.
"""

from pathlib import Path

import yaml


def test_shim_reaches_responsive_band_in_one_control_cycle():
    params = yaml.safe_load(
        (Path(__file__).parents[1] / "config/nav2_mujoco.yaml").read_text()
    )["controller_server"]["ros__parameters"]
    controller = params["FollowPath"]
    goal = params["general_goal_checker"]
    assert controller["plugin"] == "nav2_rotation_shim_controller::RotationShimController"
    assert controller["primary_controller"] == "dwb_core::DWBLocalPlanner"
    assert controller["rotate_to_goal_heading"]
    floor = controller["rotate_to_heading_angular_vel"]
    assert 0.5 <= floor <= controller["max_vel_theta"]
    # The installed shim clamps desired speed to odom +/- acceleration/frequency.
    assert (controller["max_angular_accel"] / params["controller_frequency"]
            >= controller["max_vel_theta"] + floor)
    # At 10 Hz even commanded (faster than measured) rotation crosses less than
    # half the two-sided goal window, so the checker can stop before reversal.
    assert floor / params["controller_frequency"] < goal["yaw_goal_tolerance"]
    assert controller["angular_disengage_threshold"] < controller["angular_dist_threshold"]
    assert controller["xy_goal_tolerance"] == goal["xy_goal_tolerance"]
    # XYThetaIterator's minimum-speed rejection combines XY and theta with AND.
    assert 0 < controller["min_speed_xy"] < controller["max_vel_x"]
    assert controller["min_speed_theta"] >= floor
    # Measured worst-sign response, rather than candidate command, clears progress.
    progress = params["progress_checker"]
    assert 0.362 * progress["movement_time_allowance"] > progress["required_movement_angle"]


def test_bt_action_deadline_covers_control_cycle():
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "config/nav2_mujoco.yaml").read_text()
    )
    controller = config["controller_server"]["ros__parameters"]
    bt = config["bt_navigator"]["ros__parameters"]
    # ControllerServer checks cancellation once per WallRate cycle. The BT wait
    # must outlast that cycle before allowing for transport/scheduling overhead.
    required_seconds = 1 / controller["controller_frequency"]
    assert bt["default_server_timeout"] / 1000 > required_seconds
