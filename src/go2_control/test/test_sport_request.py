import json

from go2_control.motion_contract import MotionCommand
from go2_control.sport_request import build_move_request, build_stop_request


def test_bounded_command_uses_official_move_schema() -> None:
    request = build_move_request(
        MotionCommand(velocity_x=0.12, velocity_y=-0.03, yaw_rate=0.2)
    )

    assert request.api_id == 1008
    assert json.loads(request.parameter) == {"x": 0.12, "y": -0.03, "z": 0.2}


def test_stop_request_uses_official_api_without_parameter() -> None:
    request = build_stop_request()

    assert request.api_id == 1003
    assert request.parameter == ""
