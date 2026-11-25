"""Unitree Sport request를 만드는 ROS 비의존 경계."""

from dataclasses import dataclass
import json
from typing import Final

from go2_control.motion_contract import MotionCommand


MOVE_API_ID: Final = 1008
STOP_MOVE_API_ID: Final = 1003


@dataclass(frozen=True, slots=True)
class SportRequestData:
    api_id: int
    parameter: str


def build_move_request(command: MotionCommand) -> SportRequestData:
    """Build a Move request payload."""
    return SportRequestData(
        api_id=MOVE_API_ID,
        parameter=json.dumps(
            {
                "x": command.velocity_x,
                "y": command.velocity_y,
                "z": command.yaw_rate,
            },
            allow_nan=False,
            separators=(",", ":"),
        ),
    )


def build_stop_request() -> SportRequestData:
    """Build a StopMove request payload."""
    return SportRequestData(api_id=STOP_MOVE_API_ID, parameter="")
