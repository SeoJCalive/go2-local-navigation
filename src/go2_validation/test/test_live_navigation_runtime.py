"""Domain 0 live navigation launch 명령과 source 안전 경계를 검증한다."""

from pathlib import Path
from typing import Final

from go2_validation.live_navigation_runtime import live_navigation_launch_command

PROJECT_ROOT: Final = Path(__file__).parents[3]
LIVE_LAUNCH_PATH: Final = (
    PROJECT_ROOT / "src/go2_nav2/launch/go2_nav2_live_observer.launch.py"
)
OBSERVER_PATH: Final = (
    PROJECT_ROOT
    / "src/go2_validation/go2_validation/live_navigation_runtime_observer.py"
)
RUNNER_PATH: Final = (
    PROJECT_ROOT
    / "src/go2_validation/go2_validation/live_navigation_acceptance_runner.py"
)


def test_given_live_map_when_launch_command_is_built_then_profile_is_explicit() -> None:
    # Given: 같은 임시 배치에서 저장한 live occupancy map
    map_path = Path("data/runs/live_mapping/run/artifacts/occupancy.yaml")

    # When: bounded live observer launch argv를 만든다.
    command = live_navigation_launch_command(map_path)

    # Then: validation-only live launch와 map path가 shell 없이 명시된다.
    assert command == (
        "ros2",
        "launch",
        "go2_nav2",
        "go2_nav2_live_observer.launch.py",
        f"map:={map_path}",
    )


def test_given_live_launch_when_inspected_then_output_is_inert_and_goal_is_absent() -> None:
    # Given: 실제 sensor·AMCL과 전체 Nav2를 결합할 validation-only launch
    assert LIVE_LAUNCH_PATH.is_file()
    launch_source = LIVE_LAUNCH_PATH.read_text(encoding="utf-8")

    # When: machine-consumed include, profile, output 경계를 읽는다.
    # Then: real time·onboard profile과 inert velocity만 허용한다.
    assert "go2_saved_map_localization.launch.py" in launch_source
    assert '"use_sim_time": "false"' in launch_source
    assert '"continuity_profile": "onboard_observe"' in launch_source
    assert '"execution_mode": "onboard"' in launch_source
    assert '"cmd_vel", "/go2_nav2/shadow_cmd_vel"' in launch_source
    assert 'package="go2_control"' not in launch_source
    assert "/api/sport/request" not in launch_source
    assert "/lowcmd" not in launch_source


def test_given_live_validation_code_when_inspected_then_it_cannot_send_goal() -> None:
    # Given: bounded runner와 read-only observer source
    assert OBSERVER_PATH.is_file()
    assert RUNNER_PATH.is_file()
    source = OBSERVER_PATH.read_text(encoding="utf-8") + RUNNER_PATH.read_text(
        encoding="utf-8"
    )

    # When: action과 command write API 사용 여부를 읽는다.
    # Then: observer는 graph·message를 읽을 뿐 goal이나 command client가 없다.
    assert "ActionClient" not in source
    assert "send_goal" not in source
    assert "create_publisher" not in source
