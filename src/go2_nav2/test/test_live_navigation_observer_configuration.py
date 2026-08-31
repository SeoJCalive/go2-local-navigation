"""Domain 0 live observer launch의 비동작 구성 계약을 검증한다."""

from pathlib import Path
from typing import Final

LAUNCH_PATH: Final = (
    Path(__file__).parents[1] / "launch/go2_nav2_live_observer.launch.py"
)


def test_given_live_observer_launch_when_read_then_localization_and_nav2_are_composed() -> None:
    # Given: 최종 고정 전 실제 입력을 소비할 validation-only launch
    assert LAUNCH_PATH.is_file()
    source = LAUNCH_PATH.read_text(encoding="utf-8")

    # When: child package와 lifecycle 경계를 읽는다.
    # Then: AMCL launch와 네 Nav2 server가 있고 synthetic fixture는 없다.
    assert "go2_saved_map_localization.launch.py" in source
    assert 'package="nav2_planner"' in source
    assert 'package="nav2_controller"' in source
    assert 'package="nav2_bt_navigator"' in source
    assert 'package="nav2_behaviors"' in source
    assert "synthetic_navigation_fixture" not in source
    assert '"map_server"' not in source.split('"node_names":', maxsplit=1)[1]
