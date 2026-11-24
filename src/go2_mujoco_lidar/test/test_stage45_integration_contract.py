from pathlib import Path
from typing import Final


PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]


def test_launch_keeps_onboard_mode_and_uses_planar_selected_cloud() -> None:
    source = (PACKAGE_ROOT / "launch" / "go2_mujoco_stage45.launch.py").read_text(
        encoding="utf-8"
    )

    assert '"execution_mode": "onboard"' in source
    assert 'PLANAR_TOPIC = "/simulation/utlidar/cloud_planar_selected"' in source
    assert '"raw_cloud_topic": PLANAR_TOPIC' in source
    assert "converter_min_height" not in source
