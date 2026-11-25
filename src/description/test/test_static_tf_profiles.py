"""실물 static TF profile의 선택 경계를 검증한다."""

from pathlib import Path
from typing import Final

import pytest
import math
from xml.etree import ElementTree
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from launch_test_support import evaluate



from description.static_tf_profiles import StaticTfProfileError, load_static_tf_profile


PACKAGE_ROOT: Final = Path(__file__).parents[1]
PROFILE_PATH: Final = PACKAGE_ROOT / "config/static_tf_profiles.yaml"


def test_profile_transform_matches_canonical_urdf() -> None:
    origin = ElementTree.parse(PACKAGE_ROOT / "urdf/go2_description.urdf").find(
        ".//joint[@name='radar_joint']/origin")
    expected = [float(v) for v in origin.attrib["xyz"].split()]
    pitch = float(origin.attrib["rpy"].split()[1])
    profile = load_static_tf_profile(PROFILE_PATH, "project_default", "onboard")
    assert profile.translation_xyz_m == pytest.approx(expected)
    assert profile.quaternion_xyzw == pytest.approx(
        [0, math.sin(pitch / 2), 0, math.cos(pitch / 2)])


def test_given_unknown_profile_when_loaded_then_it_is_rejected() -> None:
    with pytest.raises(StaticTfProfileError, match="unknown_static_tf_profile"):
        load_static_tf_profile(PROFILE_PATH, "missing_profile", "onboard")


def test_allowed_mode_returns_selected_profile():
    assert load_static_tf_profile(
        PROFILE_PATH, "project_default", "onboard").profile_id == "project_default"


def test_disallowed_mode_raises_typed_error():
    with pytest.raises(StaticTfProfileError) as raised:
        load_static_tf_profile(PROFILE_PATH, "project_default", "unsupported")
    assert raised.value.reason_code == "unknown_static_tf_execution_mode"


@pytest.mark.parametrize("sim_time", ["true", "false"])
def test_static_tf_launch_evaluates_typed_time_and_geometry(sim_time) -> None:
    _, nodes, _ = evaluate("description", "go2_static_tf.launch.py",
                           {"use_sim_time": sim_time})
    assert {n["executable"] for n in nodes} == {
        "robot_state_publisher", "static_transform_publisher"}
    for node in nodes:
        assert node["parameters"][0]["use_sim_time"] is (sim_time == "true")
