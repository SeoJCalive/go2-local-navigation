"""실물 static TF profile의 선택 경계를 검증한다."""

from pathlib import Path
from typing import Final

import pytest
import yaml

from bringup.static_tf_profiles import StaticTfProfileError, load_static_tf_profile


PACKAGE_ROOT: Final = Path(__file__).parents[1]
PROFILE_PATH: Final = PACKAGE_ROOT / "config/static_tf_profiles.yaml"
LAUNCH_PATH: Final = PACKAGE_ROOT / "launch/go2_static_tf.launch.py"


def test_given_profile_registry_when_loaded_then_project_transform_is_registered() -> None:
    document = yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8"))
    profiles = document["static_tf_profiles"]["profiles"]

    assert document["static_tf_profiles"]["default_profile"] == "project_default"
    assert profiles["project_default"]["scope"] == "onboard_only"
    assert profiles["project_default"]["translation_xyz_m"] == [0.28945, 0.0, -0.046825]


@pytest.mark.parametrize(
    ("profile_id", "execution_mode"),
    (
        ("project_default", "onboard"),
    ),
)
def test_given_allowed_mode_when_profile_loads_then_selected_profile_is_returned(
    profile_id: str,
    execution_mode: str,
) -> None:
    assert load_static_tf_profile(
        PROFILE_PATH,
        profile_id,
        execution_mode,
    ).profile_id == profile_id


@pytest.mark.parametrize(
    ("profile_id", "execution_mode", "reason_code"),
    (
        ("project_default", "unsupported", "unknown_static_tf_execution_mode"),
    ),
)
def test_given_disallowed_mode_when_profile_loads_then_typed_error_is_raised(
    profile_id: str,
    execution_mode: str,
    reason_code: str,
) -> None:
    with pytest.raises(StaticTfProfileError) as raised:
        load_static_tf_profile(PROFILE_PATH, profile_id, execution_mode)

    assert raised.value.reason_code == reason_code


def test_given_unknown_profile_when_loaded_then_it_is_rejected() -> None:
    with pytest.raises(StaticTfProfileError, match="unknown_static_tf_profile"):
        load_static_tf_profile(PROFILE_PATH, "missing_profile", "onboard")


def test_given_static_tf_launch_when_read_then_mode_and_sim_time_reach_nodes() -> None:
    source = LAUNCH_PATH.read_text(encoding="utf-8")

    assert 'default_value="project_default"' in source
    assert 'DeclareLaunchArgument("execution_mode", default_value="onboard")' in source
    assert 'DeclareLaunchArgument("use_sim_time", default_value="false")' in source
    assert "load_static_tf_profile(Path(profile_path), profile_id, execution_mode)" in source
    assert '"use_sim_time": ParameterValue(use_sim_time, value_type=bool)' in source
