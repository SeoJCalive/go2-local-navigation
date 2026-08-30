from math import isclose, sqrt
from pathlib import Path
from typing import Final

import pytest
import yaml


PACKAGE_ROOT: Final = Path(__file__).parents[1]
PROFILE_PATH: Final = PACKAGE_ROOT / "config" / "static_tf_profiles.yaml"
LAUNCH_PATH: Final = PACKAGE_ROOT / "launch" / "go2_static_tf.launch.py"


def test_given_static_tf_profiles_when_loaded_then_default_and_dimos_are_separate() -> None:
    # Given: 실물 기본값과 외부 replay 값을 분리한 machine-readable profile
    assert PROFILE_PATH.is_file()

    # When: 두 profile의 frame·수치·출처를 읽는다.
    document = yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8"))
    profiles = document["static_tf_profiles"]["profiles"]
    project_default = profiles["project_default"]
    dimos_replay = profiles["dimos_replay"]

    # Then: 기존 기본값은 유지되고 DimOS 값은 외부 replay로만 표시된다.
    assert document["static_tf_profiles"]["default_profile"] == "project_default"
    assert project_default["translation_xyz_m"] == [0.28945, 0.0, -0.046825]
    assert dimos_replay["scope"] == "external_replay_only"
    assert dimos_replay["translation_xyz_m"] == [0.28216, 0.0, -0.02467]
    assert dimos_replay["source"]["commit"] == (
        "b4cd9789cc68876adf87ff40a404677f127d69bf"
    )
    quaternion = dimos_replay["quaternion_xyzw"]
    assert isclose(sqrt(sum(value * value for value in quaternion)), 1.0, abs_tol=1e-7)


def test_given_profile_loader_when_profile_is_unknown_then_it_rejects_the_launch() -> None:
    # Given: 설치 config와 등록되지 않은 profile ID
    from bringup.static_tf_profiles import StaticTfProfileError, load_static_tf_profile

    # When/Then: 기본값으로 조용히 fallback하지 않는다.
    with pytest.raises(StaticTfProfileError, match="unknown_static_tf_profile"):
        load_static_tf_profile(PROFILE_PATH, "missing_profile", "onboard")


@pytest.mark.parametrize(
    ("profile_id", "execution_mode"),
    (
        ("project_default", "onboard"),
        ("project_default", "external_replay"),
        ("dimos_replay", "external_replay"),
    ),
)
def test_given_allowed_execution_mode_when_static_tf_profile_loads_then_it_is_available(
    profile_id: str,
    execution_mode: str,
) -> None:
    # Given: generic project profile와 external-replay 전용 DimOS profile의 허용 mode
    from bringup.static_tf_profiles import load_static_tf_profile

    # When: launch 경계 loader가 profile과 execution mode를 함께 파싱한다.
    profile = load_static_tf_profile(PROFILE_PATH, profile_id, execution_mode)

    # Then: 허용 조합만 선택된 profile ID를 보존한다.
    assert profile.profile_id == profile_id


@pytest.mark.parametrize(
    ("profile_id", "execution_mode", "reason_code"),
    (
        (
            "dimos_replay",
            "onboard",
            "static_tf_profile_external_replay_required",
        ),
        (
            "project_default",
            "unsupported_mode",
            "unknown_static_tf_execution_mode",
        ),
    ),
)
def test_given_disallowed_execution_mode_when_static_tf_profile_loads_then_typed_error_is_raised(
    profile_id: str,
    execution_mode: str,
    reason_code: str,
) -> None:
    # Given: replay-only scope 또는 등록되지 않은 execution mode
    from bringup.static_tf_profiles import StaticTfProfileError, load_static_tf_profile

    # When/Then: launch 전에 typed loader error로 거부한다.
    with pytest.raises(StaticTfProfileError) as raised:
        load_static_tf_profile(PROFILE_PATH, profile_id, execution_mode)

    assert raised.value.reason_code == reason_code


def test_given_static_tf_launch_when_read_then_profile_selection_is_machine_consumed() -> None:
    # Given: static TF launch source
    launch_source = LAUNCH_PATH.read_text(encoding="utf-8")

    # When/Then: profile argument가 YAML loader를 거쳐 publisher 값으로 연결된다.
    assert "static_tf_profiles.yaml" in launch_source
    assert "DeclareLaunchArgument" in launch_source
    assert '"sensor_tf_profile"' in launch_source
    assert 'default_value="project_default"' in launch_source
    assert '"execution_mode"' in launch_source
    assert 'default_value="onboard"' in launch_source
    assert 'LaunchConfiguration("execution_mode")' in launch_source
    assert (
        "load_static_tf_profile(Path(profile_path), profile_id, execution_mode)"
        in launch_source
    )
    assert "load_static_tf_profile" in launch_source
    assert '"--qx"' in launch_source
    assert '"--qw"' in launch_source
