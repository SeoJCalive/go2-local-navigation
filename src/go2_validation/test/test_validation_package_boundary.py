from pathlib import Path
from typing import Final


PROJECT_ROOT: Final = Path(__file__).parents[3]
NAV2_ROOT: Final = PROJECT_ROOT / "src" / "go2_nav2"
VALIDATION_ROOT: Final = PROJECT_ROOT / "src" / "go2_validation"
EXECUTABLES: Final = frozenset(
    {
        "integrated_preflight",
        "navigation_runtime_preflight",
        "fault_fixture",
        "fault_acceptance",
        "mapping_input_acceptance",
        "mapping_acceptance",
        "mapping_tf_profile_ab",
        "mapping_scan_profile_ab",
        "external_replay_acquisition",
        "external_replay_convert",
    }
)


def _children(path: Path) -> set[str]:
    return {entry.name for entry in path.iterdir() if entry.is_file()}


def test_given_validation_extraction_when_package_layout_is_read_then_runtime_and_validation_owners_are_disjoint() -> None:
    assert (VALIDATION_ROOT / "resource/go2_validation").is_file()
    assert (VALIDATION_ROOT / "go2_validation/__init__.py").is_file()
    assert (VALIDATION_ROOT / "launch/go2_fault_acceptance.launch.py").is_file()
    assert (VALIDATION_ROOT / "launch/go2_integrated_preflight.launch.py").is_file()
    assert _children(NAV2_ROOT / "go2_nav2") == {"__init__.py"}
    assert _children(NAV2_ROOT / "launch") == {
        "go2_costmap_only.launch.py",
        "go2_controller_preview.launch.py",
        "go2_slam_mapping.launch.py",
    }
    assert _children(NAV2_ROOT / "config") == {
        "nav2_non_actuating.yaml",
        "navigation_contract.yaml",
        "slam_mapping.yaml",
    }
    assert _children(NAV2_ROOT / "test") == {
        "test_navigation_configuration.py",
        "test_shadow_assets.py",
    }


def test_given_package_metadata_when_read_then_validation_executables_have_one_owner() -> None:
    validation_setup = (VALIDATION_ROOT / "setup.py").read_text(encoding="utf-8")
    navigation_setup = (NAV2_ROOT / "setup.py").read_text(encoding="utf-8")

    assert all(f'"{executable} = ' in validation_setup for executable in EXECUTABLES)
    assert validation_setup.count("go2_validation.") == 10
    assert all(executable not in navigation_setup for executable in EXECUTABLES)
    assert "console_scripts" not in navigation_setup


def test_given_mapping_argv_when_profile_scope_is_selected_then_mode_and_continuity_are_explicit() -> None:
    command_builder = (
        VALIDATION_ROOT / "go2_validation/mapping_command_builders.py"
    ).read_text(encoding="utf-8")
    mapping_launch = (NAV2_ROOT / "launch/go2_slam_mapping.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'execution_mode: str = "onboard"' in command_builder
    assert 'continuity_profile: str = "onboard_observe"' in command_builder
    assert 'f"execution_mode:={configuration.execution_mode}"' in command_builder
    assert 'f"continuity_profile:={configuration.continuity_profile}"' in command_builder
    assert 'DeclareLaunchArgument("continuity_profile", default_value="onboard_observe")' in mapping_launch
    assert '"continuity_profile": continuity_profile' in mapping_launch
