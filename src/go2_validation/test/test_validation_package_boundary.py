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
        "live_navigation_acceptance",
        "shadow_fixture",
        "nav2_shadow_acceptance",
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
        "go2_nav2_shadow.launch.py",
        "go2_nav2_live_observer.launch.py",
        "go2_saved_map_localization.launch.py",
        "go2_slam_mapping.launch.py",
    }
    assert _children(NAV2_ROOT / "config") == {
        "nav2_non_actuating.yaml",
        "nav2_shadow.yaml",
        "navigation_contract.yaml",
        "saved_map_localization.yaml",
        "slam_mapping.yaml",
    }
    assert _children(NAV2_ROOT / "test") == {
        "test_navigation_configuration.py",
        "test_localization_shadow_configuration.py",
        "test_live_navigation_observer_configuration.py",
        "test_nav2_shadow_runtime_configuration.py",
        "test_shadow_assets.py",
    }


def test_given_package_metadata_when_read_then_validation_executables_have_one_owner() -> None:
    validation_setup = (VALIDATION_ROOT / "setup.py").read_text(encoding="utf-8")
    navigation_setup = (NAV2_ROOT / "setup.py").read_text(encoding="utf-8")

    assert all(f'"{executable} = ' in validation_setup for executable in EXECUTABLES)
    assert validation_setup.count("go2_validation.") == 7
    assert all(executable not in navigation_setup for executable in EXECUTABLES)
    assert "console_scripts" not in navigation_setup


def test_given_validation_package_when_replay_is_retired_then_no_rosbag_surface_remains() -> None:
    # Given: the active validation implementation and its ROS package manifest.
    implementation = "\n".join(
        path.read_text(encoding="utf-8")
        for path in sorted((VALIDATION_ROOT / "go2_validation").glob("*.py"))
    )
    manifest = (VALIDATION_ROOT / "package.xml").read_text(encoding="utf-8")

    # When: the package boundary is inspected after replay retirement.
    active_surface = implementation + manifest

    # Then: no rosbag runtime API or package dependency is retained.
    assert "rosbag2_" not in active_surface
    assert "external_replay" not in active_surface
