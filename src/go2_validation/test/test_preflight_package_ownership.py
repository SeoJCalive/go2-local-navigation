"""통합 preflight의 패키지 소유권과 비순환 의존성 계약을 검증한다."""

from pathlib import Path
from typing import Final
from xml.etree import ElementTree


PROJECT_ROOT: Final = Path(__file__).parents[3]
BRINGUP_MANIFEST: Final = PROJECT_ROOT / "src" / "bringup" / "package.xml"
GO2_NAV2_MANIFEST: Final = (
    PROJECT_ROOT / "src" / "go2_validation" / "package.xml"
)


def _execution_dependencies(manifest_path: Path) -> set[str]:
    root = ElementTree.parse(manifest_path).getroot()
    return {
        dependency.text
        for dependency in root.findall("exec_depend")
        if dependency.text is not None
    }


def test_preflight_composition_dependency_is_owned_by_go2_validation() -> None:
    # Given: bringup과 상위 조합 패키지의 ROS manifest
    bringup_dependencies = _execution_dependencies(BRINGUP_MANIFEST)
    navigation_dependencies = _execution_dependencies(GO2_NAV2_MANIFEST)

    # When: 통합 preflight의 의존 방향을 확인한다.
    dependency_direction = (
        "go2_validation" in bringup_dependencies,
        "bringup" in navigation_dependencies,
    )

    # Then: navigation만 bringup에 의존해 topological cycle이 생기지 않는다.
    assert dependency_direction == (False, True)
