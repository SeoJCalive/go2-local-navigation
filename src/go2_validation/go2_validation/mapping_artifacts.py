
"""SLAM Toolbox 저장 파일의 경로·참조·크기·SHA-256 경계를 검증한다.
Occupancy YAML은 같은 directory의 image만 참조해야 하며 pose graph 두 파일도
비어 있지 않아야 한다. 반환값은 Todo 12 result와 Stage 13 freeze에서 재사용한다.
"""

from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path

import yaml


@dataclass(frozen=True, slots=True)
class MappingArtifactError(Exception):
    """저장 파일 집합이 재로딩 가능한 경계를 충족하지 못했다."""

    reason_code: str
    detail: str

    def __str__(self) -> str:
        return f"{self.reason_code}: {self.detail}"


@dataclass(frozen=True, slots=True)
class SavedMappingArtifacts:
    """검증된 occupancy와 pose graph 파일·checksum 집합이다."""

    image_path: Path
    yaml_path: Path
    pose_graph_data_path: Path
    pose_graph_path: Path
    checksums: tuple[tuple[str, str], ...]

    @property
    def paths(self) -> tuple[Path, ...]:
        """Checksum 순서와 같은 네 artifact 경로를 반환한다."""
        return (
            self.image_path,
            self.yaml_path,
            self.pose_graph_data_path,
            self.pose_graph_path,
        )


def validate_saved_mapping_artifacts(root: Path) -> SavedMappingArtifacts:
    """Map YAML을 parse하고 모든 참조와 nonzero file을 typed 결과로 만든다."""
    yaml_path = _required_file(root / "occupancy.yaml")
    pose_data = _required_file(root / "pose_graph.data")
    pose_graph = _required_file(root / "pose_graph.posegraph")
    try:
        document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise MappingArtifactError("mapping_yaml_invalid", str(error)) from error
    if not isinstance(document, dict):
        raise MappingArtifactError("mapping_yaml_invalid", "root_not_mapping")
    image_value = document.get("image")
    if not isinstance(image_value, str) or Path(image_value).name != image_value:
        raise MappingArtifactError("mapping_image_reference_invalid", str(image_value))
    resolution = document.get("resolution")
    origin = document.get("origin")
    if not isinstance(resolution, (int, float)) or resolution <= 0:
        raise MappingArtifactError("mapping_resolution_invalid", str(resolution))
    if not isinstance(origin, list) or len(origin) != 3:
        raise MappingArtifactError("mapping_origin_invalid", str(origin))
    image_path = _required_file(root / image_value)
    paths = (image_path, yaml_path, pose_data, pose_graph)
    checksums = tuple((path.name, _sha256(path)) for path in paths)
    return SavedMappingArtifacts(
        image_path=image_path,
        yaml_path=yaml_path,
        pose_graph_data_path=pose_data,
        pose_graph_path=pose_graph,
        checksums=checksums,
    )


def normalize_occupancy_image_reference(root: Path) -> None:
    """Map saver의 absolute image 참조를 atomic directory 이동용 basename으로 바꾼다."""
    yaml_path = root / "occupancy.yaml"
    try:
        document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise MappingArtifactError("mapping_yaml_invalid", str(error)) from error
    if not isinstance(document, dict):
        raise MappingArtifactError("mapping_yaml_invalid", "root_not_mapping")
    image_value = document.get("image")
    if not isinstance(image_value, str):
        raise MappingArtifactError("mapping_image_reference_invalid", str(image_value))
    document["image"] = Path(image_value).name
    temporary = yaml_path.with_suffix(".yaml.tmp")
    temporary.write_text(
        yaml.safe_dump(document, sort_keys=False),
        encoding="utf-8",
    )
    temporary.replace(yaml_path)


def _required_file(path: Path) -> Path:
    if not path.is_file() or path.is_symlink() or path.stat().st_size <= 0:
        raise MappingArtifactError("mapping_artifact_missing", str(path))
    return path


def _sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()
