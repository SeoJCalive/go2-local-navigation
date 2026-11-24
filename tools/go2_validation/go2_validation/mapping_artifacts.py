
"""저장 지도 YAML과 image의 SHA-256 identity를 계산한다."""

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


def saved_map_checksum(map_path: Path) -> str:
    """Map YAML과 같은 디렉터리의 image bytes를 하나의 identity로 만든다."""
    try:
        document = yaml.safe_load(map_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise MappingArtifactError("mapping_yaml_invalid", str(error)) from error
    if not isinstance(document, dict):
        raise MappingArtifactError("mapping_yaml_invalid", "root_not_mapping")
    image_value = document.get("image")
    if not isinstance(image_value, str) or Path(image_value).name != image_value:
        raise MappingArtifactError("mapping_image_reference_invalid", str(image_value))
    image_path = _required_file(map_path.parent / image_value)
    return _sha256_bytes((map_path, image_path))


def _required_file(path: Path) -> Path:
    if not path.is_file() or path.is_symlink() or path.stat().st_size <= 0:
        raise MappingArtifactError("mapping_artifact_missing", str(path))
    return path


def _sha256_bytes(paths: tuple[Path, ...]) -> str:
    digest = sha256()
    for path in paths:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    return digest.hexdigest()
