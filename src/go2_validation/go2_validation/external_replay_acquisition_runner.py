
"""Pinned 외부 replay를 로컬 custody에 안전하게 취득하고 결과 JSON을 남긴다."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime
import json
import os
from pathlib import Path
import tempfile
from typing import Literal, Protocol

from go2_validation.external_replay_acquisition import (
    ensure_initial_space,
    extract_single_mcap,
    file_sha256,
    promote_staged_file,
    promote_verified_archive,
    verify_file,
    verify_regular_size,
)
from go2_validation.external_replay_contract import (
    AcquisitionConflict,
    AcquisitionDeferred,
    SourceSpec,
)
from go2_validation.external_replay_download import download_archive_with_curl
from go2_validation.external_replay_manifest import load_source_spec


AcquisitionStatus = Literal["passed", "deferred", "conflict"]


class ArchiveDownloader(Protocol):
    """수신 byte cap과 source identity를 보존하는 다운로드 경계다."""

    def __call__(self, spec: SourceSpec, destination: Path) -> str:
        ...


@dataclass(frozen=True, slots=True)
class AcquisitionPaths:
    """한 source의 archive, 추출물, staging 위치를 함께 보존한다."""

    source_root: Path
    staging_root: Path
    archive_path: Path
    extracted_path: Path

    @classmethod
    def from_cache_root(cls, cache_root: Path, spec: SourceSpec) -> "AcquisitionPaths":
        """검증된 filename만 사용해 custody 경로를 만든다."""
        source_root = cache_root / "source"
        return cls(
            source_root=source_root,
            staging_root=cache_root / "staging",
            archive_path=source_root / spec.archive_filename,
            extracted_path=source_root / spec.extracted_filename,
        )


@dataclass(frozen=True, slots=True)
class AcquisitionResult:
    """후속 변환기가 소비하는 취득 상태와 실제 checksum이다."""

    status: AcquisitionStatus
    source_id: str
    reason_code: str | None
    detail: str | None
    artifact_absent: bool
    archive_path: str | None
    archive_size_bytes: int | None
    archive_sha256: str | None
    extracted_path: str | None
    extracted_size_bytes: int | None
    extracted_sha256: str | None


def acquire_external_replay(
    spec: SourceSpec,
    paths: AcquisitionPaths,
    downloader: ArchiveDownloader,
) -> AcquisitionResult:
    """필요한 artifact만 취득하며 모든 temporary directory를 회수한다."""
    paths.source_root.mkdir(parents=True, exist_ok=True)
    paths.staging_root.mkdir(parents=True, exist_ok=True)
    try:
        if not paths.archive_path.exists() or not paths.extracted_path.exists():
            ensure_initial_space(paths.staging_root, spec.minimum_free_bytes)
        archive_digest = _obtain_archive(spec, paths, downloader)
        extracted_digest = _obtain_extracted(spec, paths)
    except AcquisitionDeferred as error:
        return _failure_result("deferred", spec.source_id, error)
    except AcquisitionConflict as error:
        return _failure_result("conflict", spec.source_id, error)
    return AcquisitionResult(
        status="passed",
        source_id=spec.source_id,
        reason_code=None,
        detail=None,
        artifact_absent=False,
        archive_path=str(paths.archive_path),
        archive_size_bytes=spec.archive_size_bytes,
        archive_sha256=archive_digest,
        extracted_path=str(paths.extracted_path),
        extracted_size_bytes=spec.extracted_size_bytes,
        extracted_sha256=extracted_digest,
    )


def write_acquisition_result(path: Path, result: AcquisitionResult) -> None:
    """결과 JSON을 partial file 없이 atomic replace로 기록한다."""
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_name(f".{path.name}.partial")
    partial.unlink(missing_ok=True)
    document = {
        "schema_version": 1,
        "record_kind": "external_replay_acquisition_result",
        "recorded_at": datetime.now().astimezone().isoformat(),
        **asdict(result),
    }
    try:
        with partial.open("x", encoding="utf-8") as output:
            json.dump(document, output, ensure_ascii=False, indent=2)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(partial, path)
        descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    except OSError as error:
        partial.unlink(missing_ok=True)
        raise AcquisitionConflict("result_write_failure", str(error)) from error


def _obtain_archive(
    spec: SourceSpec,
    paths: AcquisitionPaths,
    downloader: ArchiveDownloader,
) -> str:
    if paths.archive_path.exists():
        return verify_file(
            paths.archive_path,
            spec.archive_size_bytes,
            spec.archive_sha256,
        )
    with tempfile.TemporaryDirectory(prefix="acquire-", dir=paths.staging_root) as raw:
        staged = Path(raw) / spec.archive_filename
        downloader(spec, staged)
        return promote_verified_archive(staged, paths.archive_path, spec)


def _obtain_extracted(spec: SourceSpec, paths: AcquisitionPaths) -> str:
    if paths.extracted_path.exists():
        verify_regular_size(paths.extracted_path, spec.extracted_size_bytes)
        return file_sha256(paths.extracted_path)
    with tempfile.TemporaryDirectory(prefix="extract-", dir=paths.staging_root) as raw:
        staged = Path(raw) / spec.extracted_filename
        digest = extract_single_mcap(
            paths.archive_path,
            staged,
            spec.extracted_size_bytes,
        )
        promote_staged_file(staged, paths.extracted_path)
        return digest


def _failure_result(
    status: Literal["deferred", "conflict"],
    source_id: str,
    error: AcquisitionDeferred | AcquisitionConflict,
) -> AcquisitionResult:
    return AcquisitionResult(
        status=status,
        source_id=source_id,
        reason_code=error.reason,
        detail=error.detail or None,
        artifact_absent=True,
        archive_path=None,
        archive_size_bytes=None,
        archive_sha256=None,
        extracted_path=None,
        extracted_size_bytes=None,
        extracted_sha256=None,
    )


def main(args: list[str] | None = None) -> None:
    """ROS parameter를 읽은 뒤 graph를 닫고 bounded acquisition을 실행한다."""
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from rclpy.node import Node

    source_default = (
        Path(get_package_share_directory("go2_validation"))
        / "config"
        / "external_replay_sources.yaml"
    )
    rclpy.init(args=args)
    node = Node("go2_external_replay_acquisition")
    manifest = Path(str(node.declare_parameter("source_manifest", str(source_default)).value))
    cache_root = Path(
        str(
            node.declare_parameter(
                "cache_root",
                "data/external/dimos_go2_indoor",
            ).value
        )
    )
    output_path = Path(
        str(
            node.declare_parameter(
                "output_path",
                str(cache_root / "runs/acquisition.json"),
            ).value
        )
    )
    node.destroy_node()
    rclpy.shutdown()
    try:
        spec = load_source_spec(manifest)
        paths = AcquisitionPaths.from_cache_root(cache_root, spec)
        result = acquire_external_replay(spec, paths, download_archive_with_curl)
    except AcquisitionConflict as error:
        result = _failure_result("conflict", "unparsed", error)
    write_acquisition_result(output_path, result)
    raise SystemExit(2 if result.status == "conflict" else 0)
