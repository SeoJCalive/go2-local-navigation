
"""Bounded local-only acquisition for the pinned external replay source."""
from __future__ import annotations

import hashlib
import os
import shutil
import tarfile
from pathlib import Path, PurePosixPath
from typing import BinaryIO, Final

from .external_replay_contract import (
    AcquisitionConflict,
    AcquisitionDeferred,
    SourceSpec,
)


CHUNK_BYTES: Final = 1024 * 1024


def stream_bounded(source: BinaryIO, destination: Path, byte_cap: int) -> str:
    """Copy at most byte_cap bytes and remove every partial file on failure."""
    digest = hashlib.sha256()
    written = 0
    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        with destination.open("xb") as output:
            while chunk := source.read(min(CHUNK_BYTES, byte_cap - written + 1)):
                written += len(chunk)
                if written > byte_cap:
                    raise AcquisitionConflict("download_byte_cap_exceeded")
                output.write(chunk)
                digest.update(chunk)
            output.flush()
            os.fsync(output.fileno())
    except AcquisitionConflict:
        destination.unlink(missing_ok=True)
        raise
    except OSError as error:
        destination.unlink(missing_ok=True)
        raise AcquisitionConflict("local_write_failure", str(error)) from error
    return digest.hexdigest()


def extract_single_mcap(
    archive_path: Path,
    destination: Path,
    expected_size_bytes: int,
) -> str:
    """Validate one regular relative MCAP member before atomic promotion."""
    partial = destination.with_name(f".{destination.name}.partial")
    partial.unlink(missing_ok=True)
    try:
        with tarfile.open(archive_path, "r:gz") as archive:
            members = archive.getmembers()
            if len(members) != 1:
                raise AcquisitionConflict("member_count_mismatch", str(len(members)))
            member = members[0]
            _validate_member(member)
            if member.size != expected_size_bytes:
                detail = f"{member.size}!={expected_size_bytes}"
                raise AcquisitionConflict("extracted_size_mismatch", detail)
            source = archive.extractfile(member)
            if source is None:
                raise AcquisitionConflict("member_stream_unavailable")
            digest = stream_bounded(source, partial, expected_size_bytes)
        destination.parent.mkdir(parents=True, exist_ok=True)
        os.replace(partial, destination)
        _fsync_directory(destination.parent)
        return digest
    except AcquisitionConflict:
        partial.unlink(missing_ok=True)
        raise
    except (OSError, tarfile.TarError, EOFError) as error:
        partial.unlink(missing_ok=True)
        raise AcquisitionConflict("archive_read_or_write_failure", str(error)) from error


def verify_file(path: Path, expected_size: int, expected_sha256: str) -> str:
    """Require exact size and digest for a local custody artifact."""
    verify_regular_size(path, expected_size)
    actual = file_sha256(path)
    if actual != expected_sha256:
        raise AcquisitionConflict("source_hash_mismatch", actual)
    return actual


def verify_regular_size(path: Path, expected_size: int) -> int:
    """Reject links, non-files, and size drift at the local trust boundary."""
    if not path.is_file() or path.is_symlink():
        raise AcquisitionConflict("source_must_be_regular_file", str(path))
    actual_size = path.stat().st_size
    if actual_size != expected_size:
        raise AcquisitionConflict("source_size_mismatch", str(actual_size))
    return actual_size


def ensure_initial_space(staging_root: Path, minimum_free_bytes: int) -> None:
    staging_root.mkdir(parents=True, exist_ok=True)
    available = shutil.disk_usage(staging_root).free
    if available < minimum_free_bytes:
        raise AcquisitionDeferred("initial_space_prerequisite_absent", str(available))


def classify_http_status(status_code: int) -> None:
    if status_code == 200:
        return
    if status_code in {403, 404} or status_code >= 500:
        raise AcquisitionDeferred("http_prerequisite_absent", str(status_code))
    raise AcquisitionConflict("unexpected_http_status", str(status_code))


def promote_verified_archive(staged: Path, destination: Path, spec: SourceSpec) -> str:
    digest = verify_file(staged, spec.archive_size_bytes, spec.archive_sha256)
    destination.parent.mkdir(parents=True, exist_ok=True)
    os.replace(staged, destination)
    _fsync_directory(destination.parent)
    return digest


def promote_staged_file(staged: Path, destination: Path) -> None:
    """Atomically promote one already-validated regular file and fsync its parent."""
    if not staged.is_file() or staged.is_symlink():
        raise AcquisitionConflict("staged_source_must_be_regular_file", str(staged))
    destination.parent.mkdir(parents=True, exist_ok=True)
    os.replace(staged, destination)
    _fsync_directory(destination.parent)


def _validate_member(member: tarfile.TarInfo) -> None:
    path = PurePosixPath(member.name)
    if path.is_absolute() or ".." in path.parts or not member.name:
        raise AcquisitionConflict("unsafe_member_path", member.name)
    if not member.isreg():
        raise AcquisitionConflict("member_must_be_regular_file", member.name)
    if path.suffix.lower() != ".mcap":
        raise AcquisitionConflict("member_must_be_mcap", member.name)


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def file_sha256(path: Path) -> str:
    """Hash one regular local artifact without changing it."""
    if not path.is_file() or path.is_symlink():
        raise AcquisitionConflict("source_must_be_regular_file", str(path))
    digest = hashlib.sha256()
    with path.open("rb") as source:
        while chunk := source.read(CHUNK_BYTES):
            digest.update(chunk)
    return digest.hexdigest()
