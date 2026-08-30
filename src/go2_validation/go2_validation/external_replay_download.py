
"""Run the target's bounded curl client for one pinned external artifact."""
from dataclasses import dataclass
from pathlib import Path
import subprocess
from typing import Final

from go2_validation.external_replay_acquisition import (
    classify_http_status,
    verify_file,
)
from go2_validation.external_replay_contract import (
    AcquisitionConflict,
    AcquisitionDeferred,
    SourceSpec,
)


NETWORK_EXIT_CODES: Final = frozenset({5, 6, 7, 28, 35, 47, 52, 55, 56})
MAX_FILESIZE_EXIT_CODE: Final = 63
LOCAL_WRITE_EXIT_CODE: Final = 23


@dataclass(frozen=True, slots=True)
class CurlAttempt:
    """One curl process result reduced to acquisition-relevant fields."""

    return_code: int
    http_status: int
    stderr: str


def download_archive_with_curl(spec: SourceSpec, destination: Path) -> str:
    """Download the pinned archive with explicit attempts, timeouts, and cap."""
    last_network_detail = "network_prerequisite_absent"
    destination.parent.mkdir(parents=True, exist_ok=True)
    for _attempt in range(spec.max_attempts):
        destination.unlink(missing_ok=True)
        result = _curl_attempt(spec, destination)
        if result.return_code == 0:
            try:
                classify_http_status(result.http_status)
                return verify_file(
                    destination,
                    spec.archive_size_bytes,
                    spec.archive_sha256,
                )
            except AcquisitionDeferred as error:
                destination.unlink(missing_ok=True)
                last_network_detail = str(error)
                continue
            except AcquisitionConflict:
                destination.unlink(missing_ok=True)
                raise
        if result.return_code in NETWORK_EXIT_CODES:
            last_network_detail = (
                f"curl_exit={result.return_code}; stderr={result.stderr[-500:]}"
            )
            continue
        destination.unlink(missing_ok=True)
        if result.return_code == MAX_FILESIZE_EXIT_CODE:
            raise AcquisitionConflict("download_byte_cap_exceeded")
        if result.return_code == LOCAL_WRITE_EXIT_CODE:
            raise AcquisitionConflict("local_write_failure", result.stderr[-500:])
        raise AcquisitionConflict(
            "curl_download_failure",
            f"exit={result.return_code}; stderr={result.stderr[-500:]}",
        )
    destination.unlink(missing_ok=True)
    raise AcquisitionDeferred("network_prerequisite_absent", last_network_detail)


def _curl_attempt(spec: SourceSpec, destination: Path) -> CurlAttempt:
    command = [
        "/usr/bin/curl",
        "--silent",
        "--show-error",
        "--location",
        "--proto",
        "=https",
        "--proto-redir",
        "=https",
        "--connect-timeout",
        str(spec.connect_timeout_seconds),
        "--max-time",
        str(spec.total_timeout_seconds),
        "--max-filesize",
        str(spec.archive_size_bytes),
        "--output",
        str(destination),
        "--write-out",
        "%{http_code}",
        spec.download_url,
    ]
    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=False,
            timeout=spec.total_timeout_seconds + 5,
        )
    except subprocess.TimeoutExpired as error:
        return CurlAttempt(28, 0, str(error))
    raw_status = completed.stdout.strip()
    http_status = int(raw_status) if raw_status.isdigit() else 0
    return CurlAttempt(completed.returncode, http_status, completed.stderr.strip())
