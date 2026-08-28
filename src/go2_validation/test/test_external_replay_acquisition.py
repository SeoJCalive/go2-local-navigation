from __future__ import annotations

import hashlib
import io
import shutil
import subprocess
import tarfile
from pathlib import Path

import pytest
import yaml

from go2_validation.external_replay_acquisition import (
    AcquisitionConflict,
    AcquisitionDeferred,
    SourceSpec,
    classify_http_status,
    ensure_initial_space,
    extract_single_mcap,
    stream_bounded,
    verify_file,
)
from go2_validation.external_replay_acquisition_runner import (
    AcquisitionPaths,
    acquire_external_replay,
)
from go2_validation.external_replay_manifest import load_source_spec
from go2_validation.external_replay_download import download_archive_with_curl
from go2_validation.external_replay_manifest import load_conversion_spec


PROJECT_ROOT = Path(__file__).parents[3]
KNOWLEDGE_ROOT = Path("/home/tjwocjf0915/research/Go2")
CONFIG_PATH = PROJECT_ROOT / "src/go2_validation/config/external_replay_sources.yaml"
SOURCE_CARD = (
    KNOWLEDGE_ROOT
    / "sources/repositories/related/dimos_go2_replay_source_card.md"
)


def _tar(path: Path, members: tuple[tuple[tarfile.TarInfo, bytes], ...]) -> None:
    with tarfile.open(path, "w:gz") as archive:
        for info, payload in members:
            info.size = len(payload)
            archive.addfile(info, io.BytesIO(payload))


def _regular(name: str, payload: bytes = b"mcap") -> tuple[tarfile.TarInfo, bytes]:
    return tarfile.TarInfo(name), payload


def test_source_card_matches_machine_manifest() -> None:
    # Given: the tracked source manifest and human-readable custody card.
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))["sources"][0]
    if not SOURCE_CARD.is_file():
        pytest.skip("knowledge repository is validated outside the project checkout")
    card = SOURCE_CARD.read_text(encoding="utf-8")

    # When: pinned identity fields are rendered as strings.
    pinned = (
        config["commit"],
        config["lfs_oid"],
        config["archive_sha256"],
        str(config["archive_size_bytes"]),
        str(config["extracted_size_bytes"]),
    )

    # Then: every machine-consumed identity appears in the source card.
    assert all(value in card for value in pinned)
    assert config["dataset_license_status"] == "dataset_license_unverified"
    assert config["redistribution"] is False


@pytest.mark.parametrize(
    ("members", "reason"),
    [
        ((_regular("../escape.mcap"),), "unsafe_member_path"),
        ((_regular("/absolute.mcap"),), "unsafe_member_path"),
        ((_regular("first.mcap"), _regular("second.mcap")), "member_count_mismatch"),
        ((_regular("payload.txt"),), "member_must_be_mcap"),
    ],
)
def test_extract_rejects_unsafe_or_ambiguous_members(
    tmp_path: Path,
    members: tuple[tuple[tarfile.TarInfo, bytes], ...],
    reason: str,
) -> None:
    # Given: an attacker-controlled archive shape.
    archive = tmp_path / "source.tar.gz"
    _tar(archive, members)

    # When: extraction validates before promotion.
    with pytest.raises(AcquisitionConflict) as raised:
        extract_single_mcap(archive, tmp_path / "out.mcap", 4)

    # Then: the exact conflict is reported and no output survives.
    assert raised.value.reason == reason
    assert not (tmp_path / "out.mcap").exists()


@pytest.mark.parametrize("link_type", [tarfile.SYMTYPE, tarfile.LNKTYPE])
def test_extract_rejects_links(tmp_path: Path, link_type: bytes) -> None:
    # Given: a symbolic or hard link disguised with an MCAP name.
    info = tarfile.TarInfo("recording.mcap")
    info.type = link_type
    info.linkname = "target"
    archive = tmp_path / "source.tar.gz"
    _tar(archive, ((info, b""),))

    # When: extraction inspects member type.
    with pytest.raises(AcquisitionConflict) as raised:
        extract_single_mcap(archive, tmp_path / "out.mcap", 0)

    # Then: links are never followed or promoted.
    assert raised.value.reason == "member_must_be_regular_file"
    assert not (tmp_path / "out.mcap").exists()


@pytest.mark.parametrize("special_type", [tarfile.FIFOTYPE, tarfile.CHRTYPE])
def test_extract_rejects_fifo_and_device(tmp_path: Path, special_type: bytes) -> None:
    # Given: a special member disguised with an MCAP name.
    info = tarfile.TarInfo("recording.mcap")
    info.type = special_type
    archive = tmp_path / "source.tar.gz"
    _tar(archive, ((info, b""),))

    # When: extraction checks the member before opening it.
    with pytest.raises(AcquisitionConflict) as raised:
        extract_single_mcap(archive, tmp_path / "out.mcap", 0)

    # Then: devices and FIFOs are denied without promotion.
    assert raised.value.reason == "member_must_be_regular_file"
    assert not (tmp_path / "out.mcap").exists()


def test_extract_rejects_size_mismatch(tmp_path: Path) -> None:
    # Given: one safe regular member with an unexpected size.
    archive = tmp_path / "source.tar.gz"
    _tar(archive, (_regular("recording.mcap"),))

    # When: the extracted byte count differs from custody metadata.
    with pytest.raises(AcquisitionConflict) as raised:
        extract_single_mcap(archive, tmp_path / "out.mcap", 5)

    # Then: promotion is denied.
    assert raised.value.reason == "extracted_size_mismatch"
    assert not (tmp_path / "out.mcap").exists()


def test_stream_bounded_rejects_cap_and_removes_partial(tmp_path: Path) -> None:
    # Given: a stream one byte larger than its hard cap.
    destination = tmp_path / "partial.archive"

    # When: bounded streaming reaches the extra byte.
    with pytest.raises(AcquisitionConflict) as raised:
        stream_bounded(io.BytesIO(b"12345"), destination, 4)

    # Then: no partial download remains.
    assert raised.value.reason == "download_byte_cap_exceeded"
    assert not destination.exists()


def test_extract_promotes_exact_single_member(tmp_path: Path) -> None:
    # Given: one regular MCAP with the expected size.
    archive = tmp_path / "source.tar.gz"
    _tar(archive, (_regular("nested/recording.mcap"),))
    destination = tmp_path / "source" / "recording.mcap"

    # When: secure extraction completes.
    digest = extract_single_mcap(archive, destination, 4)

    # Then: only the atomically promoted payload exists.
    assert destination.read_bytes() == b"mcap"
    assert digest == hashlib.sha256(b"mcap").hexdigest()
    assert not list(tmp_path.glob("**/*.partial"))


def test_source_spec_rejects_invalid_sha256() -> None:
    # Given: a malformed trust-boundary identity.
    # When: it is parsed into the acquisition contract.
    with pytest.raises(AcquisitionConflict) as raised:
        SourceSpec(
            source_id="fixture",
            download_url="https://example.invalid/object",
            archive_filename="fixture.mcap.tar.gz",
            extracted_filename="fixture.mcap",
            archive_sha256="bad",
            archive_size_bytes=4,
            extracted_size_bytes=4,
            minimum_free_bytes=16,
            connect_timeout_seconds=30,
            total_timeout_seconds=3600,
            max_attempts=3,
        )

    # Then: malformed provenance cannot cross the boundary.
    assert raised.value.reason == "invalid_archive_sha256"


def test_source_spec_rejects_filename_path_component() -> None:
    with pytest.raises(AcquisitionConflict) as raised:
        SourceSpec(
            source_id="fixture",
            download_url="https://example.invalid/object",
            archive_filename="../fixture.mcap.tar.gz",
            extracted_filename="fixture.mcap",
            archive_sha256="0" * 64,
            archive_size_bytes=4,
            extracted_size_bytes=4,
            minimum_free_bytes=16,
            connect_timeout_seconds=30,
            total_timeout_seconds=3600,
            max_attempts=3,
        )

    assert raised.value.reason == "source_filename_invalid"


def test_manifest_loads_pinned_source_spec() -> None:
    spec = load_source_spec(CONFIG_PATH)

    assert spec.source_id == "dimos_go2_indoor"
    assert spec.archive_filename == "go2_china_office_indoor.mcap.tar.gz"
    assert spec.extracted_filename == "go2_china_office_indoor.mcap"


def test_acquisition_promotes_verified_archive_and_mcap(tmp_path: Path) -> None:
    fixture_archive = tmp_path / "fixture.tar.gz"
    _tar(fixture_archive, (_regular("nested/recording.mcap"),))
    payload = fixture_archive.read_bytes()
    spec = SourceSpec(
        source_id="fixture",
        download_url="https://example.invalid/object",
        archive_filename="fixture.mcap.tar.gz",
        extracted_filename="fixture.mcap",
        archive_sha256=hashlib.sha256(payload).hexdigest(),
        archive_size_bytes=len(payload),
        extracted_size_bytes=4,
        minimum_free_bytes=1,
        connect_timeout_seconds=1,
        total_timeout_seconds=1,
        max_attempts=1,
    )
    paths = AcquisitionPaths.from_cache_root(tmp_path / "cache", spec)

    def download(_spec: SourceSpec, destination: Path) -> str:
        destination.write_bytes(payload)
        return hashlib.sha256(payload).hexdigest()

    result = acquire_external_replay(spec, paths, download)

    assert result.status == "passed"
    assert result.archive_sha256 == spec.archive_sha256
    assert result.extracted_sha256 == hashlib.sha256(b"mcap").hexdigest()
    assert paths.archive_path.read_bytes() == payload
    assert paths.extracted_path.read_bytes() == b"mcap"
    assert not tuple(paths.staging_root.iterdir())


def test_download_hash_conflict_removes_partial(
    tmp_path: Path,
    monkeypatch,
) -> None:
    spec = SourceSpec(
        source_id="fixture",
        download_url="https://example.invalid/object",
        archive_filename="fixture.mcap.tar.gz",
        extracted_filename="fixture.mcap",
        archive_sha256="0" * 64,
        archive_size_bytes=4,
        extracted_size_bytes=4,
        minimum_free_bytes=1,
        connect_timeout_seconds=1,
        total_timeout_seconds=1,
        max_attempts=1,
    )
    destination = tmp_path / spec.archive_filename

    def run(*_args, **_kwargs):
        destination.write_bytes(b"bad!")
        return subprocess.CompletedProcess([], 0, stdout="200", stderr="")

    monkeypatch.setattr("go2_validation.external_replay_download.subprocess.run", run)

    with pytest.raises(AcquisitionConflict) as raised:
        download_archive_with_curl(spec, destination)

    assert raised.value.reason == "source_hash_mismatch"
    assert not destination.exists()


def test_conversion_manifest_rejects_duplicate_selected_channel(
    tmp_path: Path,
) -> None:
    document = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    document["sources"][0]["selected_channels"].append(
        dict(document["sources"][0]["selected_channels"][0])
    )
    path = tmp_path / "duplicate.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")

    with pytest.raises(AcquisitionConflict) as raised:
        load_conversion_spec(path)

    assert raised.value.reason == "duplicate_selected_channel"


def test_verify_file_rejects_wrong_hash(tmp_path: Path) -> None:
    # Given: a present archive with the expected size but wrong bytes.
    archive = tmp_path / "archive"
    archive.write_bytes(b"bad!")

    # When: pinned integrity is checked.
    with pytest.raises(AcquisitionConflict) as raised:
        verify_file(archive, 4, "0" * 64)

    # Then: source promotion is blocked as conflict.
    assert raised.value.reason == "source_hash_mismatch"


def test_initial_space_absence_is_deferred(tmp_path: Path, monkeypatch) -> None:
    # Given: the staging filesystem lacks the pinned initial free space.
    monkeypatch.setattr(shutil, "disk_usage", lambda _path: shutil._ntuple_diskusage(1, 1, 0))

    # When: acquisition checks space before network access.
    with pytest.raises(AcquisitionDeferred) as raised:
        ensure_initial_space(tmp_path, 1)

    # Then: only this prerequisite is deferred.
    assert raised.value.reason == "initial_space_prerequisite_absent"


@pytest.mark.parametrize("status_code", [403, 404, 500, 503])
def test_http_prerequisite_status_is_deferred(status_code: int) -> None:
    # Given: a bounded attempt ended in a source availability status.
    # When: the response status is classified.
    with pytest.raises(AcquisitionDeferred) as raised:
        classify_http_status(status_code)

    # Then: only the external lane is deferred.
    assert raised.value.reason == "http_prerequisite_absent"


def test_http_integrity_status_is_conflict() -> None:
    # Given: a response incompatible with the pinned acquisition contract.
    # When: the response status is classified.
    with pytest.raises(AcquisitionConflict) as raised:
        classify_http_status(206)

    # Then: partial or unexpected HTTP semantics are not deferred.
    assert raised.value.reason == "unexpected_http_status"
