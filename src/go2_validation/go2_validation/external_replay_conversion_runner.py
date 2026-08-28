
"""External replay acquisition 결과를 읽어 canonical 변환과 JSON 기록을 실행한다."""
from dataclasses import asdict
from datetime import datetime
import json
import os
from pathlib import Path

from go2_validation.external_replay_acquisition_runner import AcquisitionResult
from go2_validation.external_replay_contract import AcquisitionConflict
from go2_validation.external_replay_conversion import convert_external_replay
from go2_validation.external_replay_conversion_result import (
    ConversionFailure,
    ConversionResult,
    failed_conversion,
)
from go2_validation.external_replay_manifest import load_conversion_spec


def write_conversion_result(path: Path, result: ConversionResult) -> None:
    """Canonical fixture provenance를 atomic JSON으로 기록한다."""
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_name(f".{path.name}.partial")
    partial.unlink(missing_ok=True)
    document = {
        "schema_version": 1,
        "record_kind": "external_replay_conversion_result",
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
        raise AcquisitionConflict("conversion_result_write_failure", str(error)) from error


def _read_acquisition_result(path: Path) -> AcquisitionResult:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except OSError as error:
        return _absent_acquisition("acquisition_result_absent", str(error))
    except json.JSONDecodeError as error:
        return _conflict_acquisition("acquisition_result_json_invalid", str(error))
    if not isinstance(document, dict):
        return _conflict_acquisition("acquisition_result_root_invalid", None)
    status = document.get("status")
    if status not in {"passed", "deferred", "conflict"}:
        return _conflict_acquisition("acquisition_result_status_invalid", str(status))
    try:
        return AcquisitionResult(
            status=status,
            source_id=_required_string(document.get("source_id")),
            reason_code=_optional_string(document.get("reason_code")),
            detail=_optional_string(document.get("detail")),
            artifact_absent=_required_bool(document.get("artifact_absent")),
            archive_path=_optional_string(document.get("archive_path")),
            archive_size_bytes=_optional_int(document.get("archive_size_bytes")),
            archive_sha256=_optional_string(document.get("archive_sha256")),
            extracted_path=_optional_string(document.get("extracted_path")),
            extracted_size_bytes=_optional_int(document.get("extracted_size_bytes")),
            extracted_sha256=_optional_string(document.get("extracted_sha256")),
        )
    except (KeyError, TypeError, ValueError) as error:
        return _conflict_acquisition("acquisition_result_field_invalid", str(error))


def _optional_string(value) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str):
        raise TypeError("expected optional string")
    return value


def _required_string(value) -> str:
    if not isinstance(value, str) or not value:
        raise TypeError("expected non-empty string")
    return value


def _required_bool(value) -> bool:
    if not isinstance(value, bool):
        raise TypeError("expected boolean")
    return value


def _optional_int(value) -> int | None:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError("expected optional integer")
    return value


def _absent_acquisition(reason: str, detail: str | None) -> AcquisitionResult:
    return AcquisitionResult(
        "deferred", "dimos_go2_indoor", reason, detail, True,
        None, None, None, None, None, None,
    )


def _conflict_acquisition(reason: str, detail: str | None) -> AcquisitionResult:
    return AcquisitionResult(
        "conflict", "dimos_go2_indoor", reason, detail, True,
        None, None, None, None, None, None,
    )


def main(args: list[str] | None = None) -> None:
    """ROS parameter를 읽고 graph를 닫은 뒤 local-only 변환을 실행한다."""
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from rclpy.node import Node

    manifest_default = (
        Path(get_package_share_directory("go2_validation"))
        / "config"
        / "external_replay_sources.yaml"
    )
    rclpy.init(args=args)
    node = Node("go2_external_replay_conversion")
    manifest = Path(str(node.declare_parameter("source_manifest", str(manifest_default)).value))
    cache_root = Path(str(node.declare_parameter("cache_root", "data/external/dimos_go2_indoor").value))
    output_root = Path(str(node.declare_parameter("output_root", str(cache_root / "derived")).value))
    result_path = Path(str(node.declare_parameter("result_path", str(cache_root / "runs/conversion.json")).value))
    node.destroy_node()
    rclpy.shutdown()
    acquisition = _read_acquisition_result(cache_root / "runs/acquisition.json")
    try:
        result = convert_external_replay(
            load_conversion_spec(manifest),
            acquisition,
            output_root,
        )
    except AcquisitionConflict as error:
        result = _conflict_conversion(acquisition, error)
    write_conversion_result(result_path, result)
    raise SystemExit(2 if result.status == "conflict" else 0)


def _conflict_conversion(
    acquisition: AcquisitionResult,
    error: AcquisitionConflict,
) -> ConversionResult:
    return failed_conversion(
        acquisition,
        ConversionFailure("conflict", error.reason, error.detail or None),
    )
