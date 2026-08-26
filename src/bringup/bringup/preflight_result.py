"""observer JSON을 검증하고 최종 preflight result로 다시 저장한다."""

from dataclasses import dataclass
import json
import os
from pathlib import Path
from typing import TypeAlias

from bringup.preflight_types import CheckStatus


JsonValue: TypeAlias = (
    str
    | int
    | float
    | bool
    | None
    | list["JsonValue"]
    | dict[str, "JsonValue"]
)
JsonDocument: TypeAlias = dict[str, JsonValue]


@dataclass(frozen=True, slots=True)
class ReportFormatError(Exception):
    """observer JSON이 최종 report 계약을 충족하지 못했음을 나타낸다."""

    detail: str

    def __str__(self) -> str:
        return self.detail


def load_document(path: Path) -> JsonDocument:
    """JSON root object를 읽고 다른 root type을 거부한다."""
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ReportFormatError("observer report root must be an object")
    return document


def write_document(document: JsonDocument, path: Path) -> None:
    """최종 JSON을 같은 directory의 임시 파일을 거쳐 원자적으로 저장한다."""
    temporary_path = path.with_suffix(f"{path.suffix}.tmp")
    temporary_path.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary_path, path)


def read_observer_status(document: JsonDocument) -> CheckStatus:
    """Observer overall_status를 폐쇄된 CheckStatus로 파싱한다."""
    raw_status = document.get("overall_status")
    if not isinstance(raw_status, str):
        raise ReportFormatError("observer overall_status must be a string")
    try:
        return CheckStatus(raw_status)
    except ValueError as error:
        raise ReportFormatError(
            f"unsupported observer overall_status: {raw_status}"
        ) from error
