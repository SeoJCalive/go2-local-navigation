from pathlib import Path
from typing import Final

import yaml

PROJECT_ROOT: Final = Path(__file__).resolve().parents[3]


def test_given_verification_documents_when_loaded_then_stage_order_is_canonical() -> None:
    manifest = yaml.safe_load(
        (PROJECT_ROOT / "verification/structured/project_manifest.yaml").read_text()
    )
    matrix = yaml.safe_load(
        (PROJECT_ROOT / "verification/structured/acceptance_matrix.yaml").read_text()
    )
    final_mount = yaml.safe_load(
        (
            PROJECT_ROOT
            / "verification/structured/final_mount_acceptance.yaml"
        ).read_text()
    )
    physical_motion = yaml.safe_load(
        (
            PROJECT_ROOT
            / "verification/structured/limited_physical_motion_acceptance.yaml"
        ).read_text()
    )

    assert manifest["software_navigation_stages"] == [
        {"id": 11, "name": "software_fault_recovery"},
        {"id": 12, "name": "mapping_localization_and_nav2_shadow"},
        {"id": 13, "name": "software_only_freeze"},
        {"id": 14, "name": "final_mount_integration"},
        {"id": 15, "name": "limited_physical_motion_validation"},
    ]
    assert "software_fault_recovery" in manifest["completed_stages"]
    assert "software_only_freeze" in manifest["completed_stages"]
    assert manifest["next_stage"] == "final_mount_integration"
    assert matrix["next_stage"]["stage_id"] == 14
    assert matrix["next_stage"]["name"] == "final_mount_integration"
    assert final_mount["stage_id"] == 14
    assert physical_motion["stage_id"] == 15


def test_given_replay_evidence_when_indexed_then_source_kind_and_level_are_explicit() -> None:
    matrix = yaml.safe_load(
        (PROJECT_ROOT / "verification/structured/acceptance_matrix.yaml").read_text()
    )

    vocabulary = matrix["vocabulary"]
    assert "synthetic-verified" in vocabulary["evidence_levels"]
    assert vocabulary["replay_source_kinds"] == [
        "project_stationary",
        "external_dynamic",
    ]


def test_given_structured_documents_when_sources_resolve_then_ids_remain_unique() -> None:
    structured = PROJECT_ROOT / "verification/structured"
    documents = tuple(
        yaml.safe_load(path.read_text()) for path in sorted(structured.glob("*.yaml"))
    )
    record_ids = tuple(document["record_id"] for document in documents)

    assert len(record_ids) == len(set(record_ids))
    assert "go2-local-navigation-final-mount-acceptance-20260827" in record_ids
    assert "go2-local-navigation-limited-physical-motion-acceptance-20260827" in record_ids
    for document in documents:
        source_markdown = document.get("source_markdown")
        if source_markdown is not None:
            assert (structured / source_markdown).resolve().is_file()
