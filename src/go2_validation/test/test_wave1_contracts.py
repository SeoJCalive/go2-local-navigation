"""Wave 1 stage and execution-mode contracts."""

from pathlib import Path

import yaml

from go2_validation.runtime_preflight import (
    RuntimeObservation,
    assess_mode,
    assess_runtime_observation,
)


PROJECT_ROOT = Path(__file__).resolve().parents[3]


def test_given_stage_ssot_when_loaded_then_preserves_software_before_physical_stages() -> None:
    """Stage order must not regress physical preparation into software work."""
    # Given: the project verification source of truth
    document = yaml.safe_load(
        (PROJECT_ROOT / "verification/structured/project_manifest.yaml").read_text()
    )

    # When: its navigation stages are inspected
    stages = document["software_navigation_stages"]

    # Then: every Wave 1 stage has its canonical order and provenance contract
    assert [(stage["id"], stage["name"]) for stage in stages] == [
        (11, "software_fault_recovery"),
        (12, "mapping_localization_and_nav2_shadow"),
        (13, "software_only_freeze"),
        (14, "final_mount_integration"),
        (15, "limited_physical_motion_validation"),
    ]
    assert document["replay_provenance"]["required_source_kinds"] == [
        "project_stationary",
        "external_dynamic",
    ]


def test_given_stage_documents_when_inspected_then_physical_preparation_starts_at_14() -> None:
    """The documentation must not retain the superseded 11/12 physical stages."""
    # Given: the physical-stage documentation and sidecars
    files = (
        PROJECT_ROOT / "verification/README.md",
        PROJECT_ROOT / "verification/final_mount_integration.md",
        PROJECT_ROOT / "verification/limited_physical_motion_validation.md",
        PROJECT_ROOT / "verification/structured/final_mount_acceptance.yaml",
        PROJECT_ROOT / "verification/structured/limited_physical_motion_acceptance.yaml",
    )

    # When: their stage references are read
    content = "\n".join(path.read_text() for path in files)

    # Then: physical preparation has no 11/12 stage identity
    assert "stage_11_final_mount" not in content
    assert "stage_12_limited_physical" not in content


def test_given_offline_mode_when_domain_or_clock_owner_is_wrong_then_preflight_rejects() -> None:
    """Offline fault mode must remain isolated from the live domain and clock."""
    # Given: a bad offline profile
    # When: the preflight evaluates it
    result = assess_mode("offline_fault", domain_id=0, clock_owner="fixture")

    # Then: it identifies the exact boundary violation
    assert result.exit_code == 2
    assert result.reason_code == "domain_mismatch"


def test_given_synthetic_mode_when_profile_matches_then_preflight_accepts() -> None:
    """Synthetic navigation has one fixture clock and TF owner."""
    # Given: the canonical synthetic profile
    # When: the preflight evaluates it
    result = assess_mode("synthetic_navigation", domain_id=65, clock_owner="fixture")

    # Then: it is accepted
    assert result.exit_code == 0
    assert result.reason_code is None


def test_given_fault_mode_when_clock_and_isolation_match_then_runtime_is_accepted() -> None:
    observation = RuntimeObservation(
        mode="offline_fault",
        domain_id=61,
        declared_clock_owner="fixture",
        clock_publisher_count=0,
        use_sim_time=True,
        loopback_only=True,
    )

    result = assess_runtime_observation(observation)

    assert result.exit_code == 0
    assert result.reason_code is None


def test_given_duplicate_clock_or_wall_time_when_assessed_then_preflight_rejects() -> None:
    baseline = RuntimeObservation(
        mode="scan_replay",
        domain_id=62,
        declared_clock_owner="rosbag_player",
        clock_publisher_count=2,
        use_sim_time=True,
        loopback_only=True,
    )

    duplicate = assess_runtime_observation(baseline)
    wall_time = assess_runtime_observation(
        RuntimeObservation(
            mode=baseline.mode,
            domain_id=baseline.domain_id,
            declared_clock_owner=baseline.declared_clock_owner,
            clock_publisher_count=0,
            use_sim_time=False,
            loopback_only=True,
        )
    )

    assert duplicate.reason_code == "duplicate_clock_publishers"
    assert wall_time.reason_code == "sim_time_mismatch"
