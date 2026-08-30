"""Mapping cloud gate contracts."""

from go2_perception.mapping_cloud_contract import MappingCloudSample, assess_mapping_cloud


def test_given_valid_cloud_when_assessed_then_gate_forwards_header_once() -> None:
    """A valid mapping cloud is passed through without changing its header."""
    # Given: a valid finite cloud
    sample = MappingCloudSample(
        stamp_nanoseconds=10,
        width=2,
        height=1,
        point_step=16,
        row_step=32,
        data_length=32,
        finite_point_count=2,
    )

    # When: the gate assesses it
    result = assess_mapping_cloud(sample, now_nanoseconds=11)

    # Then: it permits one unchanged publication
    assert result.publish
    assert result.reason_code is None


def test_given_malformed_empty_or_stale_cloud_when_assessed_then_gate_suppresses() -> None:
    """Invalid clouds cannot reach the mapping input topic."""
    # Given: malformed, empty, and stale samples
    samples = (
        MappingCloudSample(10, 1, 1, 0, 0, 0, 1),
        MappingCloudSample(10, 0, 1, 16, 0, 0, 0),
        MappingCloudSample(1, 1, 1, 16, 16, 16, 1),
    )

    # When: the gate assesses each sample
    results = tuple(assess_mapping_cloud(sample, now_nanoseconds=2_000_000_000) for sample in samples)

    # Then: every sample is suppressed with a reason
    assert all(not result.publish for result in results)
    assert {result.reason_code for result in results} == {"malformed_layout", "empty_cloud", "stale_cloud"}


def test_given_padded_or_nonpositive_cloud_when_assessed_then_layout_is_exact() -> None:
    padded = MappingCloudSample(10, 2, 1, 16, 48, 32, 2)
    nonpositive = MappingCloudSample(0, 1, 1, 16, 16, 16, 1)

    padded_result = assess_mapping_cloud(padded, now_nanoseconds=11)
    stamp_result = assess_mapping_cloud(nonpositive, now_nanoseconds=11)

    assert not padded_result.publish
    assert padded_result.reason_code == "malformed_layout"
    assert not stamp_result.publish
    assert stamp_result.reason_code == "nonpositive_timestamp"
