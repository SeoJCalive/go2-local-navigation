"""Mapping cloud accumulator terminal accounting log parsing을 검증한다."""

import json

import pytest


def _accounting_json(**overrides: int) -> str:
    """Terminal marker가 보존해야 하는 정상 accounting JSON을 만든다."""
    accounting = {
        "received": 12,
        "future_waited": 2,
        "recovered_after_retry": 2,
        "processed": 12,
        "output_published": 12,
        "dropped_unrecoverable": 0,
        "dropped_overflow": 0,
        "pending_at_shutdown": 0,
        "partial_frames_not_emitted": 0,
        "emit_every": 1,
        "output_stamp_regression_count": 0,
    }
    accounting.update(overrides)
    return json.dumps(accounting, separators=(",", ":"))


def test_given_ros_launch_prefix_when_terminal_accounting_is_parsed_then_fields_are_immutable() -> None:
    # Given: ros2 launch가 node stdout 앞에 붙이는 정상 prefix와 terminal marker
    log_text = (
        "[go2_mapping_cloud_accumulator-5] [INFO] [0.000]: ready\n"
        "[go2_mapping_cloud_accumulator-5] MAPPING_CLOUD_ACCOUNTING "
        f"{_accounting_json()}\n"
    )

    # When: launch log boundary를 파싱한다.
    from go2_validation.mapping_cloud_accounting import parse_mapping_cloud_accounting_log

    accounting = parse_mapping_cloud_accounting_log(log_text)

    # Then: prefix와 무관하게 terminal JSON의 immutable accounting이 보존된다.
    assert accounting.received == 12
    assert accounting.future_waited == accounting.recovered_after_retry == 2
    assert accounting.processed == 12
    assert accounting.output_published == 12
    assert accounting.partial_frames_not_emitted == 0
    assert accounting.emit_every == 1


def test_given_terminal_accounting_when_parsed_then_only_lossless_fields_are_exposed() -> None:
    from dataclasses import asdict

    from go2_validation.mapping_cloud_accounting import parse_mapping_cloud_accounting_log

    # Given: complete no-shutdown-flush accounting marker
    log_text = f"[node-1] MAPPING_CLOUD_ACCOUNTING {_accounting_json()}\n"

    # When: launch-log boundary를 파싱한다.
    fields = asdict(parse_mapping_cloud_accounting_log(log_text))

    # Then: processed와 output publish는 분리되고 ambiguous published는 없다.
    assert tuple(fields) == (
        "received",
        "future_waited",
        "recovered_after_retry",
        "processed",
        "output_published",
        "dropped_unrecoverable",
        "dropped_overflow",
        "pending_at_shutdown",
        "partial_frames_not_emitted",
        "emit_every",
        "output_stamp_regression_count",
    )
    assert "published" not in fields


def test_given_accumulated_profile_ids_when_accounting_is_selected_then_both_require_marker(
    tmp_path,
) -> None:
    from types import MappingProxyType

    import go2_validation.mapping_cloud_accounting as mapping_cloud_accounting

    # Given: one valid marker for both accumulated profile IDs
    log_path = tmp_path / "launch.log"
    log_path.write_text(
        f"[node-1] MAPPING_CLOUD_ACCOUNTING {_accounting_json()}\n",
        encoding="utf-8",
    )

    # When/Then: raw skips accounting while both accumulated IDs parse the marker.
    assert isinstance(
        mapping_cloud_accounting.CLOUD_ACCOUNTING_EMIT_CADENCES,
        MappingProxyType,
    )
    assert mapping_cloud_accounting.CLOUD_ACCOUNTING_EMIT_CADENCES == {
        "dimos_odom_accumulated": 1,
        "dimos_odom_accumulated_emit3": 3,
        "dimos_odom_accumulated_emit10": 10,
    }
    import go2_validation.mapping_acceptance as mapping_acceptance

    assert (
        mapping_acceptance.CLOUD_ACCOUNTING_EMIT_CADENCES
        is mapping_cloud_accounting.CLOUD_ACCOUNTING_EMIT_CADENCES
    )
    assert mapping_cloud_accounting.mapping_cloud_accounting_for_profile(
        log_path,
        "raw_single",
    ) is None
    assert mapping_cloud_accounting.mapping_cloud_accounting_for_profile(
        log_path, "dimos_odom_accumulated"
    ) is not None
    assert mapping_cloud_accounting.mapping_cloud_accounting_for_profile(
        log_path, "dimos_odom_accumulated_emit10"
    ) is not None


def test_given_malformed_terminal_accounting_when_parsed_then_typed_failure_is_explicit() -> None:
    # Given: marker 뒤 JSON이 잘린 terminal line
    log_text = "[node-1] MAPPING_CLOUD_ACCOUNTING {not-json}\n"

    # When/Then: parse failure는 silence가 아닌 typed reason code가 된다.
    from go2_validation.mapping_cloud_accounting import (
        MappingCloudAccountingError,
        parse_mapping_cloud_accounting_log,
    )

    with pytest.raises(MappingCloudAccountingError) as caught:
        parse_mapping_cloud_accounting_log(log_text)

    assert caught.value.reason_code == "mapping_cloud_accounting_json_invalid"


def test_given_missing_terminal_accounting_when_parsed_then_typed_failure_is_explicit() -> None:
    # Given: accumulator log가 marker 없이 종료된 실행
    log_text = "[go2_mapping_cloud_accumulator-5] shutdown complete\n"

    # When/Then: accounting absence는 diagnosable typed failure가 된다.
    from go2_validation.mapping_cloud_accounting import (
        MappingCloudAccountingError,
        parse_mapping_cloud_accounting_log,
    )

    with pytest.raises(MappingCloudAccountingError) as caught:
        parse_mapping_cloud_accounting_log(log_text)

    assert caught.value.reason_code == "mapping_cloud_accounting_missing"


def test_given_multiple_terminal_accounting_lines_when_parsed_then_typed_failure_is_explicit() -> None:
    # Given: terminal marker를 두 번 출력한 corrupted launch log
    log_text = "\n".join(
        (
            f"[node-1] MAPPING_CLOUD_ACCOUNTING {_accounting_json()}",
            f"[node-1] MAPPING_CLOUD_ACCOUNTING {_accounting_json()}",
        )
    )

    # When/Then: ambiguity는 typed failure로 남고 임의의 line을 선택하지 않는다.
    from go2_validation.mapping_cloud_accounting import (
        MappingCloudAccountingError,
        parse_mapping_cloud_accounting_log,
    )

    with pytest.raises(MappingCloudAccountingError) as caught:
        parse_mapping_cloud_accounting_log(log_text)

    assert caught.value.reason_code == "mapping_cloud_accounting_multiple"
