from math import isclose
from pathlib import Path


def test_given_seven_candidates_when_sweep_is_defined_then_resolution_and_center_are_preserved() -> None:
    # Given: 기존 합격값을 중심으로 한 coarse search sweep.
    from go2_validation.mapping_coarse_search_sweep_runner import (
        COARSE_SEARCH_ANGLE_OFFSETS,
    )

    # When: 후보 간격을 계산한다.
    intervals = tuple(
        right - left
        for left, right in zip(
            COARSE_SEARCH_ANGLE_OFFSETS[:-1],
            COARSE_SEARCH_ANGLE_OFFSETS[1:],
            strict=True,
        )
    )

    # Then: 4~16도 범위의 7개 값과 2도 간격, 중앙 10도를 보존한다.
    assert len(COARSE_SEARCH_ANGLE_OFFSETS) == 7
    assert isclose(COARSE_SEARCH_ANGLE_OFFSETS[0], 0.0698)
    assert isclose(COARSE_SEARCH_ANGLE_OFFSETS[3], 0.1745)
    assert isclose(COARSE_SEARCH_ANGLE_OFFSETS[-1], 0.2792)
    assert all(isclose(interval, 0.0349) for interval in intervals)


def test_given_external_replay_when_sweep_specs_are_built_then_only_coarse_bound_changes() -> None:
    # Given: canonical short bag의 typed replay identity.
    from go2_validation.mapping_coarse_search_sweep_runner import (
        COARSE_SEARCH_ANGLE_OFFSETS,
        build_mapping_coarse_search_specs,
    )
    from go2_validation.mapping_runtime_data import BagExpectation
    from go2_validation.mapping_tf_profile_ab_input import MappingTfProfileAbInput

    replay = MappingTfProfileAbInput(
        bag_path=Path("/fixture/short"),
        provenance="external_dynamic",
        source_checksum="source-sha256",
        replay_checksum="replay-sha256",
        expectation=BagExpectation(1_843, 18_026, 0, 120_000_000_000),
    )

    # When: 7개 runtime spec을 만든다.
    specs = build_mapping_coarse_search_specs(replay)

    # Then: coarse 값 외의 replay·profile·안전 경계는 모두 동일하다.
    assert tuple(spec.coarse_search_angle_offset for spec in specs) == (
        COARSE_SEARCH_ANGLE_OFFSETS
    )
    assert all(spec.sensor_tf_profile == "dimos_replay" for spec in specs)
    assert all(
        spec.scan_projection_profile == "dimos_odom_accumulated_emit3"
        for spec in specs
    )
    assert all(spec.execution_mode == "external_replay" for spec in specs)
    assert all(spec.continuity_profile == "replay_enforce" for spec in specs)
    assert all(spec.use_response_expansion is False for spec in specs)
    assert all(spec.do_loop_closing is True for spec in specs)
    assert all(spec.playback_rate == 1.0 for spec in specs)


def test_given_lower_range_parameter_when_parsed_then_five_offsets_are_preserved() -> None:
    # Given: 0~4도 구간을 1도 간격으로 지정한 실행 입력.
    from go2_validation.mapping_coarse_search_sweep_runner import parse_angle_offsets

    # When: comma-separated radian 후보를 파싱한다.
    offsets = parse_angle_offsets("0.0,0.01745,0.03490,0.05236,0.06980")

    # Then: 입력 순서와 수치가 그대로 typed 후보 tuple로 보존된다.
    assert offsets == (0.0, 0.01745, 0.03490, 0.05236, 0.06980)


def test_given_custom_offsets_when_specs_are_built_then_only_requested_candidates_are_emitted() -> None:
    # Given: canonical short bag identity와 lower-range 후보.
    from go2_validation.mapping_coarse_search_sweep_runner import (
        build_mapping_coarse_search_specs,
    )
    from go2_validation.mapping_runtime_data import BagExpectation
    from go2_validation.mapping_tf_profile_ab_input import MappingTfProfileAbInput

    replay = MappingTfProfileAbInput(
        bag_path=Path("/fixture/short"),
        provenance="external_dynamic",
        source_checksum="source-sha256",
        replay_checksum="replay-sha256",
        expectation=BagExpectation(1_843, 18_026, 0, 120_000_000_000),
    )

    # When: lower-range 후보만으로 runtime spec을 만든다.
    offsets = (0.0, 0.01745, 0.03490, 0.05236, 0.06980)
    specs = build_mapping_coarse_search_specs(replay, offsets)

    # Then: 요청한 후보만 같은 external replay 계약으로 생성된다.
    assert tuple(spec.coarse_search_angle_offset for spec in specs) == offsets
    assert all(spec.execution_mode == "external_replay" for spec in specs)
    assert all(spec.continuity_profile == "replay_enforce" for spec in specs)
