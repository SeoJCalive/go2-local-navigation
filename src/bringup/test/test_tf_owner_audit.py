"""global TF edge의 GID owner cardinality 계약을 검증한다."""

from bringup.mode_observer import ExecutionMode
from bringup.preflight_types import CheckStatus
from bringup.tf_owner_audit import (
    TfCallbackTransform,
    TfEndpointOwner,
    audit_global_tf_owners,
    audit_teardown_tf_owners,
)

SLAM_ENDPOINT = TfEndpointOwner(
    publisher_gid=b"slam",
    node_name="slam_toolbox",
    node_namespace="/",
)
AMCL_ENDPOINT = TfEndpointOwner(
    publisher_gid=b"amcl",
    node_name="amcl",
    node_namespace="/",
)
FIXTURE_ENDPOINT = TfEndpointOwner(
    publisher_gid=b"fixture",
    node_name="synthetic_navigation_fixture",
    node_namespace="/",
)


def checks_by_id(
    mode: ExecutionMode,
    endpoints: tuple[TfEndpointOwner, ...],
    callbacks: tuple[TfCallbackTransform, ...],
) -> dict[str, CheckStatus]:
    return {
        check.check_id: check.status
        for check in audit_global_tf_owners(mode, endpoints, callbacks)
    }


def test_mapping_requires_one_slam_owned_map_to_odom_edge() -> None:
    # Given: SLAM endpoint GID가 map→odom callback 하나를 보낸 관찰값
    callbacks = (
        TfCallbackTransform("map", "odom", b"slam"),
    )

    # When: mapping mode의 global TF owner를 감사한다.
    checks = checks_by_id(ExecutionMode.MAPPING, (SLAM_ENDPOINT,), callbacks)

    # Then: edge, cardinality, endpoint owner 모두 통과한다.
    assert all(status is CheckStatus.PASS for status in checks.values())


def test_fault_and_scan_replay_require_no_global_tf_owner() -> None:
    # Given: global TF callback이 없는 offline mode 관찰값
    callbacks: tuple[TfCallbackTransform, ...] = ()

    # When: fault와 scan-replay owner를 각각 감사한다.
    fault_checks = checks_by_id(ExecutionMode.FAULT_RECOVERY, (), callbacks)
    replay_checks = checks_by_id(ExecutionMode.SCAN_REPLAY, (), callbacks)

    # Then: 두 mode 모두 global owner 0을 통과한다.
    assert all(status is CheckStatus.PASS for status in fault_checks.values())
    assert all(status is CheckStatus.PASS for status in replay_checks.values())


def test_duplicate_gid_owners_for_same_global_edge_fail_cardinality() -> None:
    # Given: 서로 다른 GID가 동일한 map→odom edge를 publish한 관찰값
    callbacks = (
        TfCallbackTransform("map", "odom", b"slam"),
        TfCallbackTransform("map", "odom", b"second-slam"),
    )
    second_slam = TfEndpointOwner(
        publisher_gid=b"second-slam",
        node_name="slam_toolbox_secondary",
        node_namespace="/",
    )

    # When: mapping mode의 global TF owner를 감사한다.
    checks = checks_by_id(
        ExecutionMode.MAPPING,
        (SLAM_ENDPOINT, second_slam),
        callbacks,
    )

    # Then: owner cardinality가 두 개여서 실패한다.
    assert checks["tf.global_owner_cardinality"] is CheckStatus.FAIL


def test_live_shadow_rejects_canonical_map_to_odom_edge() -> None:
    # Given: SLAM endpoint가 live shadow의 canonical map→odom을 publish한다.
    callbacks = (
        TfCallbackTransform("map", "odom", b"slam"),
    )

    # When: live shadow mode의 global TF owner를 감사한다.
    checks = checks_by_id(ExecutionMode.LIVE_SHADOW, (SLAM_ENDPOINT,), callbacks)

    # Then: go2_shadow_map→odom이 아니므로 edge check가 실패한다.
    assert checks["tf.global_edge"] is CheckStatus.FAIL


def test_localization_rejects_wrong_endpoint_owner() -> None:
    # Given: SLAM GID가 localization map→odom callback을 publish한다.
    callbacks = (
        TfCallbackTransform("map", "odom", b"slam"),
    )

    # When: localization mode의 global TF owner를 감사한다.
    checks = checks_by_id(ExecutionMode.LOCALIZATION, (SLAM_ENDPOINT,), callbacks)

    # Then: AMCL owner가 아니므로 owner check가 실패한다.
    assert checks["tf.global_owner_identity"] is CheckStatus.FAIL


def test_synthetic_navigation_requires_fixture_owner() -> None:
    # Given: fixture endpoint GID가 map→odom callback을 publish한다.
    callbacks = (
        TfCallbackTransform("map", "odom", b"fixture"),
    )

    # When: synthetic navigation mode의 global TF owner를 감사한다.
    checks = checks_by_id(
        ExecutionMode.SYNTHETIC_NAVIGATION,
        (FIXTURE_ENDPOINT,),
        callbacks,
    )

    # Then: fixture 단독 owner 계약을 통과한다.
    assert all(status is CheckStatus.PASS for status in checks.values())


def test_unknown_callback_gid_fails_owner_resolution() -> None:
    # Given: endpoint graph에 없는 GID의 global TF callback
    callbacks = (
        TfCallbackTransform("map", "odom", b"unknown"),
    )

    # When: mapping mode의 global TF owner를 감사한다.
    checks = checks_by_id(ExecutionMode.MAPPING, (SLAM_ENDPOINT,), callbacks)

    # Then: callback GID를 endpoint owner로 해석하지 못해 실패한다.
    assert checks["tf.global_owner_resolution"] is CheckStatus.FAIL


def test_teardown_requires_zero_global_tf_owners() -> None:
    # Given: teardown 뒤 global TF callback이 없는 관찰값
    callbacks: tuple[TfCallbackTransform, ...] = ()

    # When: teardown owner cardinality를 감사한다.
    check = audit_teardown_tf_owners(callbacks)

    # Then: global TF owner 0을 통과한다.
    assert check.status is CheckStatus.PASS
