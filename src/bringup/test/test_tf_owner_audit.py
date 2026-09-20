"""현재 fault·synthetic·live mode의 global TF 소유권을 검증한다."""

import pytest

from bringup.mode_observer import ExecutionMode
from bringup.preflight_types import CheckStatus
from bringup.tf_owner_audit import (
    TfCallbackTransform,
    TfEndpointOwner,
    audit_global_tf_owners,
    audit_teardown_tf_owners,
)


SLAM_ENDPOINT = TfEndpointOwner(b"slam", "slam_toolbox", "/")
FIXTURE_ENDPOINT = TfEndpointOwner(
    b"fixture",
    "synthetic_navigation_fixture",
    "/",
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


@pytest.mark.parametrize(
    "mode",
    (
        ExecutionMode.FAULT_RECOVERY,
    ),
)
def test_given_zero_global_tf_when_mode_requires_none_then_audit_passes(
    mode: ExecutionMode,
) -> None:
    assert all(status is CheckStatus.PASS for status in checks_by_id(mode, (), ()).values())


def test_given_fixture_tf_when_synthetic_navigation_runs_then_audit_passes() -> None:
    callbacks = (TfCallbackTransform("map", "odom", b"fixture"),)

    assert all(
        status is CheckStatus.PASS
        for status in checks_by_id(
            ExecutionMode.SYNTHETIC_NAVIGATION,
            (FIXTURE_ENDPOINT,),
            callbacks,
        ).values()
    )


def test_given_canonical_map_edge_when_live_shadow_runs_then_edge_is_rejected() -> None:
    callbacks = (TfCallbackTransform("map", "odom", b"slam"),)
    checks = checks_by_id(ExecutionMode.LIVE_SHADOW, (SLAM_ENDPOINT,), callbacks)

    assert checks["tf.global_edge"] is CheckStatus.FAIL


def test_given_unknown_callback_gid_when_live_shadow_runs_then_owner_resolution_fails() -> None:
    callbacks = (TfCallbackTransform("go2_shadow_map", "odom", b"unknown"),)
    checks = checks_by_id(ExecutionMode.LIVE_SHADOW, (SLAM_ENDPOINT,), callbacks)

    assert checks["tf.global_owner_resolution"] is CheckStatus.FAIL


def test_given_no_global_tf_after_teardown_then_audit_passes() -> None:
    assert audit_teardown_tf_owners(()).status is CheckStatus.PASS
