"""TF callback publisher GID를 ROS endpoint owner로 해석해 global edge를 감사한다."""

from dataclasses import dataclass

from bringup.mode_observer import ExecutionMode, mode_contract
from bringup.preflight_types import CheckResult, CheckStatus


@dataclass(frozen=True, slots=True)
class TfEndpointOwner:
    """TF publisher endpoint의 GID와 ROS node owner다."""

    publisher_gid: bytes
    node_name: str
    node_namespace: str

    def node_path(self) -> str:
        """ROS endpoint의 namespace와 node name을 canonical path로 만든다."""
        return (
            f"/{self.node_name}"
            if self.node_namespace == "/"
            else f"{self.node_namespace.rstrip('/')}/{self.node_name}"
        )


@dataclass(frozen=True, slots=True)
class TfCallbackTransform:
    """callback message에서 분리한 transform edge와 publisher GID다."""

    parent_frame: str
    child_frame: str
    publisher_gid: bytes


def audit_global_tf_owners(
    mode: ExecutionMode,
    endpoints: tuple[TfEndpointOwner, ...],
    callbacks: tuple[TfCallbackTransform, ...],
) -> tuple[CheckResult, ...]:
    """Mode contract의 global TF edge, endpoint owner, cardinality를 판정한다."""
    contract = mode_contract(mode).global_tf
    global_callbacks = tuple(
        callback for callback in callbacks if callback.child_frame == "odom"
    )
    endpoint_by_gid = {endpoint.publisher_gid: endpoint for endpoint in endpoints}
    unknown_gids = tuple(
        callback.publisher_gid
        for callback in global_callbacks
        if callback.publisher_gid not in endpoint_by_gid
    )
    expected_callbacks = tuple(
        callback
        for callback in global_callbacks
        if (
            callback.parent_frame == contract.parent_frame
            and callback.child_frame == contract.child_frame
        )
    )
    observed_edges = tuple(
        sorted({(callback.parent_frame, callback.child_frame) for callback in global_callbacks})
    )
    expected_edge = (contract.parent_frame, contract.child_frame)
    expected_edge_set = () if contract.parent_frame is None else (expected_edge,)
    owner_gids = {callback.publisher_gid for callback in expected_callbacks}
    owner_paths = tuple(
        sorted(
            endpoint_by_gid[gid].node_path()
            for gid in owner_gids
            if gid in endpoint_by_gid
        )
    )
    edges_match = observed_edges == expected_edge_set
    cardinality_matches = len(owner_gids) == contract.owner_count
    owners_match = (
        contract.owner_node is None
        or owner_paths == (contract.owner_node,)
    )
    return (
        CheckResult(
            check_id="tf.global_owner_resolution",
            status=CheckStatus.PASS if not unknown_gids else CheckStatus.FAIL,
            detail=f"unknown_callback_gids={unknown_gids}",
        ),
        CheckResult(
            check_id="tf.global_edge",
            status=CheckStatus.PASS if edges_match else CheckStatus.FAIL,
            detail=f"expected={expected_edge_set}; observed={observed_edges}",
        ),
        CheckResult(
            check_id="tf.global_owner_cardinality",
            status=(
                CheckStatus.PASS if cardinality_matches else CheckStatus.FAIL
            ),
            detail=(
                f"expected={contract.owner_count}; observed={len(owner_gids)}; "
                f"gids={tuple(sorted(owner_gids))}"
            ),
        ),
        CheckResult(
            check_id="tf.global_owner_identity",
            status=CheckStatus.PASS if owners_match else CheckStatus.FAIL,
            detail=f"expected={contract.owner_node}; observed={owner_paths}",
        ),
    )


def audit_teardown_tf_owners(
    callbacks: tuple[TfCallbackTransform, ...],
) -> CheckResult:
    """Teardown 뒤 남은 global TF edge owner가 없는지 판정한다."""
    residual_global_gids = {
        callback.publisher_gid
        for callback in callbacks
        if callback.child_frame == "odom"
    }
    return CheckResult(
        check_id="tf.teardown_global_owner_cardinality",
        status=(
            CheckStatus.PASS if not residual_global_gids else CheckStatus.FAIL
        ),
        detail=f"observed={len(residual_global_gids)}; gids={tuple(sorted(residual_global_gids))}",
    )
