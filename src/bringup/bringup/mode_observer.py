"""실행 mode의 domain·live 환경·global TF owner 계약을 정의한다."""

from dataclasses import dataclass
from enum import Enum
from typing import Final

from bringup.preflight_types import CheckResult, CheckStatus


class ExecutionMode(str, Enum):
    """software-only navigation이 지원하는 폐쇄된 실행 mode다."""

    FAULT_RECOVERY = "fault_recovery"
    SCAN_REPLAY = "scan_replay"
    MAPPING = "mapping"
    LOCALIZATION = "localization"
    SYNTHETIC_NAVIGATION = "synthetic_navigation"
    LIVE_SHADOW = "live_shadow"


@dataclass(frozen=True, slots=True)
class ModeEnvironment:
    """ROS observer가 읽은 mode 판정용 최소 환경값이다."""

    rmw_implementation: str
    ros_domain_id: str
    go2_interface: str
    cyclonedds_uri: str


@dataclass(frozen=True, slots=True)
class GlobalTfOwnerContract:
    """mode별 global TF edge와 endpoint owner cardinality 계약이다."""

    parent_frame: str | None
    child_frame: str | None
    owner_node: str | None
    owner_count: int


@dataclass(frozen=True, slots=True)
class ModeContract:
    """실행 mode가 요구하는 ROS domain·환경·global TF 계약이다."""

    ros_domain_id: str
    requires_live_network: bool
    global_tf: GlobalTfOwnerContract


NO_GLOBAL_TF: Final = GlobalTfOwnerContract(
    parent_frame=None,
    child_frame=None,
    owner_node=None,
    owner_count=0,
)
MAP_SLAM_TF: Final = GlobalTfOwnerContract(
    parent_frame="map",
    child_frame="odom",
    owner_node="/slam_toolbox",
    owner_count=1,
)
MAP_AMCL_TF: Final = GlobalTfOwnerContract(
    parent_frame="map",
    child_frame="odom",
    owner_node="/amcl",
    owner_count=1,
)
MAP_FIXTURE_TF: Final = GlobalTfOwnerContract(
    parent_frame="map",
    child_frame="odom",
    owner_node="/synthetic_navigation_fixture",
    owner_count=1,
)
SHADOW_MAP_SLAM_TF: Final = GlobalTfOwnerContract(
    parent_frame="go2_shadow_map",
    child_frame="odom",
    owner_node="/slam_toolbox",
    owner_count=1,
)
MODE_CONTRACTS: Final = {
    ExecutionMode.FAULT_RECOVERY: ModeContract("61", False, NO_GLOBAL_TF),
    ExecutionMode.SCAN_REPLAY: ModeContract("62", False, NO_GLOBAL_TF),
    ExecutionMode.MAPPING: ModeContract("63", False, MAP_SLAM_TF),
    ExecutionMode.LOCALIZATION: ModeContract("64", False, MAP_AMCL_TF),
    ExecutionMode.SYNTHETIC_NAVIGATION: ModeContract("65", False, MAP_FIXTURE_TF),
    ExecutionMode.LIVE_SHADOW: ModeContract("0", True, SHADOW_MAP_SLAM_TF),
}


def mode_contract(mode: ExecutionMode) -> ModeContract:
    """실행 mode의 불변 계약을 반환한다."""
    return MODE_CONTRACTS[mode]


def assess_mode_environment(
    mode: ExecutionMode,
    environment: ModeEnvironment,
) -> CheckResult:
    """offline은 격리 domain만, live는 AGX network 환경까지 판정한다."""
    contract = mode_contract(mode)
    domain_matches = environment.ros_domain_id == contract.ros_domain_id
    live_network_matches = (
        environment.rmw_implementation == "rmw_cyclonedds_cpp"
        and environment.go2_interface == "eno1"
        and 'name="eno1"' in environment.cyclonedds_uri
    )
    passed = domain_matches and (
        live_network_matches if contract.requires_live_network else True
    )
    return CheckResult(
        check_id="mode.environment",
        status=CheckStatus.PASS if passed else CheckStatus.FAIL,
        detail=(
            f"mode={mode.value}; expected_domain={contract.ros_domain_id}; "
            f"observed_domain={environment.ros_domain_id}; "
            f"live_network_required={contract.requires_live_network}; "
            f"interface={environment.go2_interface}"
        ),
    )
