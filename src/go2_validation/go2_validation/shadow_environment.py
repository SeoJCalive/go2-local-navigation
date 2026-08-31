"""Domain 65 Nav2 shadow가 외부 네트워크와 분리됐는지 판정한다.

이 모듈은 process를 시작하지 않는다. runner가 전달한 domain, RMW, interface,
CycloneDDS URI와 simulated-time 선택을 순수하게 검사해 실패 reason code를 반환한다.
"""

import os
from dataclasses import dataclass
from typing import Final

SHADOW_DOMAIN_ID: Final = 65
SHADOW_RMW: Final = "rmw_cyclonedds_cpp"
SHADOW_INTERFACE: Final = "lo"


@dataclass(frozen=True, slots=True)
class ShadowEnvironment:
    """Domain 65 child를 시작하기 전에 확정할 실행 환경이다."""

    ros_domain_id: int
    rmw_implementation: str
    go2_interface: str
    cyclonedds_uri: str
    use_sim_time: bool


def current_shadow_environment(*, use_sim_time: bool) -> ShadowEnvironment:
    """현재 process environment를 불변 판정 입력으로 읽는다."""
    raw_domain_id = os.environ.get("ROS_DOMAIN_ID", "")
    try:
        domain_id = int(raw_domain_id)
    except ValueError:
        domain_id = -1
    return ShadowEnvironment(
        ros_domain_id=domain_id,
        rmw_implementation=os.environ.get("RMW_IMPLEMENTATION", ""),
        go2_interface=os.environ.get("GO2_AGX_INTERFACE", ""),
        cyclonedds_uri=os.environ.get("CYCLONEDDS_URI", ""),
        use_sim_time=use_sim_time,
    )


def assess_shadow_environment(environment: ShadowEnvironment) -> str | None:
    """첫 불일치 reason code를 반환하고 모든 경계가 맞으면 None을 반환한다."""
    if environment.ros_domain_id != SHADOW_DOMAIN_ID:
        return "shadow_domain_mismatch"
    if environment.rmw_implementation != SHADOW_RMW:
        return "shadow_rmw_mismatch"
    if environment.go2_interface != SHADOW_INTERFACE:
        return "shadow_interface_mismatch"
    if 'name="lo"' not in environment.cyclonedds_uri:
        return "shadow_cyclonedds_interface_mismatch"
    if 'multicast="false"' not in environment.cyclonedds_uri:
        return "shadow_multicast_must_be_disabled"
    if not environment.use_sim_time:
        return "shadow_sim_time_required"
    return None
