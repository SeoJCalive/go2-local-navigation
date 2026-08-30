
"""격리 navigation 실행 전에 domain·clock·시간 source를 판정한다."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
import os
from pathlib import Path
from typing import Final

import yaml


SOURCE_MODE_CONFIGURATION_PATH: Final = (
    Path(__file__).parents[1] / "config/execution_modes.yaml"
)


@dataclass(frozen=True, slots=True)
class ModeAssessment:
    """Shell 종료 코드와 단일 실패 사유를 보존한다."""

    exit_code: int
    reason_code: str | None


@dataclass(frozen=True, slots=True)
class ModeProfile:
    """YAML에서 파싱한 mode별 격리 기준이다."""

    domain_id: int
    clock_owner: str
    use_sim_time: bool
    loopback_only: bool


@dataclass(frozen=True, slots=True)
class RuntimeObservation:
    """실행 직전 관찰한 domain·clock·시간 source다."""

    mode: str
    domain_id: int
    declared_clock_owner: str
    clock_publisher_count: int
    use_sim_time: bool
    loopback_only: bool


def load_mode_profile(mode: str) -> ModeProfile | None:
    """Mode YAML의 한 행을 불변 profile로 파싱한다."""
    configuration_path = SOURCE_MODE_CONFIGURATION_PATH
    if not configuration_path.is_file():
        from ament_index_python.packages import get_package_share_directory

        configuration_path = (
            Path(get_package_share_directory("go2_validation"))
            / "config"
            / "execution_modes.yaml"
        )
    document = yaml.safe_load(configuration_path.read_text(encoding="utf-8"))
    raw = document["modes"].get(mode)
    if raw is None:
        return None
    return ModeProfile(
        domain_id=int(raw["domain_id"]),
        clock_owner=str(raw["clock_owner"]),
        use_sim_time=bool(raw["use_sim_time"]),
        loopback_only=bool(raw["loopback_only"]),
    )


def assess_mode(mode: str, domain_id: int, clock_owner: str) -> ModeAssessment:
    """Reject a mode that can cross its domain or simulated-clock boundary."""
    profile = load_mode_profile(mode)
    if profile is None:
        return ModeAssessment(exit_code=2, reason_code="unknown_mode")
    if domain_id != profile.domain_id:
        return ModeAssessment(exit_code=2, reason_code="domain_mismatch")
    if clock_owner != profile.clock_owner:
        return ModeAssessment(exit_code=2, reason_code="clock_owner_mismatch")
    return ModeAssessment(exit_code=0, reason_code=None)


def assess_runtime_observation(observation: RuntimeObservation) -> ModeAssessment:
    """실행 직전 mode 관찰값을 fail-fast 순서로 판정한다."""
    profile = load_mode_profile(observation.mode)
    if profile is None:
        return ModeAssessment(2, "unknown_mode")
    basic = assess_mode(
        observation.mode,
        observation.domain_id,
        observation.declared_clock_owner,
    )
    if basic.exit_code != 0:
        return basic
    if observation.use_sim_time != profile.use_sim_time:
        return ModeAssessment(2, "sim_time_mismatch")
    if observation.loopback_only != profile.loopback_only:
        return ModeAssessment(2, "network_isolation_mismatch")
    if observation.clock_publisher_count > 1:
        return ModeAssessment(2, "duplicate_clock_publishers")
    if profile.clock_owner == "none" and observation.clock_publisher_count != 0:
        return ModeAssessment(2, "unexpected_clock_publisher")
    return ModeAssessment(0, None)


def _write_result(path: Path, observation: RuntimeObservation, result: ModeAssessment) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.partial")
    document = {
        "schema_version": 1,
        "record_kind": "navigation_runtime_preflight_result",
        "recorded_at": datetime.now().astimezone().isoformat(),
        "mode": observation.mode,
        "domain_id": observation.domain_id,
        "declared_clock_owner": observation.declared_clock_owner,
        "clock_publisher_count": observation.clock_publisher_count,
        "use_sim_time": observation.use_sim_time,
        "loopback_only": observation.loopback_only,
        "status": "passed" if result.exit_code == 0 else "failed",
        "reason_code": result.reason_code,
    }
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def main(args: list[str] | None = None) -> None:
    """ROS parameter와 graph를 읽고 preflight JSON을 기록한다."""
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=args)
    node = Node("go2_navigation_runtime_preflight")
    exit_code = 2
    try:
        mode = node.declare_parameter("mode", "offline_fault").value
        profile = load_mode_profile(str(mode))
        if profile is None:
            profile = ModeProfile(-1, "unknown", True, True)
        output_value = node.declare_parameter(
            "output_path",
            "data/runs/runtime_preflight/result.json",
        ).value
        clock_owner = node.declare_parameter(
            "declared_clock_owner",
            profile.clock_owner,
        ).value
        use_sim_time = node.declare_parameter(
            "runtime_use_sim_time",
            profile.use_sim_time,
        ).value
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
        observation = RuntimeObservation(
            mode=str(mode),
            domain_id=int(os.environ.get("ROS_DOMAIN_ID", "0")),
            declared_clock_owner=str(clock_owner),
            clock_publisher_count=len(node.get_publishers_info_by_topic("/clock")),
            use_sim_time=bool(use_sim_time),
            loopback_only='name="lo"' in os.environ.get("CYCLONEDDS_URI", ""),
        )
        result = assess_runtime_observation(observation)
        _write_result(Path(str(output_value)), observation, result)
        exit_code = result.exit_code
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
