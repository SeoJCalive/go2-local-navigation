
"""통합 preflight runner의 ROS parameter를 안전한 실행값으로 파싱한다."""
from dataclasses import dataclass
from pathlib import Path
import re

from ament_index_python.packages import get_package_prefix
from rclpy.node import Node


RUN_LABEL_PATTERN = re.compile(r"^[a-z0-9][a-z0-9_-]*$")


@dataclass(frozen=True, slots=True)
class RunConfiguration:
    """runner ROS parameter에서 파싱한 실행 경계다."""

    duration_seconds: int
    run_label: str
    output_root: Path


@dataclass(frozen=True, slots=True)
class ConfigurationError(Exception):
    """runner parameter가 안전한 실행 경계를 벗어났음을 나타낸다."""

    detail: str

    def __str__(self) -> str:
        return self.detail


def parse_configuration(node: Node) -> RunConfiguration:
    """duration·label·output root를 검증한 불변 설정으로 변환한다."""
    duration = node.declare_parameter(
        "duration_sec", 30
    ).get_parameter_value().integer_value
    label = node.declare_parameter(
        "run_label", "preflight"
    ).get_parameter_value().string_value
    output_value = node.declare_parameter(
        "output_root", ""
    ).get_parameter_value().string_value
    if duration < 20 or duration > 7200:
        raise ConfigurationError("duration_sec must be between 20 and 7200")
    if RUN_LABEL_PATTERN.fullmatch(label) is None:
        raise ConfigurationError("run_label must use lowercase letters, digits, _ or -")
    project_root = Path(get_package_prefix("go2_validation")).parents[1]
    output_root = (
        Path(output_value)
        if output_value
        else project_root / "data" / "runs" / "preflight"
    )
    return RunConfiguration(
        duration_seconds=duration,
        run_label=label,
        output_root=output_root,
    )
