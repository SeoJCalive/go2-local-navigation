"""Rosbag player의 pause·discovery·Resume 시작 계약을 검증한다."""

from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import pytest


@dataclass(frozen=True, slots=True)
class _Endpoint:
    node_name: str
    node_namespace: str = "/"


class _Future:
    def __init__(self, node: "_Node", response_present: bool = True) -> None:
        self._node = node
        self._response_present = response_present

    def done(self) -> bool:
        return self._node.stage >= 4

    def exception(self) -> None:
        return None

    def result(self):
        self._node.events.append("resume_result")
        return SimpleNamespace() if self._response_present else None


class _ResumeClient:
    def __init__(self, node: "_Node", response_present: bool = True) -> None:
        self._node = node
        self._response_present = response_present
        self.request = None

    def service_is_ready(self) -> bool:
        self._node.events.append(f"service_checked:{self._node.stage}")
        return self._node.stage >= 3

    def call_async(self, request):
        self.request = request
        self._node.events.append(f"resume_called:{self._node.stage}")
        return _Future(self._node, self._response_present)


class _Node:
    def __init__(self, response_present: bool = True) -> None:
        self.stage = 0
        self.events: list[str] = []
        self.client = _ResumeClient(self, response_present)
        self.service_name = ""

    def create_client(self, _service_type, service_name: str) -> _ResumeClient:
        self.service_name = service_name
        return self.client

    def get_publishers_info_by_topic(self, topic: str) -> list[_Endpoint]:
        if topic == "/utlidar/cloud" and self.stage >= 1:
            return [_Endpoint("rosbag2_player")]
        if topic == "/utlidar/robot_odom" and self.stage == 1:
            return [_Endpoint("unrelated_player")]
        if topic == "/utlidar/robot_odom" and self.stage >= 2:
            return [_Endpoint("rosbag2_player")]
        return []

    def spin_once(self) -> None:
        self.stage += 1


def test_given_mapping_bag_when_player_command_is_built_then_it_starts_paused() -> None:
    # Given: startup discovery 전에 message를 진행하면 안 되는 mapping bag
    from go2_validation.mapping_runtime_execution import mapping_bag_play_command

    # When: 외부 rosbag player argv를 만든다.
    command = mapping_bag_play_command(Path("data/external/derived/full"), 1.0)

    # Then: player는 명시적인 Resume handshake 전까지 pause 상태로 시작한다.
    assert "--start-paused" in command


def test_given_mapping_bag_when_command_is_built_then_delay_is_one_second() -> None:
    # Given: startup readiness를 위해 고정 지연이 필요한 mapping bag
    from go2_validation.mapping_runtime_execution import mapping_bag_play_command

    # When: 외부 rosbag player argv를 만든다.
    command = mapping_bag_play_command(Path("data/external/derived/full"), 1.0)

    # Then: delay option은 한 번만 있고 바로 다음 토큰이 정확히 1.0초다.
    delay_index = command.index("--delay")
    assert command.count("--delay") == 1
    assert command[delay_index : delay_index + 2] == ("--delay", "1.0")


def test_given_staged_player_graph_when_synchronized_then_resume_precedes_deadline(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Given: cloud, odometry, service, Resume 응답이 서로 다른 spin에 준비되는 graph
    from go2_validation import mapping_player_services

    node = _Node()
    clock_values: list[float] = []

    def observe_clock() -> float:
        value = len(clock_values) * 0.01
        clock_values.append(value)
        node.events.append("clock")
        return value

    monkeypatch.setattr(mapping_player_services, "monotonic", observe_clock)
    monkeypatch.setattr(mapping_player_services.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        mapping_player_services.rclpy,
        "spin_once",
        lambda observed_node, timeout_sec: observed_node.spin_once(),
    )
    services = mapping_player_services.MappingPlayerServices(node)

    # When: paused player startup을 synchronize한다.
    deadline = services.synchronize_start(1.0, 1.0, 12.0)

    # Then: exact service를 호출하고 successful response 뒤 clock으로 deadline을 만든다.
    assert node.service_name == "/rosbag2_player/resume"
    assert node.stage == 4
    assert "service_checked:1" not in node.events
    assert node.events.count("service_checked:2") == 1
    assert node.events.count("service_checked:3") == 1
    assert node.events.count("resume_called:3") == 1
    assert node.events[-2:] == ["resume_result", "clock"]
    assert deadline == clock_values[-1] + 12.0


def test_given_incomplete_player_graph_when_readiness_expires_then_typed_reason_is_raised(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Given: 두 publisher와 Resume service가 아직 함께 준비되지 않은 graph
    from go2_validation import mapping_player_services

    node = _Node()
    monkeypatch.setattr(mapping_player_services.rclpy, "ok", lambda: True)
    services = mapping_player_services.MappingPlayerServices(node)

    # When: readiness deadline이 즉시 만료된다.
    with pytest.raises(mapping_player_services.MappingRuntimeError) as captured:
        services.wait_until_ready(0.0)

    # Then: 호출자가 분기 가능한 readiness reason code를 받는다.
    assert captured.value.reason_code == "mapping_player_readiness_failed"
    assert node.events == []


def test_given_absent_resume_response_when_resume_completes_then_typed_reason_is_raised(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Given: future는 완료됐지만 empty service response object가 없는 player
    from go2_validation import mapping_player_services

    node = _Node(response_present=False)
    node.stage = 4
    monkeypatch.setattr(mapping_player_services.rclpy, "ok", lambda: True)
    services = mapping_player_services.MappingPlayerServices(node)

    # When: Resume 완료 결과를 검증한다.
    with pytest.raises(mapping_player_services.MappingRuntimeError) as captured:
        services.resume(1.0)

    # Then: response 부재는 성공으로 승격되지 않는다.
    assert captured.value.reason_code == "mapping_player_resume_failed"
    assert captured.value.detail == "response_absent"
