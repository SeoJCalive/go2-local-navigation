
"""Fixture boundary JSON과 실제 downstream stamp를 결합한다.
이 모듈은 ROS graph와 process를 소유하지 않는다. fault fixture가 선언한 시점에
실제로 관찰된 validated cloud, scan, odometry, TF만 `FixtureEvent`로 투영한다.
"""

from dataclasses import dataclass
import json

from go2_validation.fault_fixture_model import FixtureEvent, FixturePhase, OutputCounts


@dataclass(frozen=True, slots=True)
class FixtureEventMarker:
    """Fixture event topic에서 파싱한 output 미포함 경계다."""

    phase: FixturePhase
    clock_nanoseconds: int
    reason_code: str | None
    child_exit_code: int | None


@dataclass(frozen=True, slots=True)
class StreamStampObservation:
    """한 attempt에서 실제 수신한 네 output의 source stamp다."""

    validated_cloud: tuple[int, ...]
    scan: tuple[int, ...]
    odom: tuple[int, ...]
    tf: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class FixtureEventParseError(Exception):
    """Fixture event JSON이 폐쇄된 runtime schema를 벗어났다."""

    detail: str

    def __str__(self) -> str:
        return self.detail


def parse_fixture_event(payload: str) -> FixtureEventMarker:
    """Event topic payload를 타입이 고정된 marker로 파싱한다."""
    try:
        document = json.loads(payload)
    except json.JSONDecodeError as error:
        raise FixtureEventParseError("fixture_event_json_invalid") from error
    if not isinstance(document, dict):
        raise FixtureEventParseError("fixture_event_root_invalid")
    phase_value = document.get("phase")
    clock_value = document.get("clock_nanoseconds")
    reason_value = document.get("reason_code")
    exit_value = document.get("child_exit_code")
    if not isinstance(phase_value, str):
        raise FixtureEventParseError("fixture_event_phase_invalid")
    try:
        phase = FixturePhase(phase_value)
    except ValueError as error:
        raise FixtureEventParseError("fixture_event_phase_unknown") from error
    if isinstance(clock_value, bool) or not isinstance(clock_value, int):
        raise FixtureEventParseError("fixture_event_clock_invalid")
    if reason_value is not None and not isinstance(reason_value, str):
        raise FixtureEventParseError("fixture_event_reason_invalid")
    if exit_value is not None and (
        isinstance(exit_value, bool) or not isinstance(exit_value, int)
    ):
        raise FixtureEventParseError("fixture_event_exit_invalid")
    return FixtureEventMarker(phase, clock_value, reason_value, exit_value)


def correlate_fixture_events(
    markers: tuple[FixtureEventMarker, ...],
    streams: StreamStampObservation,
) -> tuple[FixtureEvent, ...]:
    """각 marker에 같은 stamp 또는 recovery 연속 표본의 +1 ns output을 붙인다."""
    return tuple(
        FixtureEvent(
            phase=marker.phase,
            clock_nanoseconds=marker.clock_nanoseconds,
            output_counts=OutputCounts(
                validated_cloud=_seen(marker, streams.validated_cloud),
                scan=_seen(marker, streams.scan),
                odom=_seen(marker, streams.odom),
                tf=_seen(marker, streams.tf),
            ),
            reason_code=marker.reason_code,
            child_exit_code=marker.child_exit_code,
        )
        for marker in markers
    )


def _seen(marker: FixtureEventMarker, stamps: tuple[int, ...]) -> int:
    return int(
        any(
            marker.clock_nanoseconds <= stamp <= marker.clock_nanoseconds + 1
            for stamp in stamps
        )
    )
