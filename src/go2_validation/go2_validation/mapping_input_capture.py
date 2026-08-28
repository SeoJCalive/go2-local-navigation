
"""Mapping ingress observer 표본을 순수 acceptance 관찰값으로 투영한다."""
from dataclasses import dataclass
from math import isfinite, isinf

from go2_validation.mapping_input_acceptance_runner import (
    MappingInputObservation,
    MappingInputVariant,
)


NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True, slots=True)
class MappingStreamCapture:
    """한 replay variant에서 수집한 scan·odom·graph 최소 표본이다."""

    scan_type: str
    scan_frames: tuple[str, ...]
    scan_stamps_ns: tuple[int, ...]
    scan_ranges: tuple[float, ...]
    odom_stamps_ns: tuple[int, ...]
    clock_publisher_max: int
    global_tf_owner_count: int
    command_publisher_max: int


def build_observation(
    variant: MappingInputVariant,
    capture: MappingStreamCapture,
    source_checksum: str,
    *,
    domain_id: int,
    loopback_only: bool,
    minimum_rate_hz: float,
) -> MappingInputObservation:
    """Raw capture에서 frame·시간·rate·overlap verdict를 계산한다."""
    frame_id = _single_frame(capture.scan_frames)
    stamps = capture.scan_stamps_ns
    monotonic = all(previous <= current for previous, current in zip(stamps, stamps[1:]))
    return MappingInputObservation(
        variant=variant,
        scan_message_type=capture.scan_type,
        scan_frame_id=frame_id,
        scan_stamps_monotonic=monotonic,
        scan_ranges_finite_or_infinite=all(
            isfinite(value) or isinf(value) for value in capture.scan_ranges
        ),
        scan_minimum_rate_met=_rate_hz(stamps) >= minimum_rate_hz,
        clock_publishers=capture.clock_publisher_max,
        global_map_to_odom_owners=capture.global_tf_owner_count,
        command_publishers=capture.command_publisher_max,
        domain_id=domain_id,
        loopback_only=loopback_only,
        odom_overlaps_scan_clock=_overlaps(stamps, capture.odom_stamps_ns),
        source_checksum=source_checksum,
    )


def _single_frame(frames: tuple[str, ...]) -> str:
    unique = tuple(sorted(set(frames)))
    return unique[0] if len(unique) == 1 else "|".join(unique)


def _rate_hz(stamps: tuple[int, ...]) -> float:
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return 0.0
    return (len(stamps) - 1) * NANOSECONDS_PER_SECOND / (stamps[-1] - stamps[0])


def _overlaps(first: tuple[int, ...], second: tuple[int, ...]) -> bool:
    if not first or not second:
        return False
    return max(first[0], second[0]) <= min(first[-1], second[-1])
