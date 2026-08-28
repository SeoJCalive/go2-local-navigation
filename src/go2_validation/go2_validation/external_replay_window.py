
"""외부 replay의 120초 dynamic 구간 후보와 결정 규칙을 계산한다."""
from bisect import bisect_left
from dataclasses import dataclass
from math import hypot
from typing import Final

from go2_validation.external_replay_contract import ContractConflict


NANOSECONDS: Final = 1_000_000_000
SHORT_DURATION_NS: Final = 120 * NANOSECONDS


@dataclass(frozen=True, slots=True)
class MessageRecord:
    channel: str
    log_time_ns: int
    sequence: int
    planar_xy: tuple[float, float] | None = None


@dataclass(frozen=True, slots=True)
class ShortWindow:
    start_ns: int
    end_ns: int
    cloud_count: int
    odometry_count: int
    path_score: float


def choose_short_window(
    odometry: tuple[MessageRecord, ...],
    cloud_log_times_ns: tuple[int, ...],
    interval_start_ns: int,
    interval_end_ns: int,
    minimum_cloud_count: int,
    minimum_odometry_count: int,
) -> ShortWindow:
    """최대 path score를 선택하고 동률이면 가장 이른 시작을 택한다."""
    candidates = short_window_candidates(
        odometry,
        cloud_log_times_ns,
        interval_start_ns,
        interval_end_ns,
        minimum_cloud_count,
        minimum_odometry_count,
    )
    if not candidates:
        raise ContractConflict("short_window_candidate_absent")
    return min(candidates, key=lambda item: (-item.path_score, item.start_ns))


def short_window_candidates(
    odometry: tuple[MessageRecord, ...],
    cloud_log_times_ns: tuple[int, ...],
    interval_start_ns: int,
    interval_end_ns: int,
    minimum_cloud_count: int,
    minimum_odometry_count: int,
) -> tuple[ShortWindow, ...]:
    """1초 간격 후보를 선형 score index로 계산한다."""
    first_start = ((interval_start_ns + NANOSECONDS - 1) // NANOSECONDS) * NANOSECONDS
    cloud_times = tuple(sorted(cloud_log_times_ns))
    ordered_odometry = tuple(
        sorted(odometry, key=lambda item: (item.log_time_ns, item.sequence))
    )
    odometry_times = tuple(record.log_time_ns for record in ordered_odometry)
    first_by_second: dict[int, MessageRecord] = {}
    for record in ordered_odometry:
        second = (record.log_time_ns // NANOSECONDS) * NANOSECONDS
        if second not in first_by_second:
            first_by_second[second] = record
    path_samples = tuple(
        (second, record.planar_xy)
        for second, record in sorted(first_by_second.items())
        if record.planar_xy is not None
    )
    path_seconds = tuple(sample[0] for sample in path_samples)
    edge_prefix = [0.0]
    for left, right in zip(path_samples, path_samples[1:]):
        left_xy = left[1]
        right_xy = right[1]
        if left_xy is None or right_xy is None:
            raise ContractConflict("short_window_path_sample_invalid")
        edge_prefix.append(
            edge_prefix[-1]
            + hypot(right_xy[0] - left_xy[0], right_xy[1] - left_xy[1])
        )
    candidates: list[ShortWindow] = []
    for start in range(
        first_start,
        interval_end_ns - SHORT_DURATION_NS + 1,
        NANOSECONDS,
    ):
        end = start + SHORT_DURATION_NS
        cloud_count = bisect_left(cloud_times, end) - bisect_left(cloud_times, start)
        odometry_count = bisect_left(odometry_times, end) - bisect_left(
            odometry_times,
            start,
        )
        if cloud_count < minimum_cloud_count or odometry_count < minimum_odometry_count:
            continue
        left_index = bisect_left(path_seconds, start)
        right_index = bisect_left(path_seconds, end)
        score = (
            edge_prefix[right_index - 1] - edge_prefix[left_index]
            if right_index - left_index > 1
            else 0.0
        )
        candidates.append(
            ShortWindow(
                start,
                end,
                cloud_count,
                odometry_count,
                round(score, 9),
            )
        )
    return tuple(candidates)
