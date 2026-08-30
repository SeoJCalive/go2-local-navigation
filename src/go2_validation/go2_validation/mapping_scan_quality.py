
"""LaserScan payload를 보관하지 않고 유효 beam 분포만 누적한다."""
from collections.abc import Sequence
from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True, slots=True)
class MappingScanQualityObservation:
    """한 mapping run의 scan 수와 유효 beam 분위수다."""

    sample_count: int
    minimum_valid_beams: int
    percentile_10_valid_beams: int
    median_valid_beams: int
    percentile_90_valid_beams: int
    maximum_valid_beams: int


def empty_mapping_scan_quality_observation() -> MappingScanQualityObservation:
    """아직 scan을 받지 않은 기본 관찰값을 반환한다."""
    return MappingScanQualityObservation(0, 0, 0, 0, 0, 0)


class MappingScanQualityAccumulator:
    """유효 beam 개수 histogram으로 bounded 분위수를 계산한다."""

    def __init__(self) -> None:
        self._sample_count = 0
        self._histogram: list[int] = [0]

    def observe(
        self,
        ranges: Sequence[float],
        range_min: float,
        range_max: float,
    ) -> None:
        """한 LaserScan에서 finite하고 허용 range 안인 beam 수를 반영한다."""
        valid_beams = sum(
            isfinite(value) and range_min <= value <= range_max for value in ranges
        )
        if valid_beams >= len(self._histogram):
            self._histogram.extend(
                0 for _ in range(valid_beams - len(self._histogram) + 1)
            )
        self._histogram[valid_beams] += 1
        self._sample_count += 1

    def observation(self) -> MappingScanQualityObservation:
        """현재 histogram을 고정된 10·50·90 percentile 관찰값으로 투영한다."""
        if self._sample_count == 0:
            return empty_mapping_scan_quality_observation()
        minimum = next(
            index for index, count in enumerate(self._histogram) if count > 0
        )
        maximum = next(
            index
            for index in range(len(self._histogram) - 1, -1, -1)
            if self._histogram[index] > 0
        )
        return MappingScanQualityObservation(
            sample_count=self._sample_count,
            minimum_valid_beams=minimum,
            percentile_10_valid_beams=self._percentile(0.10),
            median_valid_beams=self._percentile(0.50),
            percentile_90_valid_beams=self._percentile(0.90),
            maximum_valid_beams=maximum,
        )

    def _percentile(self, fraction: float) -> int:
        target_rank = max(1, int((self._sample_count * fraction) + 0.999999999))
        cumulative = 0
        for valid_beams, count in enumerate(self._histogram):
            cumulative += count
            if cumulative >= target_rank:
                return valid_beams
        return len(self._histogram) - 1
