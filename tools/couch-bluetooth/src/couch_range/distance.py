from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum


class DistanceMode(StrEnum):
    MANUAL = "manual"
    TIMED_STATION = "timed_station"
    EVENT_MARKER = "event_marker"
    SIMULATION = "simulation"


@dataclass(frozen=True, slots=True)
class DistanceStation:
    meters: float
    hold_seconds: float
    source: str


def parse_distances(raw: str | None, default: list[float]) -> list[float]:
    if raw is None or raw.strip() == "":
        return default
    distances: list[float] = []
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        distances.append(float(part))
    return distances


def build_stations(
    distances: list[float], hold_seconds: float, source: str
) -> list[DistanceStation]:
    return [DistanceStation(distance, hold_seconds, source) for distance in distances]
