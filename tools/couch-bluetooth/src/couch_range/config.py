from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path
import tomllib


@dataclass(frozen=True)
class Thresholds:
    reliable_success_rate: float = 0.99
    reliable_p95_latency_ms: float = 100.0
    reliable_packet_loss_percent: float = 1.0
    reliable_max_reconnects: int = 0
    marginal_success_rate: float = 0.95
    marginal_p95_latency_ms: float = 250.0
    marginal_packet_loss_percent: float = 5.0
    marginal_max_reconnects: int = 0


@dataclass(frozen=True)
class RunConfig:
    sample_rate_hz: float = 10.0
    timeout_ms: int = 500
    hold_seconds: float = 30.0
    distances_m: list[float] = field(default_factory=lambda: [0, 5, 10, 20, 30, 50])
    distance_source: str = "manual_marker"
    transport: str = "tcp"
    notes: str = "Synthetic command/ack range test. No couch motion."


@dataclass(frozen=True)
class AppConfig:
    run: RunConfig = field(default_factory=RunConfig)
    thresholds: Thresholds = field(default_factory=Thresholds)

    def with_sample_rate(self, sample_rate_hz: float) -> AppConfig:
        return replace(self, run=replace(self.run, sample_rate_hz=sample_rate_hz))


def _coerce_float_list(value: object, default: list[float]) -> list[float]:
    if not isinstance(value, list):
        return default
    result: list[float] = []
    for item in value:
        if not isinstance(item, str | int | float):
            continue
        try:
            result.append(float(item))
        except (TypeError, ValueError):
            continue
    return result or default


def load_config(path: Path | None) -> AppConfig:
    if path is None:
        return AppConfig()

    data = tomllib.loads(path.read_text())
    run_data = data.get("run", {})
    threshold_data = data.get("thresholds", {})
    if not isinstance(run_data, dict):
        run_data = {}
    if not isinstance(threshold_data, dict):
        threshold_data = {}

    run_defaults = RunConfig()
    threshold_defaults = Thresholds()
    run = RunConfig(
        sample_rate_hz=float(
            run_data.get("sample_rate_hz", run_defaults.sample_rate_hz)
        ),
        timeout_ms=int(run_data.get("timeout_ms", run_defaults.timeout_ms)),
        hold_seconds=float(run_data.get("hold_seconds", run_defaults.hold_seconds)),
        distances_m=_coerce_float_list(
            run_data.get("distances_m"), run_defaults.distances_m
        ),
        distance_source=str(
            run_data.get("distance_source", run_defaults.distance_source)
        ),
        transport=str(run_data.get("transport", run_defaults.transport)),
        notes=str(run_data.get("notes", run_defaults.notes)),
    )
    thresholds = Thresholds(
        reliable_success_rate=float(
            threshold_data.get(
                "reliable_success_rate", threshold_defaults.reliable_success_rate
            )
        ),
        reliable_p95_latency_ms=float(
            threshold_data.get(
                "reliable_p95_latency_ms", threshold_defaults.reliable_p95_latency_ms
            )
        ),
        reliable_packet_loss_percent=float(
            threshold_data.get(
                "reliable_packet_loss_percent",
                threshold_defaults.reliable_packet_loss_percent,
            )
        ),
        reliable_max_reconnects=int(
            threshold_data.get(
                "reliable_max_reconnects", threshold_defaults.reliable_max_reconnects
            )
        ),
        marginal_success_rate=float(
            threshold_data.get(
                "marginal_success_rate", threshold_defaults.marginal_success_rate
            )
        ),
        marginal_p95_latency_ms=float(
            threshold_data.get(
                "marginal_p95_latency_ms", threshold_defaults.marginal_p95_latency_ms
            )
        ),
        marginal_packet_loss_percent=float(
            threshold_data.get(
                "marginal_packet_loss_percent",
                threshold_defaults.marginal_packet_loss_percent,
            )
        ),
        marginal_max_reconnects=int(
            threshold_data.get(
                "marginal_max_reconnects", threshold_defaults.marginal_max_reconnects
            )
        ),
    )
    return AppConfig(run=run, thresholds=thresholds)
