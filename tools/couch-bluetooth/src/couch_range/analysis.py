from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
import csv
import json
import statistics

from .config import Thresholds
from .telemetry import ReliabilityBand, Sample, analysis_dir, read_samples


@dataclass(frozen=True, slots=True)
class DistanceSummary:
    distance_m: float
    samples: int
    sent: int
    acked: int
    timeouts: int
    success_rate: float
    packet_loss_percent: float
    latency_p50_ms: float | None
    latency_p95_ms: float | None
    latency_p99_ms: float | None
    ping_rtt_median_ms: float | None
    rssi_median_dbm: float | None
    reconnect_events: int
    drop_events: int
    reliability: str
    classification_reason: str


def summarize_run(run_dir: Path, thresholds: Thresholds) -> list[DistanceSummary]:
    return summarize_samples(read_samples(run_dir), thresholds)


def summarize_samples(
    samples: list[Sample], thresholds: Thresholds
) -> list[DistanceSummary]:
    by_distance: dict[float, list[Sample]] = {}
    for sample in samples:
        by_distance.setdefault(sample.declared_distance_m, []).append(sample)
    return [
        summarize_distance(distance, grouped, thresholds)
        for distance, grouped in sorted(by_distance.items())
    ]


def summarize_distance(
    distance: float, samples: list[Sample], thresholds: Thresholds
) -> DistanceSummary:
    if not samples:
        return DistanceSummary(
            distance,
            0,
            0,
            0,
            0,
            0,
            100,
            None,
            None,
            None,
            None,
            None,
            0,
            0,
            ReliabilityBand.NO_DATA.value,
            "no samples",
        )
    sent = max(sample.command_sent_count for sample in samples)
    acked = max(sample.command_ack_count for sample in samples)
    timeouts = max(sample.command_timeout_count for sample in samples)
    success_rate = acked / sent if sent else 0.0
    loss = (timeouts / sent) * 100 if sent else 100.0
    latencies = [
        sample.command_latency_ms
        for sample in samples
        if sample.command_latency_ms is not None
    ]
    ping = [sample.ping_rtt_ms for sample in samples if sample.ping_rtt_ms is not None]
    rssi = [sample.rssi_dbm for sample in samples if sample.rssi_dbm is not None]
    reconnects = max(sample.reconnect_events for sample in samples)
    drops = max(sample.drop_events for sample in samples)
    p95 = percentile(latencies, 95)
    band, reason = classify_with_reason(
        success_rate, p95, loss, reconnects, drops, len(samples), sent, thresholds
    )
    return DistanceSummary(
        distance_m=distance,
        samples=len(samples),
        sent=sent,
        acked=acked,
        timeouts=timeouts,
        success_rate=success_rate,
        packet_loss_percent=loss,
        latency_p50_ms=percentile(latencies, 50),
        latency_p95_ms=p95,
        latency_p99_ms=percentile(latencies, 99),
        ping_rtt_median_ms=statistics.median(ping) if ping else None,
        rssi_median_dbm=statistics.median(rssi) if rssi else None,
        reconnect_events=reconnects,
        drop_events=drops,
        reliability=band.value,
        classification_reason=reason,
    )


def classify(
    success_rate: float,
    p95_latency_ms: float | None,
    packet_loss_percent: float,
    reconnect_events: int,
    drop_events: int,
    thresholds: Thresholds,
) -> ReliabilityBand:
    return classify_with_reason(
        success_rate,
        p95_latency_ms,
        packet_loss_percent,
        reconnect_events,
        drop_events,
        5,
        1,
        thresholds,
    )[0]


def classify_with_reason(
    success_rate: float,
    p95_latency_ms: float | None,
    packet_loss_percent: float,
    reconnect_events: int,
    drop_events: int,
    sample_count: int,
    sent_count: int,
    thresholds: Thresholds,
) -> tuple[ReliabilityBand, str]:
    if sent_count == 0:
        return ReliabilityBand.NO_DATA, "no attempted commands"
    if sample_count < 5:
        return ReliabilityBand.INSUFFICIENT_DATA, f"only {sample_count} samples"
    p95 = p95_latency_ms if p95_latency_ms is not None else float("inf")
    if (
        success_rate >= thresholds.reliable_success_rate
        and p95 <= thresholds.reliable_p95_latency_ms
        and packet_loss_percent <= thresholds.reliable_packet_loss_percent
        and reconnect_events <= thresholds.reliable_max_reconnects
        and drop_events == 0
    ):
        return (
            ReliabilityBand.RELIABLE,
            "met reliable success, latency, loss, and drop thresholds",
        )
    if (
        success_rate >= thresholds.marginal_success_rate
        and p95 <= thresholds.marginal_p95_latency_ms
        and packet_loss_percent <= thresholds.marginal_packet_loss_percent
        and reconnect_events <= thresholds.marginal_max_reconnects
        and drop_events == 0
    ):
        return (
            ReliabilityBand.MARGINAL,
            "met marginal thresholds but failed at least one reliable threshold",
        )
    reasons: list[str] = []
    if success_rate < thresholds.marginal_success_rate:
        reasons.append(f"success {success_rate * 100:.2f}% below marginal")
    if p95 > thresholds.marginal_p95_latency_ms:
        reasons.append(f"p95 latency {p95:.1f} ms above marginal")
    if packet_loss_percent > thresholds.marginal_packet_loss_percent:
        reasons.append(f"loss {packet_loss_percent:.2f}% above marginal")
    if reconnect_events > thresholds.marginal_max_reconnects:
        reasons.append(f"{reconnect_events} reconnect events")
    if drop_events > 0:
        reasons.append(f"{drop_events} drop events")
    return ReliabilityBand.UNSAFE, "; ".join(reasons) or "failed marginal thresholds"


def percentile(values: list[float], percent: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * (percent / 100)
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    weight = rank - lower
    return ordered[lower] * (1 - weight) + ordered[upper] * weight


def write_summary(run_dir: Path, summaries: list[DistanceSummary]) -> Path:
    out_dir = analysis_dir(run_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "summary.json"
    path.write_text(
        json.dumps([asdict(summary) for summary in summaries], indent=2, sort_keys=True)
        + "\n"
    )
    legacy_path = run_dir / "summary.json"
    legacy_path.write_text(path.read_text())
    write_distance_summary_csv(out_dir / "distance_summary.csv", summaries)
    (out_dir / "reliability_by_distance.json").write_text(
        json.dumps(
            {
                str(summary.distance_m): {
                    "classification": summary.reliability,
                    "reason": summary.classification_reason,
                }
                for summary in summaries
            },
            indent=2,
            sort_keys=True,
        )
        + "\n"
    )
    return path


def load_summary(run_dir: Path) -> list[DistanceSummary]:
    path = analysis_dir(run_dir) / "summary.json"
    if not path.exists():
        path = run_dir / "summary.json"
    data = json.loads(path.read_text())
    return [DistanceSummary(**item) for item in data]


def write_distance_summary_csv(path: Path, summaries: list[DistanceSummary]) -> None:
    fieldnames = [
        "distance_m",
        "sample_count",
        "command_sent_count",
        "command_ack_count",
        "command_timeout_count",
        "command_success_rate",
        "packet_loss_pct",
        "latency_p50_ms",
        "latency_p95_ms",
        "latency_p99_ms",
        "ping_rtt_p50_ms",
        "rssi_mean_dbm",
        "drop_count",
        "reconnect_count",
        "classification",
        "classification_reason",
    ]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for summary in summaries:
            writer.writerow(
                {
                    "distance_m": summary.distance_m,
                    "sample_count": summary.samples,
                    "command_sent_count": summary.sent,
                    "command_ack_count": summary.acked,
                    "command_timeout_count": summary.timeouts,
                    "command_success_rate": summary.success_rate,
                    "packet_loss_pct": summary.packet_loss_percent,
                    "latency_p50_ms": summary.latency_p50_ms,
                    "latency_p95_ms": summary.latency_p95_ms,
                    "latency_p99_ms": summary.latency_p99_ms,
                    "ping_rtt_p50_ms": summary.ping_rtt_median_ms,
                    "rssi_mean_dbm": summary.rssi_median_dbm,
                    "drop_count": summary.drop_events,
                    "reconnect_count": summary.reconnect_events,
                    "classification": summary.reliability,
                    "classification_reason": summary.classification_reason,
                }
            )
