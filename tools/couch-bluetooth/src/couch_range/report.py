from __future__ import annotations

from pathlib import Path

from .analysis import DistanceSummary, load_summary
from .config import Thresholds
from .telemetry import read_samples
from .theme import BERKELEY_MONO_FONT, PALETTE, report_band_label


def generate_report(run_dir: Path, thresholds: Thresholds) -> Path:
    summaries = load_summary(run_dir)
    path = run_dir / "report.md"
    path.write_text(render_report(run_dir, summaries, thresholds))
    return path


def render_report(
    run_dir: Path, summaries: list[DistanceSummary], thresholds: Thresholds
) -> str:
    samples = read_samples(run_dir)
    first = samples[0] if samples else None
    conclusion = _conclusion(summaries, first.device_role if first else "")
    reliable = _max_distance(summaries, "reliable")
    marginal = _max_distance(summaries, "marginal")
    first_failure = _first_distance(summaries, "unsafe")
    lines = [
        "# Couch Bluetooth Range Report",
        "",
        f"Conclusion: {conclusion}",
        "",
        "## Reliability By Distance",
        "",
        "| Distance (m) | Band | Success | Loss | p95 latency | Reconnects | Drops | Reason |",
        "| ---: | --- | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for summary in summaries:
        p95 = (
            "n/a"
            if summary.latency_p95_ms is None
            else f"{summary.latency_p95_ms:.1f} ms"
        )
        lines.append(
            f"| {summary.distance_m:g} | {report_band_label(summary.reliability)} | {summary.success_rate * 100:.2f}% | "
            f"{summary.packet_loss_percent:.2f}% | {p95} | {summary.reconnect_events} | {summary.drop_events} | {summary.classification_reason} |"
        )
    lines.extend(
        [
            "",
            "## Decision",
            "",
            f"- Reliable distance: {_format_distance(reliable)}",
            f"- Marginal distance: {_format_distance(marginal)}",
            f"- First unsafe/failure distance: {_format_distance(first_failure)}",
            "",
            "## Tested Devices",
            "",
            f"- Laptop hostname: `{first.laptop_hostname if first else 'unknown'}`",
            f"- Receiver/Orin hostname: `{first.orin_hostname if first else 'unknown'}`",
            f"- Controller: `{first.controller_name if first else 'unknown'}`",
            f"- Adapter: `{first.adapter_name if first else 'unknown'}`",
            "",
            "## Adapter Inventory",
            "",
            "- Inventory, when collected, is saved beside the raw logs as `inventory.json` and `inventory.md`.",
            "",
            "## Test Environment",
            "",
            f"- Distance method: `{first.distance_source if first else 'unknown'}`",
            f"- Notes: `{first.notes if first else ''}`",
            "",
            "## Thresholds",
            "",
            f"- Reliable: success >= {thresholds.reliable_success_rate * 100:.1f}%, "
            f"p95 <= {thresholds.reliable_p95_latency_ms:g} ms, "
            f"loss <= {thresholds.reliable_packet_loss_percent:g}%, "
            f"reconnects <= {thresholds.reliable_max_reconnects}.",
            f"- Marginal: success >= {thresholds.marginal_success_rate * 100:.1f}%, "
            f"p95 <= {thresholds.marginal_p95_latency_ms:g} ms, "
            f"loss <= {thresholds.marginal_packet_loss_percent:g}%, "
            f"reconnects <= {thresholds.marginal_max_reconnects}.",
            "",
            "## Charts",
            "",
            f"Charts use the Midnight palette with `{BERKELEY_MONO_FONT}` preferred first, then clean Matplotlib fallbacks.",
            f"Palette anchors: background `{PALETTE.foreground}`, text `{PALETTE.background}`, blue `{PALETTE.blue}`, green `{PALETTE.green}`, yellow `{PALETTE.yellow}`, red `{PALETTE.red}`.",
            "",
        ]
    )
    chart_dir = run_dir / "charts"
    if chart_dir.exists():
        for chart in sorted(chart_dir.glob("*.png")):
            lines.append(f"- [{chart.name}](charts/{chart.name})")
    else:
        lines.append("- no charts generated")
    lines.extend(
        [
            "",
            "## Raw Logs",
            "",
            "- [raw/samples.csv](raw/samples.csv)",
            "- [raw/samples.jsonl](raw/samples.jsonl)",
            "- [raw/events.csv](raw/events.csv)",
            "- [raw/events.jsonl](raw/events.jsonl)",
            "- [analysis/distance_summary.csv](analysis/distance_summary.csv)",
            "- [analysis/summary.json](analysis/summary.json)",
            "",
            "## Caveats",
            "",
            "- Manual distance markers are the preferred ground truth for short range.",
            "- GPS/geolocation and RSSI can be inaccurate indoors, near buildings, under trees, or on hardware without GPS.",
            "- This report is only about the measured diagnostic path named in the conclusion. Do not infer real couch actuation safety from ping-only, simulated, or generic Bluetooth pairing results.",
            "- Do not test real couch motion until synthetic commands and harmless controller events are reliable.",
            "",
            "## Next Recommended Test",
            "",
            "Repeat the walk-away test in the real environment with the intended Bluetooth-backed transport and a physical emergency stop available.",
            "",
        ]
    )
    return "\n".join(lines)


def _conclusion(summaries: list[DistanceSummary], device_role: str) -> str:
    reliable = _max_distance(summaries, "reliable")
    marginal = _max_distance(summaries, "marginal")
    subject = _subject_for_role(device_role)
    if reliable is not None:
        if marginal is not None and marginal > reliable:
            return f"{subject} is reliable through {reliable:g} m and marginal through {marginal:g} m on this run."
        return f"{subject} is reliable through {reliable:g} m on this run."
    if marginal is not None:
        return f"{subject} is only marginal through {marginal:g} m on this run; treat it as unsafe for actuation."
    return f"{subject} was not reliable at any tested distance on this run."


def _subject_for_role(device_role: str) -> str:
    match device_role:
        case "laptop_ping_probe":
            return "Laptop-to-target Tailscale ping reachability"
        case "couchvision_status_probe":
            return "CouchVision no-motion heartbeat reachability"
        case "laptop_logger":
            return "Synthetic no-motion command acknowledgement"
        case _:
            return "Simulated diagnostic path"


def _max_distance(summaries: list[DistanceSummary], band: str) -> float | None:
    values = [
        summary.distance_m for summary in summaries if summary.reliability == band
    ]
    return max(values) if values else None


def _first_distance(summaries: list[DistanceSummary], band: str) -> float | None:
    values = [
        summary.distance_m for summary in summaries if summary.reliability == band
    ]
    return min(values) if values else None


def _format_distance(distance: float | None) -> str:
    return "not observed" if distance is None else f"{distance:g} m"
