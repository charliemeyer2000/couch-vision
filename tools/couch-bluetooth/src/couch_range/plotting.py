from __future__ import annotations

from pathlib import Path
from typing import Any, cast

from .analysis import DistanceSummary
from .telemetry import read_samples
from .theme import chart_color, configure_matplotlib, reliability_color


def create_plots(run_dir: Path, summaries: list[DistanceSummary]) -> list[Path]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    chart_dir = run_dir / "charts"
    chart_dir.mkdir(parents=True, exist_ok=True)
    configure_matplotlib(plt)
    paths = [
        _plot_line(
            chart_dir,
            summaries,
            "success_rate_vs_distance.png",
            "Command success rate vs distance",
            "Success rate (%)",
            lambda s: s.success_rate * 100,
        ),
        _plot_latency(chart_dir, summaries),
        _plot_line(
            chart_dir,
            summaries,
            "packet_loss_vs_distance.png",
            "Packet loss vs distance",
            "Packet loss (%)",
            lambda s: s.packet_loss_percent,
        ),
        _plot_response_density(run_dir, chart_dir),
        _plot_drops_over_time(run_dir, chart_dir),
        _plot_reliability(chart_dir, summaries),
    ]
    if any(summary.rssi_median_dbm is not None for summary in summaries):
        paths.append(
            _plot_line(
                chart_dir,
                summaries,
                "rssi_vs_distance.png",
                "Bluetooth RSSI vs distance",
                "Median RSSI (dBm)",
                lambda s: s.rssi_median_dbm,
            )
        )
    if any(summary.ping_rtt_median_ms is not None for summary in summaries):
        paths.append(
            _plot_line(
                chart_dir,
                summaries,
                "ping_rtt_vs_distance.png",
                "Ping RTT vs distance",
                "Median RTT (ms)",
                lambda s: s.ping_rtt_median_ms,
            )
        )
    return paths


def _distances(summaries: list[DistanceSummary]) -> list[float]:
    return [summary.distance_m for summary in summaries]


def _plot_line(
    chart_dir: Path,
    summaries: list[DistanceSummary],
    filename: str,
    title: str,
    ylabel: str,
    getter,
) -> Path:
    import matplotlib.pyplot as plt

    x = _distances(summaries)
    y = [getter(summary) for summary in summaries]
    path = chart_dir / filename
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(x, y, marker="o", color=chart_color("blue"), linewidth=2)
    ax.set_title(title)
    ax.set_xlabel("Declared distance (m)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def _plot_latency(chart_dir: Path, summaries: list[DistanceSummary]) -> Path:
    import matplotlib.pyplot as plt

    x = _distances(summaries)
    path = chart_dir / "latency_percentiles_vs_distance.png"
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for label, attr, color in [
        ("p50", "latency_p50_ms", chart_color("green")),
        ("p95", "latency_p95_ms", chart_color("yellow")),
        ("p99", "latency_p99_ms", chart_color("red")),
    ]:
        ax.plot(
            x,
            [getattr(summary, attr) for summary in summaries],
            marker="o",
            label=label,
            color=color,
            linewidth=2,
        )
    ax.set_title("Command latency percentiles vs distance")
    ax.set_xlabel("Declared distance (m)")
    ax.set_ylabel("Latency (ms)")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def _plot_response_density(run_dir: Path, chart_dir: Path) -> Path:
    import matplotlib.pyplot as plt

    samples = read_samples(run_dir)
    by_distance: dict[float, list[float]] = {}
    timeout_rates: dict[float, float] = {}
    for sample in samples:
        by_distance.setdefault(sample.declared_distance_m, [])
        if sample.command_latency_ms is not None:
            by_distance[sample.declared_distance_m].append(sample.command_latency_ms)
    for distance in by_distance:
        grouped = [
            sample for sample in samples if sample.declared_distance_m == distance
        ]
        timeout_count = sum(
            1 for sample in grouped if sample.command_latency_ms is None
        )
        timeout_rates[distance] = (timeout_count / len(grouped)) * 100 if grouped else 0

    distances = sorted(by_distance)
    path = chart_dir / "response_density_by_distance.png"
    fig, ax = plt.subplots(figsize=(8.5, 4.8))

    latency_groups = [by_distance[distance] for distance in distances]
    has_responses = any(latency_groups)
    if has_responses:
        positions = list(range(1, len(distances) + 1))
        non_empty_positions = [
            position
            for position, values in zip(positions, latency_groups, strict=True)
            if values
        ]
        non_empty_values = [values for values in latency_groups if values]
        violins = ax.violinplot(
            non_empty_values,
            positions=non_empty_positions,
            showmeans=True,
            showmedians=True,
            widths=0.8,
        )
        for body in cast(list[Any], violins["bodies"]):
            body.set_facecolor(chart_color("cyan"))
            body.set_edgecolor(chart_color("blue"))
            body.set_alpha(0.45)
        for key in ["cmeans", "cmedians", "cbars", "cmins", "cmaxes"]:
            violins[key].set_color(chart_color("foreground"))
        for position, distance, values in zip(
            positions, distances, latency_groups, strict=True
        ):
            if values:
                ax.scatter(
                    [position] * len(values),
                    values,
                    s=12,
                    color=chart_color("blue"),
                    alpha=0.45,
                    zorder=3,
                )
            if timeout_rates[distance] > 0:
                top = max([*values, 1.0]) * 1.08 if values else 1.0
                ax.text(
                    position,
                    top,
                    f"{timeout_rates[distance]:.0f}% timeout",
                    ha="center",
                    va="bottom",
                    fontsize=8,
                    color=chart_color("red"),
                )
        ax.set_xticks(positions, [f"{distance:g}" for distance in distances])
        ax.set_ylabel("Response latency (ms)")
        ax.set_title("Response latency density by distance")
    else:
        ax.bar(
            [f"{distance:g}" for distance in distances],
            [timeout_rates[distance] for distance in distances],
            color=chart_color("red"),
        )
        ax.set_ylim(0, 100)
        ax.set_ylabel("Timeout rate (%)")
        ax.set_title("No responses received; timeout density by distance")

    ax.set_xlabel("Declared distance (m)")
    ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def _plot_drops_over_time(run_dir: Path, chart_dir: Path) -> Path:
    import matplotlib.pyplot as plt

    samples = read_samples(run_dir)
    x = [sample.elapsed_s for sample in samples]
    y = [sample.declared_distance_m for sample in samples]
    timeout_x = [sample.elapsed_s for sample in samples if sample.error]
    timeout_y = [sample.declared_distance_m for sample in samples if sample.error]
    path = chart_dir / "drops_over_time.png"
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.step(
        x,
        y,
        where="post",
        label="declared distance",
        color=chart_color("blue"),
        linewidth=2,
    )
    if timeout_x:
        ax.scatter(
            timeout_x,
            timeout_y,
            label="timeout/error",
            color=chart_color("red"),
            marker="x",
            s=45,
        )
        title = "Reconnect/drop/timeout events over time"
    else:
        title = "Distance timeline over time; no reconnect/drop events observed"
    ax.set_title(title)
    ax.set_xlabel("Elapsed time (s)")
    ax.set_ylabel("Declared distance (m)")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def _plot_reliability(chart_dir: Path, summaries: list[DistanceSummary]) -> Path:
    import matplotlib.pyplot as plt

    values = {
        "unsafe": 0,
        "marginal": 1,
        "reliable": 2,
        "insufficient_data": -1,
        "no_data": -1,
    }
    x = _distances(summaries)
    y = [values[summary.reliability] for summary in summaries]
    path = chart_dir / "reliability_by_distance.png"
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.bar(
        x,
        y,
        color=[reliability_color(summary.reliability) for summary in summaries],
        width=2.5,
    )
    ax.set_title("Reliability classification by distance")
    ax.set_xlabel("Declared distance (m)")
    ax.set_yticks([-1, 0, 1, 2], ["insufficient", "unsafe", "marginal", "reliable"])
    ax.set_ylim(-1.2, 2.2)
    ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path
