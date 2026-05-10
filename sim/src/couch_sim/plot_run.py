"""Plot simulator logs for manual control comparison."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt


@dataclass(slots=True)
class LoadedRun:
    """In-memory representation of a saved run."""

    label: str
    metadata: dict
    samples: list[dict]


def _load_run(path: Path) -> LoadedRun:
    payload = json.loads(path.read_text())
    metadata = payload["metadata"]
    label = f"{metadata['scenario']}:{metadata['mode']}"
    return LoadedRun(label=label, metadata=metadata, samples=payload["samples"])


def _series(run: LoadedRun, key: str) -> tuple[list[float], list[float]]:
    times = [float(sample["time"]) for sample in run.samples]
    values = [float(sample[key]) for sample in run.samples]
    return times, values


def _trajectory(run: LoadedRun) -> tuple[list[float], list[float]]:
    return (
        [float(sample["x"]) for sample in run.samples],
        [float(sample["y"]) for sample in run.samples],
    )


def plot_runs(primary: LoadedRun, compare: LoadedRun | None, output_path: Path) -> Path:
    """Create a comparison plot for one or two runs."""
    runs = [primary] if compare is None else [primary, compare]
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    ax_traj, ax_heading, ax_lat, ax_torque = axes.flat

    for run in runs:
        xs, ys = _trajectory(run)
        ax_traj.plot(xs, ys, label=run.label)
        t_heading, heading_error = _series(run, "heading_error")
        ax_heading.plot(t_heading, heading_error, label=run.label)
        t_lat, lateral_speed = _series(run, "lateral_speed")
        ax_lat.plot(t_lat, lateral_speed, label=run.label)
        t_left, left_torque = _series(run, "left_torque")
        t_right, right_torque = _series(run, "right_torque")
        ax_torque.plot(t_left, left_torque, label=f"{run.label} left")
        ax_torque.plot(t_right, right_torque, linestyle="--", label=f"{run.label} right")

    ax_traj.set_title("Trajectory")
    ax_traj.set_xlabel("x (m)")
    ax_traj.set_ylabel("y (m)")
    ax_traj.axis("equal")

    ax_heading.set_title("Heading Error")
    ax_heading.set_xlabel("time (s)")
    ax_heading.set_ylabel("rad")

    ax_lat.set_title("Lateral Speed")
    ax_lat.set_xlabel("time (s)")
    ax_lat.set_ylabel("m/s")

    ax_torque.set_title("Wheel Torque Commands")
    ax_torque.set_xlabel("time (s)")
    ax_torque.set_ylabel("N m")

    for axis in axes.flat:
        axis.grid(True, alpha=0.25)
        axis.legend()

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return output_path


def main() -> None:
    """CLI entrypoint for log plotting."""
    parser = argparse.ArgumentParser(description="Plot couch simulator logs")
    parser.add_argument("log", type=Path, help="Primary JSON run log")
    parser.add_argument("--compare", type=Path, help="Optional comparison log")
    parser.add_argument("--output", type=Path, help="Output PNG path")
    args = parser.parse_args()

    primary = _load_run(args.log)
    compare = _load_run(args.compare) if args.compare else None
    if args.output is not None:
        output = args.output
    else:
        compare_suffix = f"_{compare.metadata['mode']}" if compare else ""
        output = Path("output/plots") / f"{primary.metadata['scenario']}_{primary.metadata['mode']}{compare_suffix}.png"
    path = plot_runs(primary, compare, output)
    print(path)


if __name__ == "__main__":
    main()
