from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from .result import EkfResult


def plot_dashboard(
    result: EkfResult,
    save_path: Path | str | None = None,
    show: bool = True,
) -> None:
    t = result.times - result.times[0]
    gps_t = result.gps_times - result.times[0]

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("CouchVision EKF: IMU + GPS Fusion", fontsize=14, fontweight="bold")

    ax = axes[0, 0]
    sc = ax.scatter(
        result.positions[:, 0], result.positions[:, 1],
        c=t, cmap="viridis", s=1, label="EKF",
    )
    ax.scatter(
        result.gps_enu[:, 0], result.gps_enu[:, 1],
        c="red", s=30, marker="x", label="GPS raw", zorder=5,
    )
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("2D Path")
    ax.legend(loc="upper left")
    ax.set_aspect("equal")
    plt.colorbar(sc, ax=ax, label="Time (s)")

    ax = axes[0, 1]
    ax.plot(t, result.positions[:, 2], linewidth=0.5, label="EKF Up")
    ax.scatter(gps_t, result.gps_enu[:, 2], c="red", s=20, marker="x", label="GPS alt", zorder=5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Up (m)")
    ax.set_title("Altitude")
    ax.legend()

    ax = axes[1, 0]
    for i, label in enumerate(["East", "North", "Up"]):
        sigma = np.sqrt(np.clip(result.pos_cov[:, i], 0, None))
        ax.plot(t, sigma, linewidth=0.8, label=f"\u03c3_{label}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position std dev (m)")
    ax.set_title("Position Uncertainty (1\u03c3)")
    ax.legend()
    ax.set_yscale("log")

    ax = axes[1, 1]
    for i, label in enumerate(["vE", "vN", "vU"]):
        sigma = np.sqrt(np.clip(result.vel_cov[:, i], 0, None))
        ax.plot(t, sigma, linewidth=0.8, label=f"\u03c3_{label}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity std dev (m/s)")
    ax.set_title("Velocity Uncertainty (1\u03c3)")
    ax.legend()

    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)

    if show:
        plt.show()
    else:
        plt.close(fig)
