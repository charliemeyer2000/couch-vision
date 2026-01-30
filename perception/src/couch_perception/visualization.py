"""Visualization: overlay detections on frames and generate dashboard."""

from collections import Counter
from pathlib import Path

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from couch_perception.yolov8_detector import Detection
from couch_perception.yolop_detector import YOLOPResult

_YELLOW = (0, 255, 255)
_GREEN = (0, 255, 0)
_RED = (0, 0, 255)


def draw_overlay(frame: np.ndarray, detections: list[Detection], yolop: YOLOPResult | None) -> np.ndarray:
    vis = frame.copy()

    if yolop is not None:
        color_area = np.zeros_like(vis)
        color_area[yolop.drivable_mask == 1] = _GREEN
        color_area[yolop.lane_mask == 1] = _RED
        vis = cv2.addWeighted(vis, 1, color_area, 0.5, 0)

    for det in detections:
        cv2.rectangle(vis, (det.x1, det.y1), (det.x2, det.y2), _YELLOW, 2)
        label = f"{det.class_name} {det.confidence:.2f}"
        cv2.putText(vis, label, (det.x1, det.y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, _YELLOW, 1, cv2.LINE_AA)

    return vis


def generate_dashboard(
    class_counts: Counter[str],
    fps_values: list[float],
    total_frames: int,
    sample_frames: list[np.ndarray],
    output_path: str | Path,
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle(f"CouchVision Perception Dashboard — {total_frames} frames", fontsize=16)

    ax = axes[0, 0]
    if class_counts:
        classes, counts = zip(*class_counts.most_common())
        ax.barh(classes, counts, color="steelblue")
        ax.set_xlabel("Count")
    else:
        ax.text(0.5, 0.5, "No detections", ha="center", va="center", transform=ax.transAxes)
    ax.set_title("Detection Class Distribution")

    ax = axes[0, 1]
    if fps_values:
        ax.plot(fps_values, linewidth=0.8)
        ax.set_xlabel("Frame")
        ax.set_ylabel("FPS")
        avg_fps = sum(fps_values) / len(fps_values)
        ax.axhline(avg_fps, color="r", linestyle="--", label=f"avg {avg_fps:.1f}")
        ax.legend()
    ax.set_title("Processing FPS")

    for i, ax in enumerate([axes[1, 0], axes[1, 1]]):
        if i < len(sample_frames):
            ax.imshow(cv2.cvtColor(sample_frames[i], cv2.COLOR_BGR2RGB))
            ax.set_title(f"Sample Frame {i + 1}")
        ax.axis("off")

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Dashboard saved to {output_path}")
