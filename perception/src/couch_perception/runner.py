"""CLI entry point: run YOLOv8 + YOLOP on an MCAP bag file."""

import argparse
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from couch_perception.bag_reader import read_compressed_images
from couch_perception.visualization import draw_overlay, generate_dashboard
from couch_perception.yolov8_detector import YOLOv8Detector
from couch_perception.yolop_detector import YOLOPDetector


def main() -> None:
    parser = argparse.ArgumentParser(description="Run perception on an MCAP bag")
    parser.add_argument("--bag", required=True, help="Path to .mcap bag file")
    parser.add_argument("--output", default="output", help="Output directory")
    parser.add_argument("--topic-suffix", default="image/compressed", help="Image topic suffix to read")
    parser.add_argument("--skip-yolop", action="store_true", help="Skip YOLOP (faster, detection only)")
    parser.add_argument("--device", default=None, help="Torch device (cuda, mps, cpu)")
    parser.add_argument("--conf", type=float, default=0.3, help="YOLOv8 confidence threshold")
    parser.add_argument("--max-frames", type=int, default=None, help="Process at most N frames")
    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    yolo = YOLOv8Detector(conf_threshold=args.conf, device=args.device)
    print(f"YOLOv8 loaded (device={yolo.device})")

    yolop: YOLOPDetector | None = None
    if not args.skip_yolop:
        print("Loading YOLOP...")
        yolop = YOLOPDetector(device=args.device)

    frames_iter = read_compressed_images(args.bag, topic_suffix=args.topic_suffix)
    first = next(frames_iter, None)
    if first is None:
        print(f"No images found in {args.bag} matching topic suffix '{args.topic_suffix}'")
        return

    _, frame0 = first
    h, w = frame0.shape[:2]

    video_path = output_dir / "detections.mp4"
    writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"mp4v"), 10.0, (w, h))

    class_counts: Counter[str] = Counter()
    fps_values: list[float] = []
    sample_frames: list[np.ndarray] = []
    frame_num = 0

    def process_frame(frame: np.ndarray) -> None:
        nonlocal frame_num
        t0 = time.perf_counter()

        detections = yolo.detect(frame)
        yolop_result = yolop.detect(frame) if yolop else None

        dt = time.perf_counter() - t0
        fps = 1.0 / dt if dt > 0 else 0.0
        fps_values.append(fps)

        for d in detections:
            class_counts[d.class_name] += 1

        vis = draw_overlay(frame, detections, yolop_result)
        cv2.putText(vis, f"{fps:.1f} FPS", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        writer.write(vis)

        if frame_num in (0, 50, 150):
            sample_frames.append(vis.copy())

        frame_num += 1
        if frame_num % 10 == 0:
            print(f"\rProcessed {frame_num} frames ({fps:.1f} FPS)", end="", flush=True)

    process_frame(frame0)

    for _ts, frame in frames_iter:
        if args.max_frames and frame_num >= args.max_frames:
            break
        process_frame(frame)

    writer.release()
    print(f"\nDone. {frame_num} frames → {video_path}")

    generate_dashboard(class_counts, fps_values, frame_num, sample_frames, output_dir / "dashboard.png")


if __name__ == "__main__":
    main()
