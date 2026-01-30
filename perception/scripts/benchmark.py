#!/usr/bin/env python3
"""Benchmark YOLOv8 inference: PyTorch vs TensorRT engine."""

import argparse
import time
from pathlib import Path

import numpy as np
from ultralytics import YOLO


def benchmark(model_path: str, n_frames: int = 100, imgsz: int = 640) -> None:
    print(f"Loading {model_path}...")
    model = YOLO(model_path)

    dummy = np.random.randint(0, 255, (imgsz, imgsz, 3), dtype=np.uint8)

    for _ in range(5):
        model(dummy, verbose=False)

    times: list[float] = []
    for _ in range(n_frames):
        t0 = time.perf_counter()
        model(dummy, verbose=False)
        times.append(time.perf_counter() - t0)

    times_ms = [t * 1000 for t in times]
    avg = sum(times_ms) / len(times_ms)
    fps = 1000.0 / avg
    print(f"  {Path(model_path).name}: avg={avg:.1f}ms  min={min(times_ms):.1f}ms  max={max(times_ms):.1f}ms  FPS={fps:.1f}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Benchmark YOLOv8")
    parser.add_argument("--pytorch", default="yolov8n.pt", help="PyTorch weights")
    parser.add_argument("--engine", default=None, help="TensorRT engine path")
    parser.add_argument("--frames", type=int, default=100, help="Number of frames")
    parser.add_argument("--imgsz", type=int, default=640, help="Image size")
    args = parser.parse_args()

    benchmark(args.pytorch, args.frames, args.imgsz)
    if args.engine and Path(args.engine).exists():
        benchmark(args.engine, args.frames, args.imgsz)


if __name__ == "__main__":
    main()
