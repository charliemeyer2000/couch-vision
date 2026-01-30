#!/usr/bin/env python3
"""Export YOLOv8n to TensorRT INT8 engine for Jetson deployment.

Requires CUDA — run on Jetson or a workstation with an NVIDIA GPU.
"""

import argparse
from pathlib import Path

from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser(description="Export YOLOv8 to TensorRT")
    parser.add_argument("--model", default="yolov8n.pt", help="Source PyTorch weights")
    parser.add_argument("--imgsz", type=int, default=640, help="Input image size")
    parser.add_argument("--no-int8", action="store_true", help="Disable INT8 quantization (use FP16)")
    parser.add_argument("--output-dir", default="weights", help="Output directory")
    args = parser.parse_args()

    model = YOLO(args.model)
    out = model.export(format="engine", int8=not args.no_int8, imgsz=args.imgsz)
    print(f"Exported TensorRT engine: {out}")

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Engine saved. Copy to {output_dir}/ if needed.")


if __name__ == "__main__":
    main()
