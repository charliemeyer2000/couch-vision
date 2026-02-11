"""Jetson hardware benchmarks for the perception stack.

Run with: make benchmark
Or:       cd perception && uv run pytest tests/test_benchmark.py -v
"""

from __future__ import annotations

import subprocess
import sys
import textwrap
import time
from pathlib import Path

import numpy as np
import pytest
import torch

from couch_perception.bag_reader import SyncedFrame
from couch_perception.camera_model import make_camera_model
from couch_perception.costmap import build_costmap
from couch_perception.projection import apply_imu_rotation, build_depth_camera_model, extract_mask_pixels


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _gpu_mem_mb() -> float:
    if torch.cuda.is_available():
        return torch.cuda.memory_allocated() / 1024**2
    return 0.0


# ---------------------------------------------------------------------------
# YOLOv8
# ---------------------------------------------------------------------------

class TestYOLOv8:
    @pytest.fixture(scope="class")
    def yolo_pt(self, weights_dir, device):
        from couch_perception.yolov8_detector import YOLOv8Detector

        pt = weights_dir / "yolov8n.pt"
        if not pt.exists():
            pytest.skip(f"Weights not found: {pt}")
        return YOLOv8Detector(model_path=str(pt), device=device)

    @pytest.fixture(scope="class")
    def yolo_trt(self, weights_dir, device):
        from couch_perception.yolov8_detector import YOLOv8Detector

        engine = weights_dir / "yolov8n.engine"
        if not engine.exists():
            pytest.skip(f"TensorRT engine not found: {engine}")
        return YOLOv8Detector(model_path=str(engine), device=device)

    def test_yolov8_inference_pt(self, benchmark, yolo_pt, dummy_image):
        # warmup
        for _ in range(3):
            yolo_pt.detect(dummy_image)

        mem_before = _gpu_mem_mb()
        result = benchmark(yolo_pt.detect, dummy_image)
        mem_after = _gpu_mem_mb()

        assert isinstance(result, list)
        print(f"\n  GPU mem: {mem_before:.0f} → {mem_after:.0f} MB")

    @pytest.mark.tensorrt
    def test_yolov8_inference_trt(self, benchmark, yolo_trt, dummy_image):
        for _ in range(3):
            yolo_trt.detect(dummy_image)

        mem_before = _gpu_mem_mb()
        result = benchmark(yolo_trt.detect, dummy_image)
        mem_after = _gpu_mem_mb()

        assert isinstance(result, list)
        print(f"\n  GPU mem: {mem_before:.0f} → {mem_after:.0f} MB")


# ---------------------------------------------------------------------------
# YOLOP
# ---------------------------------------------------------------------------

class TestYOLOP:
    @pytest.fixture(scope="class")
    def yolop(self, device):
        from couch_perception.yolop_detector import YOLOPDetector

        return YOLOPDetector(device=device)

    def test_yolop_inference(self, benchmark, yolop, dummy_image):
        for _ in range(3):
            yolop.detect(dummy_image)

        mem_before = _gpu_mem_mb()
        result = benchmark(yolop.detect, dummy_image)
        mem_after = _gpu_mem_mb()

        assert result.drivable_mask.shape[:2] == dummy_image.shape[:2] or result.drivable_mask.ndim == 2
        print(f"\n  GPU mem: {mem_before:.0f} → {mem_after:.0f} MB")


# ---------------------------------------------------------------------------
# Projection
# ---------------------------------------------------------------------------

class TestProjection:
    def test_depth_projection(self, benchmark, dummy_depth, dummy_intrinsics):
        cam_model = make_camera_model(dummy_intrinsics)
        depth_model = build_depth_camera_model(cam_model, dummy_depth.shape)

        mask = (dummy_depth > 0.1).astype(np.uint8)
        pixels, depths = extract_mask_pixels(mask, dummy_depth, subsample=4)

        def project():
            return depth_model.project_pixels_to_3d(pixels, depths)

        result = benchmark(project)
        assert result.shape[1] == 3

    def test_imu_rotation(self, benchmark):
        points = np.random.randn(5000, 3).astype(np.float32)
        orientation = np.array([0.0, 0.0, 0.0, 1.0])

        result = benchmark(apply_imu_rotation, points, orientation)
        assert result.shape == points.shape


# ---------------------------------------------------------------------------
# Costmap
# ---------------------------------------------------------------------------

class TestCostmap:
    def test_costmap_build(self, benchmark):
        drivable = np.random.randn(3000, 3).astype(np.float32)
        drivable[:, 0] = np.abs(drivable[:, 0]) * 5  # forward
        lane = np.random.randn(500, 3).astype(np.float32)
        det = np.random.randn(200, 3).astype(np.float32)

        result = benchmark(build_costmap, drivable, lane, det)
        assert result.ndim == 2
        assert result.shape[0] == result.shape[1]
        assert result.dtype == np.int8


# ---------------------------------------------------------------------------
# Full pipeline
# ---------------------------------------------------------------------------

class TestPipeline:
    @pytest.fixture(scope="class")
    def pipeline(self, device):
        from couch_perception.perception_pipeline import PerceptionPipeline

        return PerceptionPipeline(device=device, conf=0.3, model_path="yolov8n.pt")

    def test_full_pipeline(self, benchmark, pipeline, dummy_synced_frame):
        for _ in range(3):
            pipeline.process_frame(dummy_synced_frame)

        mem_before = _gpu_mem_mb()
        result = benchmark(pipeline.process_frame, dummy_synced_frame)
        mem_after = _gpu_mem_mb()

        assert result is not None
        print(f"\n  GPU mem: {mem_before:.0f} → {mem_after:.0f} MB")

    def test_full_costmap_pipeline(self, benchmark, pipeline, dummy_synced_frame):
        def run():
            r = pipeline.process_frame(dummy_synced_frame)
            return build_costmap(r.drivable_pts, r.lane_pts, r.det_pts)

        for _ in range(3):
            run()

        result = benchmark(run)
        assert result.ndim == 2


# ---------------------------------------------------------------------------
# System profiling (not via pytest-benchmark — manual timing + tegrastats)
# ---------------------------------------------------------------------------

@pytest.mark.slow
def test_system_profile(dummy_synced_frame, tmp_path, device):
    """Run pipeline for many frames while optionally capturing tegrastats."""
    from couch_perception.perception_pipeline import PerceptionPipeline

    pipeline = PerceptionPipeline(device=device, conf=0.3, model_path="yolov8n.pt")
    n_frames = 50
    tegra_log = tmp_path / "tegrastats.log"

    # Try to start tegrastats in background (Jetson only)
    tegra_proc = None
    try:
        tegra_proc = subprocess.Popen(
            ["tegrastats", "--interval", "200", "--logfile", str(tegra_log)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        pass  # not on Jetson

    times: list[float] = []
    for _ in range(n_frames):
        t0 = time.perf_counter()
        pipeline.process_frame(dummy_synced_frame)
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        times.append(time.perf_counter() - t0)

    if tegra_proc:
        tegra_proc.terminate()
        tegra_proc.wait()

    times_ms = [t * 1000 for t in times]
    avg = sum(times_ms) / len(times_ms)
    fps = 1000.0 / avg if avg > 0 else 0

    report = textwrap.dedent(f"""\
        === System Profile ({n_frames} frames) ===
        Avg latency : {avg:.1f} ms
        Min latency : {min(times_ms):.1f} ms
        Max latency : {max(times_ms):.1f} ms
        FPS         : {fps:.1f}
        Device      : {pipeline.device}
        CUDA mem    : {_gpu_mem_mb():.0f} MB
    """)

    if tegra_log.exists() and tegra_log.stat().st_size > 0:
        report += f"\nTegrastats log: {tegra_log}\n"
        # Parse a few key metrics from last line
        last_line = tegra_log.read_text().strip().splitlines()[-1]
        report += f"Last tegrastats: {last_line}\n"

    print(report)
    assert avg > 0
