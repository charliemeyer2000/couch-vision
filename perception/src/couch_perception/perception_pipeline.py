"""Core per-frame perception pipeline: detect → project to 3D → IMU rotate.

Shared by all runners (bev_projection, costmap, nav2_planner). Each runner
feeds frames in and gets back structured results without duplicating the
detect/project/rotate loop.
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field

import numpy as np
import torch

from couch_perception.bag_reader import SyncedFrame
from couch_perception.camera_model import CameraModel, make_camera_model
from couch_perception.config import PipelineConfig
from couch_perception.projection import (
    apply_imu_rotation,
    build_depth_camera_model,
    extract_bbox_pixels_grouped,
    extract_mask_pixels,
)
from couch_perception.yolov8_detector import Detection, YOLOv8Detector
from couch_perception.yolop_detector import YOLOPDetector, YOLOPResult


@dataclass
class PerceptionResult:
    """Output of a single frame through the perception pipeline."""

    timestamp: float
    detections: list[Detection]
    yolop_result: YOLOPResult | None
    drivable_pts: np.ndarray | None = None  # (N,3) world-frame
    lane_pts: np.ndarray | None = None
    det_pts: np.ndarray | None = None
    det_groups: list[np.ndarray] = field(default_factory=list)


def _resolve_detection_model(config: PipelineConfig) -> str | None:
    """Find the best available weights/engine for the configured detection model."""
    from pathlib import Path

    weights_dir = Path(__file__).resolve().parent.parent.parent / "weights"

    if config.detection_engine:
        return config.detection_engine

    name = config.detection_model
    if name == "none":
        return None

    # Prefer TensorRT engine, fall back to PyTorch weights
    engine = weights_dir / f"{name}.engine"
    if engine.exists():
        return str(engine)
    pt = weights_dir / f"{name}.pt"
    if pt.exists():
        return str(pt)
    # Download into weights dir so TRT engines persist on the volume mount
    return str(weights_dir / f"{name}.pt")


class PerceptionPipeline:
    """Runs YOLOv8 + YOLOP detection and projects results to 3D world coordinates.

    Lazy-initializes the depth camera model on the first frame.
    """

    def __init__(
        self,
        config: PipelineConfig | None = None,
        # Legacy kwargs — used if config is None
        device: str | None = None,
        conf: float = 0.3,
        model_path: str | None = None,
        subsample_drivable: int = 4,
        subsample_lane: int = 2,
        subsample_bbox: int = 8,
    ) -> None:
        if config is None:
            # Legacy path: build config from kwargs
            config = PipelineConfig(
                device=device,
                detection_confidence=conf,
                subsample_drivable=subsample_drivable,
                subsample_lane=subsample_lane,
                subsample_bbox=subsample_bbox,
            )
            self._legacy_model_path = model_path
        else:
            self._legacy_model_path = None

        self.config = config
        self.subsample_drivable = config.subsample_drivable
        self.subsample_lane = config.subsample_lane
        self.subsample_bbox = config.subsample_bbox

        # Detection
        if config.detection_model != "none":
            det_path = self._legacy_model_path or _resolve_detection_model(config)
            self.yolo: YOLOv8Detector | None = YOLOv8Detector(
                model_path=det_path,
                conf_threshold=config.detection_confidence,
                device=config.device,
            )
        else:
            self.yolo = None

        # Segmentation
        if config.segmentation_model != "none":
            self.yolop: YOLOPDetector | None = YOLOPDetector(device=config.device)
        else:
            self.yolop = None

        # CUDA streams for concurrent inference
        self._use_streams = (
            config.cuda_streams
            and torch.cuda.is_available()
            and self.yolo is not None
            and self.yolop is not None
        )
        if self._use_streams:
            self._det_stream = torch.cuda.Stream()
            self._seg_stream = torch.cuda.Stream()

        # Thread pool for parallel CPU inference (torch releases GIL during forward)
        self._cpu_executor: ThreadPoolExecutor | None = None
        if not self._use_streams and self.yolo is not None and self.yolop is not None:
            self._cpu_executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="inference")

        self._cam_model: CameraModel | None = None
        self._depth_cam_model: CameraModel | None = None

    @property
    def device(self) -> str:
        if self.yolo:
            return self.yolo.device
        if self.yolop:
            return str(self.yolop.device)
        return "cpu"

    def shutdown(self) -> None:
        if self._cpu_executor:
            self._cpu_executor.shutdown(wait=False)
            self._cpu_executor = None

    def _ensure_camera_model(self, frame: SyncedFrame) -> None:
        if self._cam_model is None:
            self._cam_model = make_camera_model(frame.intrinsics)
            self._depth_cam_model = build_depth_camera_model(
                self._cam_model, frame.depth.shape[:2]
            )

    def _run_inference(
        self, image: np.ndarray
    ) -> tuple[list[Detection], YOLOPResult | None]:
        """Run detection and segmentation, optionally on separate CUDA streams."""
        if self._use_streams:
            detections: list[Detection] = []
            yolop_result: YOLOPResult | None = None

            with torch.cuda.stream(self._det_stream):
                if self.yolo:
                    detections = self.yolo.detect(image)
            with torch.cuda.stream(self._seg_stream):
                if self.yolop:
                    yolop_result = self.yolop.detect(image)

            torch.cuda.synchronize()
            return detections, yolop_result

        if self._cpu_executor:
            det_future = self._cpu_executor.submit(self.yolo.detect, image)
            seg_future = self._cpu_executor.submit(self.yolop.detect, image)
            return det_future.result(), seg_future.result()

        detections = self.yolo.detect(image) if self.yolo else []
        yolop_result = self.yolop.detect(image) if self.yolop else None
        return detections, yolop_result

    def process_frame(self, frame: SyncedFrame) -> PerceptionResult:
        """Run full perception on a single frame and return projected 3D results."""
        self._ensure_camera_model(frame)
        dcm = self._depth_cam_model

        detections, yolop_result = self._run_inference(frame.image)

        # Project drivable area
        drivable_pts = None
        if yolop_result and yolop_result.drivable_mask is not None:
            pixels, depths = extract_mask_pixels(
                yolop_result.drivable_mask,
                frame.depth,
                subsample=self.subsample_drivable,
            )
            if len(pixels) > 0:
                pts_3d = dcm.project_pixels_to_3d(pixels, depths)
                drivable_pts = apply_imu_rotation(pts_3d, frame.orientation)

        # Project lane lines
        lane_pts = None
        if yolop_result and yolop_result.lane_mask is not None:
            pixels, depths = extract_mask_pixels(
                yolop_result.lane_mask,
                frame.depth,
                subsample=self.subsample_lane,
            )
            if len(pixels) > 0:
                pts_3d = dcm.project_pixels_to_3d(pixels, depths)
                lane_pts = apply_imu_rotation(pts_3d, frame.orientation)

        # Project detections (per-detection groups)
        det_pts = None
        det_groups: list[np.ndarray] = []
        if detections:
            groups = extract_bbox_pixels_grouped(
                detections,
                frame.depth,
                frame.image.shape[:2],
                subsample=self.subsample_bbox,
            )
            all_world_pts: list[np.ndarray] = []
            for pixels, depths_g in groups:
                if len(pixels) == 0:
                    continue
                pts_3d = dcm.project_pixels_to_3d(pixels, depths_g)
                world_pts = apply_imu_rotation(pts_3d, frame.orientation)
                det_groups.append(world_pts)
                all_world_pts.append(world_pts)
            if all_world_pts:
                det_pts = np.vstack(all_world_pts)

        return PerceptionResult(
            timestamp=frame.timestamp,
            detections=detections,
            yolop_result=yolop_result,
            drivable_pts=drivable_pts,
            lane_pts=lane_pts,
            det_pts=det_pts,
            det_groups=det_groups,
        )
