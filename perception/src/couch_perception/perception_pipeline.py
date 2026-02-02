"""Core per-frame perception pipeline: detect → project to 3D → IMU rotate.

Shared by all runners (bev_projection, costmap, nav2_planner). Each runner
feeds frames in and gets back structured results without duplicating the
detect/project/rotate loop.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from couch_perception.bag_reader import SyncedFrame
from couch_perception.camera_model import CameraModel, make_camera_model
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


class PerceptionPipeline:
    """Runs YOLOv8 + YOLOP detection and projects results to 3D world coordinates.

    Lazy-initializes the depth camera model on the first frame.
    """

    def __init__(
        self,
        device: str | None = None,
        conf: float = 0.3,
        model_path: str | None = None,
        subsample_drivable: int = 4,
        subsample_lane: int = 2,
        subsample_bbox: int = 8,
    ) -> None:
        self.subsample_drivable = subsample_drivable
        self.subsample_lane = subsample_lane
        self.subsample_bbox = subsample_bbox

        self.yolo = YOLOv8Detector(
            model_path=model_path, conf_threshold=conf, device=device
        )
        self.yolop = YOLOPDetector(device=device)

        self._cam_model: CameraModel | None = None
        self._depth_cam_model: CameraModel | None = None

    @property
    def device(self) -> str:
        return self.yolo.device

    def _ensure_camera_model(self, frame: SyncedFrame) -> None:
        if self._cam_model is None:
            self._cam_model = make_camera_model(frame.intrinsics)
            self._depth_cam_model = build_depth_camera_model(
                self._cam_model, frame.depth.shape[:2]
            )

    def process_frame(self, frame: SyncedFrame) -> PerceptionResult:
        """Run full perception on a single frame and return projected 3D results."""
        self._ensure_camera_model(frame)
        dcm = self._depth_cam_model

        detections = self.yolo.detect(frame.image)
        yolop_result = self.yolop.detect(frame.image)

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
