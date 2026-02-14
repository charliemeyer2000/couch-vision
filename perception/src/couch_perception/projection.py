"""Geometry utilities for projecting 2D perception results into 3D world coordinates.

Functions to extract pixel coordinates from masks/bounding boxes, project them
to 3D using depth + camera intrinsics, and rotate into a gravity-aligned frame
using IMU orientation.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import cv2
import numpy as np

from couch_perception.camera_model import CameraModel, make_camera_model

if TYPE_CHECKING:
    from couch_perception.yolov8_detector import Detection


def quat_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Convert quaternion (x, y, z, w) to a 3x3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


_DEFAULT_CAMERA_TO_BASE_ROTATION = np.array(
    [
        [0.0, 0.0, 1.0],   # base +x (forward) <- camera +z
        [-1.0, 0.0, 0.0],  # base +y (left) <- camera -x
        [0.0, -1.0, 0.0],  # base +z (up) <- camera -y
    ],
    dtype=np.float64,
)


def apply_imu_rotation(
    points: np.ndarray,
    orientation: np.ndarray | None,
    camera_to_base_rotation: np.ndarray | None = None,
) -> np.ndarray:
    """Rotate projected points with IMU and map them into a forward-facing base frame.

    The IMU quaternion is used for gravity alignment (inverse rotation), then points
    are remapped from camera axes into base axes. The axis remap defaults to the
    historical camera-optical mapping but can be overridden with a URDF-derived
    camera link rotation matrix.
    """
    if len(points) == 0:
        return points

    pts_rot = points
    if orientation is not None:
        R = quat_to_rotation_matrix(orientation)
        pts_rot = (R.T @ points.T).T

    camera_to_base = (
        _DEFAULT_CAMERA_TO_BASE_ROTATION
        if camera_to_base_rotation is None
        else np.asarray(camera_to_base_rotation, dtype=np.float64)
    )
    if camera_to_base.shape != (3, 3):
        raise ValueError("camera_to_base_rotation must be shape (3, 3)")

    return (camera_to_base @ pts_rot.T).T.astype(np.float32)


MASK_DEPTH_MIN = 0.1
MASK_DEPTH_MAX = 8.0   # iPhone LiDAR reliable range (~5m, clamped conservatively)
BBOX_DEPTH_MIN = 0.1
BBOX_DEPTH_MAX = 20.0


def extract_mask_pixels(
    mask: np.ndarray, depth: np.ndarray, subsample: int = 4
) -> tuple[np.ndarray, np.ndarray]:
    """Get pixel coordinates and depth values where a binary mask is active.

    The mask and depth may differ in resolution; depth is the authority for
    the coordinate space. The mask is resized to match depth dimensions.

    Returns:
        pixels: (M, 2) array of (u, v) pixel coords in depth image space.
        depths: (M,) array of depth values.
    """
    dh, dw = depth.shape[:2]
    if mask.shape[:2] != (dh, dw):
        mask = cv2.resize(mask.astype(np.uint8), (dw, dh), interpolation=cv2.INTER_NEAREST)

    ys, xs = np.where(
        (mask > 0) & (depth > MASK_DEPTH_MIN) & (depth < MASK_DEPTH_MAX)
    )
    if subsample > 1:
        idx = np.arange(0, len(ys), subsample)
        ys, xs = ys[idx], xs[idx]

    pixels = np.column_stack([xs, ys])
    d = depth[ys, xs]
    return pixels, d


def _extract_single_bbox(
    det: Detection,
    depth: np.ndarray,
    scale_x: float,
    scale_y: float,
    subsample: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Extract valid pixel coords and depths for one detection bbox."""
    dh, dw = depth.shape[:2]
    x1 = max(0, int(det.x1 * scale_x))
    y1 = max(0, int(det.y1 * scale_y))
    x2 = min(dw - 1, int(det.x2 * scale_x))
    y2 = min(dh - 1, int(det.y2 * scale_y))

    ys, xs = np.mgrid[y1:y2:subsample, x1:x2:subsample]
    xs, ys = xs.ravel(), ys.ravel()
    d = depth[ys, xs]

    valid = (d > BBOX_DEPTH_MIN) & (d < BBOX_DEPTH_MAX)
    return np.column_stack([xs[valid], ys[valid]]), d[valid]


def extract_bbox_pixels(
    detections: list[Detection],
    depth: np.ndarray,
    image_shape: tuple[int, int],
    subsample: int = 8,
) -> tuple[np.ndarray, np.ndarray]:
    """Get pixel coordinates and depths for all detection bounding boxes.

    Bounding box coords are in the original image space; we scale them
    to the depth image resolution.
    """
    dh, dw = depth.shape[:2]
    ih, iw = image_shape
    scale_x, scale_y = dw / iw, dh / ih

    groups = [
        _extract_single_bbox(det, depth, scale_x, scale_y, subsample)
        for det in detections
    ]
    if not groups:
        return np.empty((0, 2)), np.empty((0,))

    pixels, depths = zip(*groups)
    return np.vstack(pixels), np.concatenate(depths)


def extract_bbox_pixels_grouped(
    detections: list[Detection],
    depth: np.ndarray,
    image_shape: tuple[int, int],
    subsample: int = 8,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """Like extract_bbox_pixels but returns per-detection groups."""
    dh, dw = depth.shape[:2]
    ih, iw = image_shape
    scale_x, scale_y = dw / iw, dh / ih

    return [
        _extract_single_bbox(det, depth, scale_x, scale_y, subsample)
        for det in detections
    ]


def build_depth_camera_model(
    cam_model: CameraModel, depth_shape: tuple[int, int]
) -> CameraModel:
    """Create a camera model scaled to the depth image resolution.

    The bag's CameraInfo is at the RGB image resolution. The depth image
    is typically lower resolution. We scale the intrinsic matrix K accordingly.
    """
    from couch_perception.bag_reader import CameraIntrinsics

    dh, dw = depth_shape
    orig = cam_model.intrinsics
    sx = dw / orig.width
    sy = dh / orig.height

    K_scaled = orig.K.copy()
    K_scaled[0, :] *= sx
    K_scaled[1, :] *= sy

    scaled_intrinsics = CameraIntrinsics(
        width=dw,
        height=dh,
        K=K_scaled,
        D=orig.D,
        distortion_model=orig.distortion_model,
        frame_id=orig.frame_id,
    )
    return make_camera_model(scaled_intrinsics)


def validate_ground_plane(pts: np.ndarray, label: str) -> None:
    """Assert that X/Y variance exceeds Z variance, confirming the ground plane is in XY."""
    if len(pts) < 10:
        return
    var_x, var_y, var_z = pts[:, 0].var(), pts[:, 1].var(), pts[:, 2].var()
    if var_z >= var_x or var_z >= var_y:
        logging.warning(
            f"{label}: ground plane check FAILED — var(x)={var_x:.3f} var(y)={var_y:.3f} var(z)={var_z:.3f}. "
            f"Expected var(z) < var(x) and var(z) < var(y)."
        )
