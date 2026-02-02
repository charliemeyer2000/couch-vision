"""Geometry utilities for projecting 2D perception results into 3D world coordinates.

Functions to extract pixel coordinates from masks/bounding boxes, project them
to 3D using depth + camera intrinsics, and rotate into a gravity-aligned frame
using IMU orientation.
"""

from __future__ import annotations

import logging

import cv2
import numpy as np

from couch_perception.camera_model import CameraModel, make_camera_model
from couch_perception.yolov8_detector import Detection


def quat_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Convert quaternion (x, y, z, w) to a 3x3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def apply_imu_rotation(points: np.ndarray, orientation: np.ndarray | None) -> np.ndarray:
    """Rotate camera-frame points into a gravity-aligned world frame using IMU orientation.

    Uses the inverse of the IMU orientation quaternion to rotate camera-frame points
    into a gravity-aligned frame, then permutes axes so that X/Y span the ground plane
    and Z points up.
    """
    if orientation is None or len(points) == 0:
        return points
    R = quat_to_rotation_matrix(orientation)
    pts_world = (R.T @ points.T).T
    result = np.empty_like(pts_world)
    result[:, 0] = -pts_world[:, 0]  # forward → -X (negated to align with ENU/Google Maps)
    result[:, 1] = pts_world[:, 2]   # lateral → Y
    result[:, 2] = -pts_world[:, 1]  # -gravity → Z (up)
    return result.astype(np.float32)


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

    ys, xs = np.where((mask > 0) & (depth > 0.1) & (depth < 50.0))
    if subsample > 1:
        idx = np.arange(0, len(ys), subsample)
        ys, xs = ys[idx], xs[idx]

    pixels = np.column_stack([xs, ys])
    d = depth[ys, xs]
    return pixels, d


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
    scale_x = dw / iw
    scale_y = dh / ih

    all_pixels = []
    all_depths = []

    for det in detections:
        x1 = max(0, int(det.x1 * scale_x))
        y1 = max(0, int(det.y1 * scale_y))
        x2 = min(dw - 1, int(det.x2 * scale_x))
        y2 = min(dh - 1, int(det.y2 * scale_y))

        ys, xs = np.mgrid[y1:y2:subsample, x1:x2:subsample]
        xs = xs.ravel()
        ys = ys.ravel()
        d = depth[ys, xs]

        valid = (d > 0.1) & (d < 20.0)
        all_pixels.append(np.column_stack([xs[valid], ys[valid]]))
        all_depths.append(d[valid])

    if not all_pixels:
        return np.empty((0, 2)), np.empty((0,))

    return np.vstack(all_pixels), np.concatenate(all_depths)


def extract_bbox_pixels_grouped(
    detections: list[Detection],
    depth: np.ndarray,
    image_shape: tuple[int, int],
    subsample: int = 8,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """Like extract_bbox_pixels but returns per-detection groups.

    Returns a list of (pixels, depths) tuples, one per detection.
    """
    dh, dw = depth.shape[:2]
    ih, iw = image_shape
    scale_x = dw / iw
    scale_y = dh / ih

    groups = []
    for det in detections:
        x1 = max(0, int(det.x1 * scale_x))
        y1 = max(0, int(det.y1 * scale_y))
        x2 = min(dw - 1, int(det.x2 * scale_x))
        y2 = min(dh - 1, int(det.y2 * scale_y))

        ys, xs = np.mgrid[y1:y2:subsample, x1:x2:subsample]
        xs = xs.ravel()
        ys = ys.ravel()
        d = depth[ys, xs]

        valid = (d > 0.1) & (d < 20.0)
        pixels = np.column_stack([xs[valid], ys[valid]])
        groups.append((pixels, d[valid]))

    return groups


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
