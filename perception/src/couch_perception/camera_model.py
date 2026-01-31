"""Camera models for projecting 2D pixels + depth into 3D points."""

from abc import ABC, abstractmethod

import cv2
import numpy as np

from couch_perception.bag_reader import CameraIntrinsics


class CameraModel(ABC):
    """Base class for camera projection models."""

    def __init__(self, intrinsics: CameraIntrinsics) -> None:
        self.intrinsics = intrinsics
        self.K = intrinsics.K
        self.D = intrinsics.D
        self.K_inv = np.linalg.inv(self.K)

    @abstractmethod
    def project_pixels_to_3d(
        self, pixels: np.ndarray, depths: np.ndarray
    ) -> np.ndarray:
        """Project 2D pixel coordinates with depth values to 3D camera-frame points.

        Args:
            pixels: (N, 2) array of (u, v) pixel coordinates.
            depths: (N,) array of depth values in meters.

        Returns:
            (N, 3) array of (x, y, z) points in the camera frame.
        """
        ...


class PlumbBobModel(CameraModel):
    """Plumb bob (Brown-Conrady) distortion model.

    Standard 5-parameter radial-tangential distortion used by ROS.
    If distortion coefficients are all zero, skips undistortion for speed.
    """

    def __init__(self, intrinsics: CameraIntrinsics) -> None:
        super().__init__(intrinsics)
        self._has_distortion = np.any(np.abs(self.D) > 1e-9)
        if self._has_distortion:
            # Pre-compute the undistortion map for the full image size
            self._map_x, self._map_y = cv2.initUndistortRectifyMap(
                self.K,
                self.D,
                None,
                self.K,
                (intrinsics.width, intrinsics.height),
                cv2.CV_32FC1,
            )

    def project_pixels_to_3d(
        self, pixels: np.ndarray, depths: np.ndarray
    ) -> np.ndarray:
        if len(pixels) == 0:
            return np.empty((0, 3), dtype=np.float32)

        pts = pixels.astype(np.float64)

        if self._has_distortion:
            pts_undistorted = cv2.undistortPoints(
                pts.reshape(-1, 1, 2), self.K, self.D, P=self.K
            ).reshape(-1, 2)
        else:
            pts_undistorted = pts

        # Homogeneous pixel coords → normalized camera coords
        ones = np.ones((len(pts_undistorted), 1), dtype=np.float64)
        pixels_h = np.hstack([pts_undistorted, ones])  # (N, 3)
        rays = (self.K_inv @ pixels_h.T).T  # (N, 3)

        # Scale by depth
        points_3d = rays * depths[:, np.newaxis]
        return points_3d.astype(np.float32)


def make_camera_model(intrinsics: CameraIntrinsics) -> CameraModel:
    """Factory function to create the appropriate camera model.

    Args:
        intrinsics: Camera intrinsic parameters including distortion model name.

    Returns:
        A CameraModel instance.

    Raises:
        ValueError: If the distortion model is not supported.
    """
    model_name = intrinsics.distortion_model.lower()
    if model_name in ("plumb_bob", "plumb-bob", ""):
        return PlumbBobModel(intrinsics)
    raise ValueError(
        f"Unsupported distortion model '{intrinsics.distortion_model}'. "
        f"Supported: plumb_bob. Extend camera_model.py to add new models."
    )
