"""GPU-accelerated image utilities with CPU fallback.

Uses cv2.cuda for GPU resize when available (Jetson), otherwise falls back to CPU.
"""

import cv2
import numpy as np

_CUDA_AVAILABLE: bool | None = None


def _check_cuda() -> bool:
    """Check if cv2.cuda module is available and functional."""
    global _CUDA_AVAILABLE
    if _CUDA_AVAILABLE is None:
        try:
            _CUDA_AVAILABLE = cv2.cuda.getCudaEnabledDeviceCount() > 0
        except (cv2.error, AttributeError):
            _CUDA_AVAILABLE = False
    return _CUDA_AVAILABLE


def resize_image(
    image: np.ndarray,
    target_size: tuple[int, int],
    interpolation: int = cv2.INTER_LINEAR,
) -> np.ndarray:
    """Resize image using GPU if available, else CPU.

    Args:
        image: Input image (HWC or HW format)
        target_size: (width, height) tuple
        interpolation: cv2 interpolation flag (INTER_LINEAR, INTER_NEAREST, INTER_AREA)

    Returns:
        Resized image
    """
    if _check_cuda():
        try:
            gpu_img = cv2.cuda_GpuMat()
            gpu_img.upload(image)
            resized_gpu = cv2.cuda.resize(gpu_img, target_size, interpolation=interpolation)
            return resized_gpu.download()
        except cv2.error:
            pass
    return cv2.resize(image, target_size, interpolation=interpolation)


def resize_depth(
    depth: np.ndarray,
    target_size: tuple[int, int],
) -> np.ndarray:
    """Resize depth image using nearest-neighbor interpolation.

    Nearest-neighbor preserves depth values without interpolation artifacts.
    """
    return resize_image(depth, target_size, cv2.INTER_NEAREST)
