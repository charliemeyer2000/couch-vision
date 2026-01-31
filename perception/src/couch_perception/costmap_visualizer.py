"""Color-coded visualization of costmap grids."""

import cv2
import numpy as np


# Cost value constants
COST_UNKNOWN = -1 # not the end of the world to drive on
COST_FREE = 0
COST_LANE = 80
COST_FOV_BOUNDARY = 99
COST_LETHAL = 100

# BGR colors for visualization
_COLOR_MAP = {
    COST_UNKNOWN: (0, 0, 0),        # black
    COST_FREE: (0, 200, 0),         # green
    COST_LANE: (0, 200, 200),       # yellow
    COST_FOV_BOUNDARY: (50, 50, 50),  # dark gray
    COST_LETHAL: (0, 0, 220),       # red
}


def costmap_to_color_image(grid: np.ndarray) -> np.ndarray:
    """Convert a costmap (int8 values) to a BGR color image.

    Args:
        grid: (H, W) array with values in {-1, 0, 80, 99, 100}.

    Returns:
        (H, W, 3) uint8 BGR image.
    """
    h, w = grid.shape
    img = np.zeros((h, w, 3), dtype=np.uint8)

    for cost_val, color in _COLOR_MAP.items():
        mask = grid == cost_val
        img[mask] = color

    # Anything not in the map → gradient gray based on value
    known_values = set(_COLOR_MAP.keys())
    for cost_val in np.unique(grid):
        if int(cost_val) not in known_values:
            # Linear interpolation: 1-79 → blue-ish gradient
            t = max(0, min(int(cost_val), 100)) / 100.0
            c = (int(50 * (1 - t)), int(200 * (1 - t)), int(50 + 170 * t))
            img[grid == cost_val] = c

    return img


def costmap_to_upscaled_image(grid: np.ndarray, scale: int = 4) -> np.ndarray:
    """Create an upscaled color image for better visibility in Foxglove."""
    img = costmap_to_color_image(grid)
    return cv2.resize(img, (grid.shape[1] * scale, grid.shape[0] * scale),
                      interpolation=cv2.INTER_NEAREST)
