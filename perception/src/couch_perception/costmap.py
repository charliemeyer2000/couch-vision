"""Ego-centric costmap generation from projected 3D perception points.

Builds a Nav2-compatible occupancy grid centered at (0,0) with:
  - Drivable area → cost 0 (free)
  - Lane lines → cost 50 (soft barrier, crossable)
  - Obstacles → cost 100 (lethal)
  - Inside FOV, no data → cost 0 (free — visible and clear)
  - Outside FOV → cost -1 (unknown)
"""

from __future__ import annotations

import math

import numpy as np

from couch_perception.costmap_visualizer import (
    COST_FREE,
    COST_LANE,
    COST_LETHAL,
    COST_UNSEEN,
)

# Grid parameters
GRID_SIZE_M = 20.0
GRID_RESOLUTION = 0.2
GRID_CELLS = int(GRID_SIZE_M / GRID_RESOLUTION)
GRID_ORIGIN = -GRID_SIZE_M / 2.0

EGO_DRIVABLE_RADIUS_M = 1.0

FOV_DEG = 120.0
FOV_HALF_RAD = math.radians(FOV_DEG / 2.0)


def _build_fov_mask() -> np.ndarray:
    """Pre-compute a boolean mask where True = inside FOV, False = outside.

    Forward direction is -X (negated to align with ENU/Google Maps).
    Grid layout: row 0 = most negative x (forward), row N = most positive x (behind).
    Col 0 = min y (left), col N = max y (right).
    """
    mask = np.zeros((GRID_CELLS, GRID_CELLS), dtype=bool)

    for r in range(GRID_CELLS):
        for c in range(GRID_CELLS):
            x = GRID_ORIGIN + r * GRID_RESOLUTION + GRID_RESOLUTION / 2
            y = GRID_ORIGIN + c * GRID_RESOLUTION + GRID_RESOLUTION / 2

            if x < 0:
                angle = abs(math.atan2(y, -x))
                if angle <= FOV_HALF_RAD:
                    mask[r, c] = True

    return mask


_FOV_MASK = _build_fov_mask()


def _build_ego_drivable_mask() -> np.ndarray:
    """Boolean mask where True = within EGO_DRIVABLE_RADIUS_M of ego (grid center)."""
    r_cells = EGO_DRIVABLE_RADIUS_M / GRID_RESOLUTION
    center = (GRID_CELLS - 1) / 2.0
    ys, xs = np.ogrid[:GRID_CELLS, :GRID_CELLS]
    dist_sq = (ys - center) ** 2 + (xs - center) ** 2
    return dist_sq <= r_cells**2


_EGO_DRIVABLE_MASK = _build_ego_drivable_mask()


def _world_to_grid(
    x: np.ndarray, y: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert world coordinates (meters, ego at 0,0) to grid row/col indices."""
    col = ((y - GRID_ORIGIN) / GRID_RESOLUTION).astype(np.int32)
    row = ((x - GRID_ORIGIN) / GRID_RESOLUTION).astype(np.int32)
    valid = (row >= 0) & (row < GRID_CELLS) & (col >= 0) & (col < GRID_CELLS)
    return row, col, valid


def build_costmap(
    drivable_pts: np.ndarray | None,
    lane_pts: np.ndarray | None,
    det_pts: np.ndarray | None,
    det_groups: list[np.ndarray] | None = None,
) -> np.ndarray:
    """Build a costmap grid from projected 3D points.

    Points are in ego-centric world frame (x=forward, y=lateral, z=up).

    If det_groups is provided (list of per-detection (N,3) point arrays),
    each detection's ground-plane bounding box is filled as lethal cost.
    Otherwise falls back to rasterizing det_pts as individual points.

    Returns:
        (GRID_CELLS, GRID_CELLS) int8 array with Nav2-compatible cost values.
    """
    grid = np.full((GRID_CELLS, GRID_CELLS), COST_UNSEEN, dtype=np.int8)
    grid[_FOV_MASK] = COST_FREE
    grid[_EGO_DRIVABLE_MASK] = COST_FREE

    if drivable_pts is not None and len(drivable_pts) > 0:
        row, col, valid = _world_to_grid(drivable_pts[:, 0], drivable_pts[:, 1])
        grid[row[valid], col[valid]] = COST_FREE

    if lane_pts is not None and len(lane_pts) > 0:
        row, col, valid = _world_to_grid(lane_pts[:, 0], lane_pts[:, 1])
        grid[row[valid], col[valid]] = COST_LANE

    if det_groups is not None:
        for group_pts in det_groups:
            if len(group_pts) < 2:
                continue
            x_min, x_max = group_pts[:, 0].min(), group_pts[:, 0].max()
            y_min, y_max = group_pts[:, 1].min(), group_pts[:, 1].max()
            r_min, c_min, _ = _world_to_grid(
                np.array([x_min]), np.array([y_min])
            )
            r_max, c_max, _ = _world_to_grid(
                np.array([x_max]), np.array([y_max])
            )
            r0 = max(0, int(r_min[0]))
            r1 = min(GRID_CELLS - 1, int(r_max[0]))
            c0 = max(0, int(c_min[0]))
            c1 = min(GRID_CELLS - 1, int(c_max[0]))
            if r0 <= r1 and c0 <= c1:
                grid[r0 : r1 + 1, c0 : c1 + 1] = COST_LETHAL
    elif det_pts is not None and len(det_pts) > 0:
        row, col, valid = _world_to_grid(det_pts[:, 0], det_pts[:, 1])
        grid[row[valid], col[valid]] = COST_LETHAL

    return grid
