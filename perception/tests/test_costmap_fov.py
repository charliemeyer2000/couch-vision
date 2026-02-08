"""Test costmap FOV mask fix using real bag data.

Compares costmap output with and without the FOV mask applied,
demonstrating the improvement for Nav2 path planning.

Run: cd perception && uv run pytest tests/test_costmap_fov.py -v --bag ../bags/filtered_walk_around.mcap
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from couch_perception.bag_reader import read_synced_frames, SyncedFrame
from couch_perception.camera_model import make_camera_model
from couch_perception.costmap import (
    build_costmap,
    _FOV_MASK,
    _EGO_DRIVABLE_MASK,
    GRID_CELLS,
)
from couch_perception.costmap_visualizer import COST_FREE, COST_LANE, COST_LETHAL, COST_UNSEEN
from couch_perception.projection import (
    apply_imu_rotation,
    build_depth_camera_model,
    extract_mask_pixels,
)


def _build_costmap_without_fov(
    drivable_pts: np.ndarray | None,
    lane_pts: np.ndarray | None,
    det_pts: np.ndarray | None,
) -> np.ndarray:
    """Simulate the old costmap behavior (no FOV mask)."""
    grid = np.full((GRID_CELLS, GRID_CELLS), COST_UNSEEN, dtype=np.int8)
    grid[_EGO_DRIVABLE_MASK] = COST_FREE

    if drivable_pts is not None and len(drivable_pts) > 0:
        from couch_perception.costmap import _world_to_grid
        row, col, valid = _world_to_grid(drivable_pts[:, 0], drivable_pts[:, 1])
        grid[row[valid], col[valid]] = COST_FREE

    if lane_pts is not None and len(lane_pts) > 0:
        from couch_perception.costmap import _world_to_grid
        row, col, valid = _world_to_grid(lane_pts[:, 0], lane_pts[:, 1])
        grid[row[valid], col[valid]] = COST_LANE

    return grid


# ── Unit tests (no bag required) ──────────────────────────────────────────


class TestCostmapFOV:
    def test_fov_mask_applied_to_empty_costmap(self):
        """FOV cells should be free, not unknown, even with no perception data."""
        grid = build_costmap(None, None, None)
        fov_free = np.sum((grid == COST_FREE) & _FOV_MASK)
        assert fov_free == _FOV_MASK.sum(), "All FOV cells should be COST_FREE"

    def test_outside_fov_remains_unknown(self):
        """Cells outside the FOV should stay COST_UNSEEN."""
        grid = build_costmap(None, None, None)
        outside_fov = ~_FOV_MASK & ~_EGO_DRIVABLE_MASK
        unseen_outside = np.sum(grid[outside_fov] == COST_UNSEEN)
        assert unseen_outside == outside_fov.sum(), "Non-FOV, non-ego cells should be unknown"

    def test_obstacles_overwrite_fov_free(self):
        """Obstacle points within the FOV should overwrite free cells as lethal."""
        det_pts = np.array([[-3.0, 0.0, 0.0], [-5.0, 1.0, 0.0]], dtype=np.float32)
        grid = build_costmap(None, None, det_pts)
        lethal = np.sum(grid == COST_LETHAL)
        assert lethal >= 2, "Obstacle points should be marked lethal"

    def test_lanes_overwrite_fov_free(self):
        """Lane points within the FOV should overwrite free cells."""
        lane_pts = np.array([[-4.0, 0.5, 0.0], [-4.0, -0.5, 0.0]], dtype=np.float32)
        grid = build_costmap(None, lane_pts, None)
        lane_cells = np.sum(grid == COST_LANE)
        assert lane_cells >= 2, "Lane points should be marked as lane cost"

    def test_ego_center_always_free(self):
        """The ego cell at grid center should always be free."""
        grid = build_costmap(None, None, None)
        assert grid[GRID_CELLS // 2, GRID_CELLS // 2] == COST_FREE

    def test_fov_reduces_unknown_space(self):
        """The FOV mask should reduce unknown space vs the old approach."""
        grid_new = build_costmap(None, None, None)
        grid_old = _build_costmap_without_fov(None, None, None)

        new_unseen = np.sum(grid_new == COST_UNSEEN)
        old_unseen = np.sum(grid_old == COST_UNSEEN)
        assert new_unseen < old_unseen, (
            f"FOV fix should reduce unknown cells: {new_unseen} vs {old_unseen}"
        )


# ── Bag-based integration test ────────────────────────────────────────────


class TestCostmapFOVWithBag:
    @pytest.fixture(scope="class")
    def bag_frames(self, bag_path) -> list[SyncedFrame]:
        if bag_path is None:
            pytest.skip("No --bag provided")
        frames = list(read_synced_frames(bag_path))
        if not frames:
            pytest.skip("No synced frames in bag")
        return frames[:10]  # Test on first 10 frames

    def test_real_data_before_after(self, bag_frames):
        """Compare costmap stats before/after FOV fix on real bag frames."""
        old_free_total = 0
        old_unseen_total = 0
        new_free_total = 0
        new_unseen_total = 0

        cam_model = make_camera_model(bag_frames[0].intrinsics)
        depth_model = build_depth_camera_model(cam_model, bag_frames[0].depth.shape[:2])

        for i, frame in enumerate(bag_frames):
            # Project depth to 3D as drivable area (simulating YOLOP output)
            mask = (frame.depth > 0.1).astype(np.uint8)
            pixels, depths = extract_mask_pixels(mask, frame.depth, subsample=4)
            drivable_pts = None
            if len(pixels) > 0:
                pts_3d = depth_model.project_pixels_to_3d(pixels, depths)
                drivable_pts = apply_imu_rotation(pts_3d, frame.orientation)

            # New costmap (with FOV mask)
            grid_new = build_costmap(drivable_pts, None, None)
            new_free = int(np.sum(grid_new == COST_FREE))
            new_unseen = int(np.sum(grid_new == COST_UNSEEN))

            # Old costmap (without FOV mask)
            grid_old = _build_costmap_without_fov(drivable_pts, None, None)
            old_free = int(np.sum(grid_old == COST_FREE))
            old_unseen = int(np.sum(grid_old == COST_UNSEEN))

            old_free_total += old_free
            old_unseen_total += old_unseen
            new_free_total += new_free
            new_unseen_total += new_unseen

            print(
                f"  Frame {i}: OLD free={old_free} unseen={old_unseen} | "
                f"NEW free={new_free} unseen={new_unseen}"
            )

        n = len(bag_frames)
        total = GRID_CELLS * GRID_CELLS
        print(f"\n  === Average over {n} frames ===")
        print(f"  OLD: free={old_free_total/n:.0f} ({old_free_total/n/total*100:.1f}%), "
              f"unseen={old_unseen_total/n:.0f} ({old_unseen_total/n/total*100:.1f}%)")
        print(f"  NEW: free={new_free_total/n:.0f} ({new_free_total/n/total*100:.1f}%), "
              f"unseen={new_unseen_total/n:.0f} ({new_unseen_total/n/total*100:.1f}%)")

        assert new_unseen_total < old_unseen_total, "FOV fix should reduce unknown cells on real data"
        assert new_free_total > old_free_total, "FOV fix should increase free cells on real data"
