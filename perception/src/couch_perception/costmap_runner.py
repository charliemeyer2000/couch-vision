"""CLI entry point: generate Nav2-compatible costmaps from perception on MCAP bags.

Runs YOLOv8 + YOLOP perception, projects results to 3D, and builds an
ego-centric occupancy grid centered at (0,0) with:
  - Drivable area → cost 0 (free)
  - Lane lines → cost 80 (soft barrier)
  - Obstacles → cost 100 (lethal)
  - Outside 70° FOV → cost 99 (high cost)
  - Unknown → cost -1

Publishes via Foxglove WebSocket on port 8765 for visualization.
"""

import argparse
import datetime
import logging
import math
import time

import cv2
import numpy as np

import foxglove
from foxglove.schemas import (
    CompressedImage,
    Grid,
    PackedElementField,
    PackedElementFieldNumericType,
    Pose,
    Quaternion,
    Timestamp,
    Vector2,
    Vector3,
)
from foxglove.websocket import ServerListener, Client, ChannelView

from couch_perception.frame_source import BagSource
from couch_perception.perception_pipeline import PerceptionPipeline
from couch_perception.costmap_visualizer import (
    COST_UNKNOWN,
    COST_FREE,
    COST_LANE,
    COST_FOV_BOUNDARY,
    COST_LETHAL,
    costmap_to_upscaled_image,
)
from couch_perception.bev_projection_runner import (
    pack_points_rgba_vectorized,
    make_pointcloud,
)

logger = logging.getLogger(__name__)

# Grid parameters
GRID_SIZE_M = 40.0      # 40m x 40m (covers full depth range ~20m forward)
GRID_RESOLUTION = 0.3   # 50cm per cell
GRID_CELLS = int(GRID_SIZE_M / GRID_RESOLUTION)
GRID_ORIGIN = -GRID_SIZE_M / 2.0  # -20.0m (grid centered at 0,0)

# Radius around ego assumed drivable (camera can't see the ground nearby)
EGO_DRIVABLE_RADIUS_M = 1.0

# FOV parameters
FOV_DEG = 120.0
FOV_HALF_RAD = math.radians(FOV_DEG / 2.0)


class _Listener(ServerListener):
    def __init__(self) -> None:
        self.subscribers: dict[int, set[str]] = {}

    def on_subscribe(self, client: Client, channel: ChannelView) -> None:
        self.subscribers.setdefault(client.id, set()).add(channel.topic)
        logger.info(f"Client {client.id} subscribed to {channel.topic}")

    def on_unsubscribe(self, client: Client, channel: ChannelView) -> None:
        if client.id in self.subscribers:
            self.subscribers[client.id].discard(channel.topic)
            if not self.subscribers[client.id]:
                del self.subscribers[client.id]


def _build_fov_mask() -> np.ndarray:
    """Pre-compute a boolean mask where True = inside FOV, False = outside.

    Forward direction is -X (negated to align with ENU/Google Maps).
    Grid layout: row 0 = most negative x (forward), row 99 = most positive x (behind).
    Col 0 = min y (left), col 99 = max y (right).
    """
    mask = np.zeros((GRID_CELLS, GRID_CELLS), dtype=bool)

    for r in range(GRID_CELLS):
        for c in range(GRID_CELLS):
            # World coords relative to ego at center
            x = GRID_ORIGIN + r * GRID_RESOLUTION + GRID_RESOLUTION / 2
            y = GRID_ORIGIN + c * GRID_RESOLUTION + GRID_RESOLUTION / 2

            # Forward is -X direction
            if x < 0:
                angle = abs(math.atan2(y, -x))
                if angle <= FOV_HALF_RAD:
                    mask[r, c] = True

    return mask


# Pre-compute once at import time
_FOV_MASK = _build_fov_mask()


def _build_ego_drivable_mask() -> np.ndarray:
    """Boolean mask where True = within EGO_DRIVABLE_RADIUS_M of ego (grid center)."""
    r_cells = EGO_DRIVABLE_RADIUS_M / GRID_RESOLUTION
    center = (GRID_CELLS - 1) / 2.0
    ys, xs = np.ogrid[:GRID_CELLS, :GRID_CELLS]
    dist_sq = (ys - center) ** 2 + (xs - center) ** 2
    return dist_sq <= r_cells ** 2


_EGO_DRIVABLE_MASK = _build_ego_drivable_mask()


def _world_to_grid(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert world coordinates (meters, ego at 0,0) to grid row/col indices.

    Forward is -X, so row 0 = most negative x (forward), row GRID_CELLS-1 = most positive x (behind).
    Col 0 = min y (left), col GRID_CELLS-1 = max y (right).
    """
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
    """Build a 100x100 costmap from projected 3D points.

    Points are in ego-centric world frame (x=forward, y=lateral, z=up).
    Grid is centered at (0,0).

    If det_groups is provided (list of per-detection (N,3) point arrays),
    each detection's ground-plane bounding box is filled as lethal cost.
    Otherwise falls back to rasterizing det_pts as individual points.

    Returns:
        (GRID_CELLS, GRID_CELLS) int8 array with Nav2-compatible cost values.
    """
    grid = np.full((GRID_CELLS, GRID_CELLS), COST_UNKNOWN, dtype=np.int8)

    # Outside FOV → high cost
    grid[~_FOV_MASK] = COST_FOV_BOUNDARY

    # Assume everything within EGO_DRIVABLE_RADIUS_M of ego is drivable
    # (camera can't see the ground directly around the user)
    grid[_EGO_DRIVABLE_MASK] = COST_FREE

    # Rasterize drivable area (lowest priority cost, written first)
    if drivable_pts is not None and len(drivable_pts) > 0:
        row, col, valid = _world_to_grid(drivable_pts[:, 0], drivable_pts[:, 1])
        grid[row[valid], col[valid]] = COST_FREE

    # Rasterize lane lines (medium priority)
    if lane_pts is not None and len(lane_pts) > 0:
        row, col, valid = _world_to_grid(lane_pts[:, 0], lane_pts[:, 1])
        grid[row[valid], col[valid]] = COST_LANE

    # Rasterize obstacles (highest priority, written last to override)
    if det_groups is not None:
        for group_pts in det_groups:
            if len(group_pts) < 2:
                continue
            # Fill the bounding rectangle of this detection on the grid
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
                grid[r0:r1+1, c0:c1+1] = COST_LETHAL
    elif det_pts is not None and len(det_pts) > 0:
        row, col, valid = _world_to_grid(det_pts[:, 0], det_pts[:, 1])
        grid[row[valid], col[valid]] = COST_LETHAL

    return grid


def _make_foxglove_grid(grid: np.ndarray, timestamp: float) -> Grid:
    """Create a Foxglove Grid message from a costmap array."""
    dt = datetime.datetime.fromtimestamp(timestamp, tz=datetime.timezone.utc)

    # Convert int8 to uint8 for transmission: map -1 → 255 (Foxglove convention)
    data = grid.copy().astype(np.uint8)  # -1 wraps to 255 naturally

    return Grid(
        timestamp=Timestamp.from_datetime(dt),
        frame_id="base_link",
        pose=Pose(
            position=Vector3(x=GRID_ORIGIN, y=GRID_ORIGIN, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        column_count=GRID_CELLS,
        cell_size=Vector2(x=GRID_RESOLUTION, y=GRID_RESOLUTION),
        row_stride=GRID_CELLS,
        cell_stride=1,
        fields=[
            PackedElementField(
                name="cost",
                offset=0,
                type=PackedElementFieldNumericType.Uint8,
            ),
        ],
        data=data.tobytes(),
    )


def _make_compressed_image(
    image: np.ndarray, timestamp: float, frame_id: str = "costmap"
) -> CompressedImage:
    _, buf = cv2.imencode(".png", image)
    dt = datetime.datetime.fromtimestamp(timestamp, tz=datetime.timezone.utc)
    return CompressedImage(
        timestamp=Timestamp.from_datetime(dt),
        frame_id=frame_id,
        data=buf.tobytes(),
        format="png",
    )


def process_bag(
    bag_path: str,
    device: str | None = None,
    conf: float = 0.3,
    max_frames: int | None = None,
    subsample_drivable: int = 4,
    subsample_lane: int = 2,
    subsample_bbox: int = 8,
    playback_rate: float = 1.0,
) -> None:
    """Process a bag file, build costmaps, and publish via Foxglove WebSocket."""

    pipeline = PerceptionPipeline(
        device=device,
        conf=conf,
        subsample_drivable=subsample_drivable,
        subsample_lane=subsample_lane,
        subsample_bbox=subsample_bbox,
    )
    print(f"YOLOv8 loaded (device={pipeline.device})")

    listener = _Listener()
    server = foxglove.start_server(server_listener=listener)
    print("Foxglove server started on ws://localhost:8765")
    print("Open Foxglove Studio and connect to ws://localhost:8765")

    source = BagSource(bag_path, playback_rate=playback_rate, max_frames=max_frames)
    streams = source.open()
    frame_num = 0

    for frame in streams.frames:
        t0 = time.perf_counter()

        result = pipeline.process_frame(frame)

        # Publish point clouds
        if result.drivable_pts is not None:
            data = pack_points_rgba_vectorized(result.drivable_pts, 0.0, 1.0, 0.0)
            foxglove.log("/perception/pointcloud/drivable", make_pointcloud(data, frame.timestamp))

        if result.lane_pts is not None:
            data = pack_points_rgba_vectorized(result.lane_pts, 1.0, 0.0, 0.0)
            foxglove.log("/perception/pointcloud/lanes", make_pointcloud(data, frame.timestamp))

        if result.det_pts is not None:
            data = pack_points_rgba_vectorized(result.det_pts, 1.0, 1.0, 0.0)
            foxglove.log("/perception/pointcloud/detections", make_pointcloud(data, frame.timestamp))

        # Build and publish costmap
        grid = build_costmap(
            result.drivable_pts, result.lane_pts, result.det_pts,
            det_groups=result.det_groups or None,
        )

        foxglove.log("/costmap/occupancy_grid", _make_foxglove_grid(grid, frame.timestamp))

        color_img = costmap_to_upscaled_image(grid, scale=4)
        foxglove.log("/costmap/image", _make_compressed_image(color_img, frame.timestamp))

        dt = time.perf_counter() - t0
        frame_num += 1
        if frame_num % 5 == 0:
            print(
                f"\rFrame {frame_num}: {1/dt:.1f} FPS, {len(result.detections)} detections",
                end="",
                flush=True,
            )

    print(f"\nDone. Processed {frame_num} frames.")
    print("Press Ctrl+C to stop the server.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        server.stop()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate Nav2-compatible costmaps from perception on MCAP bags"
    )
    parser.add_argument("--bag", required=True, help="Path to .mcap bag file")
    parser.add_argument("--device", default=None, help="Torch device (cuda, mps, cpu)")
    parser.add_argument("--conf", type=float, default=0.3, help="YOLOv8 confidence threshold")
    parser.add_argument("--max-frames", type=int, default=None, help="Process at most N frames")
    parser.add_argument(
        "--subsample-drivable", type=int, default=4,
        help="Subsample factor for drivable area points",
    )
    parser.add_argument(
        "--subsample-lane", type=int, default=2,
        help="Subsample factor for lane line points",
    )
    parser.add_argument(
        "--subsample-bbox", type=int, default=8,
        help="Subsample factor for bounding box points",
    )
    parser.add_argument(
        "--playback-rate", type=float, default=1.0,
        help="Playback speed multiplier (0 = as fast as possible)",
    )
    args = parser.parse_args()

    process_bag(
        bag_path=args.bag,
        device=args.device,
        conf=args.conf,
        max_frames=args.max_frames,
        subsample_drivable=args.subsample_drivable,
        subsample_lane=args.subsample_lane,
        subsample_bbox=args.subsample_bbox,
        playback_rate=args.playback_rate,
    )


if __name__ == "__main__":
    main()
