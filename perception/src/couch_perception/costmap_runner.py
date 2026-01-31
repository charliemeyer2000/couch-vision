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
import struct
import time

import cv2
import numpy as np

import foxglove
from foxglove.schemas import (
    CompressedImage,
    Grid,
    PackedElementField,
    PackedElementFieldNumericType,
    PointCloud,
    Pose,
    Quaternion,
    Timestamp,
    Vector2,
    Vector3,
)
from foxglove.websocket import ServerListener, Client, ChannelView

from couch_perception.bag_reader import read_synced_frames, SyncedFrame
from couch_perception.camera_model import make_camera_model, CameraModel
from couch_perception.yolov8_detector import YOLOv8Detector, Detection
from couch_perception.yolop_detector import YOLOPDetector, YOLOPResult
from couch_perception.costmap_visualizer import (
    COST_UNKNOWN,
    COST_FREE,
    COST_LANE,
    COST_FOV_BOUNDARY,
    COST_LETHAL,
    costmap_to_upscaled_image,
)

# Reuse projection helpers from bev_projection_runner
from couch_perception.bev_projection_runner import (
    _quat_to_rotation_matrix,
    _apply_imu_rotation,
    _extract_mask_pixels,
    _extract_bbox_pixels,
    _build_depth_camera_model,
    _pack_points_rgba_vectorized,
    _make_pointcloud,
    POINT_STRIDE,
    POINTCLOUD_FIELDS,
)

logger = logging.getLogger(__name__)

# Grid parameters
GRID_SIZE_M = 10.0      # 10m x 10m
GRID_RESOLUTION = 0.1   # 10cm per cell
GRID_CELLS = int(GRID_SIZE_M / GRID_RESOLUTION)  # 100
GRID_ORIGIN = -GRID_SIZE_M / 2.0  # -5.0m (grid centered at 0,0)

# FOV parameters
FOV_DEG = 70.0
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
    """Pre-compute a boolean mask where True = inside 70° FOV, False = outside.

    The ego is at grid center. Forward direction is +X (row increases = further forward).
    Grid layout: row 0 = max forward (top), row 99 = behind. Col 0 = left, col 99 = right.
    World coords: x = forward, y = lateral (left positive).
    """
    mask = np.zeros((GRID_CELLS, GRID_CELLS), dtype=bool)

    for r in range(GRID_CELLS):
        for c in range(GRID_CELLS):
            # World coords relative to ego at center
            x = GRID_ORIGIN + (GRID_CELLS - 1 - r) * GRID_RESOLUTION + GRID_RESOLUTION / 2
            y = GRID_ORIGIN + c * GRID_RESOLUTION + GRID_RESOLUTION / 2

            # Only forward hemisphere and within FOV angle
            if x > 0:
                angle = abs(math.atan2(y, x))
                if angle <= FOV_HALF_RAD:
                    mask[r, c] = True

    return mask


# Pre-compute once at import time
_FOV_MASK = _build_fov_mask()


def _world_to_grid(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Convert world coordinates (meters, ego at 0,0) to grid row/col indices.

    Row 0 = max forward, row GRID_CELLS-1 = max behind.
    Col 0 = max left, col GRID_CELLS-1 = max right.
    """
    col = ((y - GRID_ORIGIN) / GRID_RESOLUTION).astype(np.int32)
    row = (GRID_CELLS - 1 - ((x - GRID_ORIGIN) / GRID_RESOLUTION)).astype(np.int32)

    valid = (row >= 0) & (row < GRID_CELLS) & (col >= 0) & (col < GRID_CELLS)
    return row, col, valid


def build_costmap(
    drivable_pts: np.ndarray | None,
    lane_pts: np.ndarray | None,
    det_pts: np.ndarray | None,
) -> np.ndarray:
    """Build a 100x100 costmap from projected 3D points.

    Points are in ego-centric world frame (x=forward, y=lateral, z=up).
    Grid is centered at (0,0).

    Returns:
        (GRID_CELLS, GRID_CELLS) int8 array with Nav2-compatible cost values.
    """
    grid = np.full((GRID_CELLS, GRID_CELLS), COST_UNKNOWN, dtype=np.int8)

    # Outside FOV → high cost
    grid[~_FOV_MASK] = COST_FOV_BOUNDARY

    # Rasterize drivable area (lowest priority cost, written first)
    if drivable_pts is not None and len(drivable_pts) > 0:
        row, col, valid = _world_to_grid(drivable_pts[:, 0], drivable_pts[:, 1])
        grid[row[valid], col[valid]] = COST_FREE

    # Rasterize lane lines (medium priority)
    if lane_pts is not None and len(lane_pts) > 0:
        row, col, valid = _world_to_grid(lane_pts[:, 0], lane_pts[:, 1])
        grid[row[valid], col[valid]] = COST_LANE

    # Rasterize obstacles (highest priority, written last to override)
    if det_pts is not None and len(det_pts) > 0:
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

    yolo = YOLOv8Detector(conf_threshold=conf, device=device)
    print(f"YOLOv8 loaded (device={yolo.device})")

    print("Loading YOLOP...")
    yolop = YOLOPDetector(device=device)

    listener = _Listener()
    server = foxglove.start_server(server_listener=listener)
    print("Foxglove server started on ws://localhost:8765")
    print("Open Foxglove Studio and connect to ws://localhost:8765")

    frames = read_synced_frames(bag_path)
    cam_model: CameraModel | None = None
    depth_cam_model: CameraModel | None = None

    frame_num = 0
    prev_wall_time: float | None = None
    prev_bag_time: float | None = None

    for frame in frames:
        if max_frames and frame_num >= max_frames:
            break

        if cam_model is None:
            cam_model = make_camera_model(frame.intrinsics)
            depth_cam_model = _build_depth_camera_model(
                cam_model, frame.depth.shape[:2]
            )

        # Pace playback
        if prev_bag_time is not None and playback_rate > 0:
            bag_dt = frame.timestamp - prev_bag_time
            wall_dt = time.monotonic() - prev_wall_time
            sleep_time = (bag_dt / playback_rate) - wall_dt
            if sleep_time > 0:
                time.sleep(sleep_time)
        prev_bag_time = frame.timestamp
        prev_wall_time = time.monotonic()

        t0 = time.perf_counter()

        # Run perception
        detections = yolo.detect(frame.image)
        yolop_result = yolop.detect(frame.image)

        # Project drivable area
        drivable_pts: np.ndarray | None = None
        if yolop_result and yolop_result.drivable_mask is not None:
            pixels, depths = _extract_mask_pixels(
                yolop_result.drivable_mask, frame.depth, subsample=subsample_drivable
            )
            if len(pixels) > 0:
                pts_3d = depth_cam_model.project_pixels_to_3d(pixels, depths)
                drivable_pts = _apply_imu_rotation(pts_3d, frame.orientation)
                data = _pack_points_rgba_vectorized(drivable_pts, 0.0, 1.0, 0.0)
                foxglove.log(
                    "/perception/pointcloud/drivable",
                    _make_pointcloud(data, frame.timestamp),
                )

        # Project lane lines
        lane_pts: np.ndarray | None = None
        if yolop_result and yolop_result.lane_mask is not None:
            pixels, depths = _extract_mask_pixels(
                yolop_result.lane_mask, frame.depth, subsample=subsample_lane
            )
            if len(pixels) > 0:
                pts_3d = depth_cam_model.project_pixels_to_3d(pixels, depths)
                lane_pts = _apply_imu_rotation(pts_3d, frame.orientation)
                data = _pack_points_rgba_vectorized(lane_pts, 1.0, 0.0, 0.0)
                foxglove.log(
                    "/perception/pointcloud/lanes",
                    _make_pointcloud(data, frame.timestamp),
                )

        # Project bounding boxes
        det_pts: np.ndarray | None = None
        if detections:
            pixels, depths = _extract_bbox_pixels(
                detections, frame.depth, frame.image.shape[:2], subsample=subsample_bbox
            )
            if len(pixels) > 0:
                pts_3d = depth_cam_model.project_pixels_to_3d(pixels, depths)
                det_pts = _apply_imu_rotation(pts_3d, frame.orientation)
                data = _pack_points_rgba_vectorized(det_pts, 1.0, 1.0, 0.0)
                foxglove.log(
                    "/perception/pointcloud/detections",
                    _make_pointcloud(data, frame.timestamp),
                )

        # Build and publish costmap
        grid = build_costmap(drivable_pts, lane_pts, det_pts)

        foxglove.log(
            "/costmap/occupancy_grid",
            _make_foxglove_grid(grid, frame.timestamp),
        )

        color_img = costmap_to_upscaled_image(grid, scale=4)
        foxglove.log(
            "/costmap/image",
            _make_compressed_image(color_img, frame.timestamp),
        )

        dt = time.perf_counter() - t0
        frame_num += 1
        if frame_num % 5 == 0:
            n_det = len(detections)
            print(
                f"\rFrame {frame_num}: {1/dt:.1f} FPS, {n_det} detections",
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
