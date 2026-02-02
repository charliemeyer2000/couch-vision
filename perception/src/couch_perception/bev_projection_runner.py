"""CLI entry point: project perception results into 3D point clouds via depth + camera intrinsics.

Reads an MCAP bag with compressed images, depth images, and camera_info.
Runs YOLOv8 + YOLOP perception, then projects drivable area, lane lines,
and bounding box regions into three separate Foxglove PointCloud messages
served over a WebSocket for visualization.
"""

import argparse
import datetime
import time

import cv2
import numpy as np

import foxglove
from foxglove.schemas import (
    CompressedImage,
    PackedElementField,
    PackedElementFieldNumericType,
    PointCloud,
    Timestamp,
)
from foxglove.websocket import ServerListener, Client, ChannelView

from couch_perception.frame_source import BagSource
from couch_perception.perception_pipeline import PerceptionPipeline
from couch_perception.projection import validate_ground_plane

# Point stride: x(f32) + y(f32) + z(f32) + r(f32) + g(f32) + b(f32) + a(f32) = 28 bytes
POINT_STRIDE = 28

POINTCLOUD_FIELDS = [
    PackedElementField(name="x", offset=0, type=PackedElementFieldNumericType.Float32),
    PackedElementField(name="y", offset=4, type=PackedElementFieldNumericType.Float32),
    PackedElementField(name="z", offset=8, type=PackedElementFieldNumericType.Float32),
    PackedElementField(name="red", offset=12, type=PackedElementFieldNumericType.Float32),
    PackedElementField(name="green", offset=16, type=PackedElementFieldNumericType.Float32),
    PackedElementField(name="blue", offset=20, type=PackedElementFieldNumericType.Float32),
    PackedElementField(name="alpha", offset=24, type=PackedElementFieldNumericType.Float32),
]


class _Listener(ServerListener):
    def __init__(self) -> None:
        self.subscribers: dict[int, set[str]] = {}

    def has_subscribers(self) -> bool:
        return len(self.subscribers) > 0

    def on_subscribe(self, client: Client, channel: ChannelView) -> None:
        self.subscribers.setdefault(client.id, set()).add(channel.topic)

    def on_unsubscribe(self, client: Client, channel: ChannelView) -> None:
        if client.id in self.subscribers:
            self.subscribers[client.id].discard(channel.topic)
            if not self.subscribers[client.id]:
                del self.subscribers[client.id]


def pack_points_rgba_vectorized(
    points: np.ndarray, r: float, g: float, b: float, a: float = 1.0
) -> bytes:
    """Vectorized version of point packing for better performance."""
    if len(points) == 0:
        return b""
    n = len(points)
    data = np.empty((n, 7), dtype=np.float32)
    data[:, :3] = points
    data[:, 3] = r
    data[:, 4] = g
    data[:, 5] = b
    data[:, 6] = a
    return data.tobytes()


def make_pointcloud(
    data: bytes, timestamp: float, frame_id: str = "camera"
) -> PointCloud:
    dt = datetime.datetime.fromtimestamp(timestamp, tz=datetime.timezone.utc)
    return PointCloud(
        frame_id=frame_id,
        timestamp=Timestamp.from_datetime(dt),
        point_stride=POINT_STRIDE,
        fields=POINTCLOUD_FIELDS,
        data=data,
    )


# BEV occupancy grid: 100x100 px image.
# World X = forward (depth direction), Y = lateral.
# Image: row 0 = max forward distance (top), col 0 = leftmost lateral.
_BEV_SIZE = 100


def _points_to_occupancy_image(
    pts: np.ndarray,
    color: tuple[int, int, int],
    size: int = _BEV_SIZE,
) -> np.ndarray:
    """Project XY points onto a BEV occupancy grid image, discarding Z."""
    grid = np.zeros((size, size, 3), dtype=np.uint8)
    if len(pts) == 0:
        return grid

    x = pts[:, 0]
    y = pts[:, 1]

    x_min, x_max = float(x.min()), float(x.max())
    y_min, y_max = float(y.min()), float(y.max())

    margin = 0.2
    x_min -= margin
    x_max += margin
    y_min -= margin
    y_max += margin

    x_span = x_max - x_min if x_max > x_min else 1.0
    y_span = y_max - y_min if y_max > y_min else 1.0

    col = ((y - y_min) / y_span * (size - 1)).astype(np.int32)
    row = ((x_max - x) / x_span * (size - 1)).astype(np.int32)

    valid = (col >= 0) & (col < size) & (row >= 0) & (row < size)
    row, col = row[valid], col[valid]
    grid[row, col] = color
    return grid


def _make_compressed_image(
    image: np.ndarray, timestamp: float, frame_id: str = "bev"
) -> CompressedImage:
    """Encode an RGB image as a Foxglove CompressedImage (PNG)."""
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
    """Process a bag file and publish 3D point clouds via Foxglove WebSocket."""

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

        # Validate ground plane on first frame
        if frame_num == 0:
            if result.drivable_pts is not None:
                validate_ground_plane(result.drivable_pts, "drivable")
            if result.lane_pts is not None:
                validate_ground_plane(result.lane_pts, "lanes")
            if result.det_pts is not None:
                validate_ground_plane(result.det_pts, "detections")

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

        # Publish BEV occupancy grid images
        empty = np.empty((0, 3))
        bev_drivable = _points_to_occupancy_image(
            result.drivable_pts if result.drivable_pts is not None else empty,
            color=(0, 255, 0),
        )
        foxglove.log("/perception/bev/drivable", _make_compressed_image(bev_drivable, frame.timestamp))

        bev_lanes = _points_to_occupancy_image(
            result.lane_pts if result.lane_pts is not None else empty,
            color=(0, 0, 255),
        )
        foxglove.log("/perception/bev/lanes", _make_compressed_image(bev_lanes, frame.timestamp))

        bev_detections = _points_to_occupancy_image(
            result.det_pts if result.det_pts is not None else empty,
            color=(0, 255, 255),
        )
        foxglove.log("/perception/bev/detections", _make_compressed_image(bev_detections, frame.timestamp))

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
        description="Project perception results into 3D point clouds from an MCAP bag"
    )
    parser.add_argument("--bag", required=True, help="Path to .mcap bag file")
    parser.add_argument("--device", default=None, help="Torch device (cuda, mps, cpu)")
    parser.add_argument("--conf", type=float, default=0.3, help="YOLOv8 confidence threshold")
    parser.add_argument("--max-frames", type=int, default=None, help="Process at most N frames")
    parser.add_argument(
        "--subsample-drivable", type=int, default=4,
        help="Subsample factor for drivable area points (higher = fewer points)",
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
