"""CLI entry point: project perception results into 3D point clouds via depth + camera intrinsics.

Reads an MCAP bag with compressed images, depth images, and camera_info.
Runs YOLOv8 + YOLOP perception, then projects drivable area, lane lines,
and bounding box regions into three separate Foxglove PointCloud messages
served over a WebSocket for visualization.
"""

import argparse
import datetime
import logging
import struct
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

from couch_perception.bag_reader import read_synced_frames, SyncedFrame
from couch_perception.camera_model import make_camera_model, CameraModel
from couch_perception.yolov8_detector import YOLOv8Detector, Detection
from couch_perception.yolop_detector import YOLOPDetector, YOLOPResult


def _quat_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Convert quaternion (x, y, z, w) to a 3x3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def _apply_imu_rotation(points: np.ndarray, orientation: np.ndarray | None) -> np.ndarray:
    """Rotate camera-frame points into a gravity-aligned world frame using IMU orientation.

    TEMPORARY HACK: Using the inverse of the IMU orientation quaternion to rotate
    camera-frame points into a gravity-aligned frame, then permuting axes so that
    X/Y span the ground plane and Z points up. This will be replaced by a proper
    URDF of the car with correct camera-to-body and body-to-world transforms once
    the vehicle model is defined.
    """
    if orientation is None or len(points) == 0:
        return points
    R = _quat_to_rotation_matrix(orientation)
    # R.T = inverse rotation (world-to-device → device-to-world)
    pts_world = (R.T @ points.T).T
    # The IMU world frame has Y as the gravity-normal axis. Permute so Z is up:
    # new_x = x, new_y = z, new_z = -y
    result = np.empty_like(pts_world)
    result[:, 0] = pts_world[:, 0]
    result[:, 1] = pts_world[:, 2]
    result[:, 2] = -pts_world[:, 1]
    return result.astype(np.float32)

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
        logging.info(f"Client {client.id} subscribed to {channel.topic}")

    def on_unsubscribe(self, client: Client, channel: ChannelView) -> None:
        if client.id in self.subscribers:
            self.subscribers[client.id].discard(channel.topic)
            if not self.subscribers[client.id]:
                del self.subscribers[client.id]


def _pack_points_rgba(
    points: np.ndarray, r: float, g: float, b: float, a: float = 1.0
) -> bytes:
    """Pack (N,3) float32 points with a constant RGBA color into PointCloud data bytes."""
    if len(points) == 0:
        return b""
    n = len(points)
    buf = bytearray(n * POINT_STRIDE)
    color = struct.pack("<ffff", r, g, b, a)
    for i in range(n):
        offset = i * POINT_STRIDE
        struct.pack_into("<fff", buf, offset, points[i, 0], points[i, 1], points[i, 2])
        buf[offset + 12 : offset + 28] = color
    return bytes(buf)


def _pack_points_rgba_vectorized(
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


def _extract_mask_pixels(
    mask: np.ndarray, depth: np.ndarray, subsample: int = 4
) -> tuple[np.ndarray, np.ndarray]:
    """Get pixel coordinates and depth values where a binary mask is active.

    The mask and depth may differ in resolution; depth is the authority for
    the coordinate space. The mask is resized to match depth dimensions.

    Args:
        mask: 2D binary mask (any resolution).
        depth: 2D float32 depth image (target resolution).
        subsample: Take every Nth pixel to reduce point count.

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


def _extract_bbox_pixels(
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


def _make_pointcloud(
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


def _build_depth_camera_model(
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
    K_scaled[0, :] *= sx  # fx, cx
    K_scaled[1, :] *= sy  # fy, cy

    scaled_intrinsics = CameraIntrinsics(
        width=dw,
        height=dh,
        K=K_scaled,
        D=orig.D,
        distortion_model=orig.distortion_model,
        frame_id=orig.frame_id,
    )
    return make_camera_model(scaled_intrinsics)


def _validate_ground_plane(pts: np.ndarray, label: str) -> None:
    """Assert that X/Y variance exceeds Z variance, confirming the ground plane is in XY."""
    if len(pts) < 10:
        return
    var_x, var_y, var_z = pts[:, 0].var(), pts[:, 1].var(), pts[:, 2].var()
    if var_z >= var_x or var_z >= var_y:
        logging.warning(
            f"{label}: ground plane check FAILED — var(x)={var_x:.3f} var(y)={var_y:.3f} var(z)={var_z:.3f}. "
            f"Expected var(z) < var(x) and var(z) < var(y)."
        )


# BEV occupancy grid: 100x100 px image.
# World X = forward (depth direction), Y = lateral.
# Image: row 0 = max forward distance (top), col 0 = leftmost lateral.
_BEV_SIZE = 100
# Ranges are computed per-frame from the actual point extents with a small margin.


def _points_to_occupancy_image(
    pts: np.ndarray,
    color: tuple[int, int, int],
    size: int = _BEV_SIZE,
) -> np.ndarray:
    """Project XY points onto a BEV occupancy grid image, discarding Z.

    The grid bounds are fit to the actual point extents so that points fill
    the image. World X (forward) maps to image rows (top = far), world Y
    (lateral) maps to image columns (left = left).

    Returns a BGR image (size, size, 3).
    """
    grid = np.zeros((size, size, 3), dtype=np.uint8)

    if len(pts) == 0:
        return grid

    x = pts[:, 0]  # forward
    y = pts[:, 1]  # lateral

    x_min, x_max = float(x.min()), float(x.max())
    y_min, y_max = float(y.min()), float(y.max())

    # Add small margin so edge points aren't clipped
    margin = 0.2
    x_min -= margin
    x_max += margin
    y_min -= margin
    y_max += margin

    x_span = x_max - x_min if x_max > x_min else 1.0
    y_span = y_max - y_min if y_max > y_min else 1.0

    # Map to pixel coords: row = forward (flipped so far=top), col = lateral
    col = ((y - y_min) / y_span * (size - 1)).astype(np.int32)
    row = ((x_max - x) / x_span * (size - 1)).astype(np.int32)  # flip so forward is up

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

        # Build camera model once
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
                pts_3d = _apply_imu_rotation(pts_3d, frame.orientation)
                if frame_num == 0:
                    _validate_ground_plane(pts_3d, "drivable")
                drivable_pts = pts_3d
                data = _pack_points_rgba_vectorized(pts_3d, 0.0, 1.0, 0.0)  # green
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
                pts_3d = _apply_imu_rotation(pts_3d, frame.orientation)
                if frame_num == 0:
                    _validate_ground_plane(pts_3d, "lanes")
                lane_pts = pts_3d
                data = _pack_points_rgba_vectorized(pts_3d, 1.0, 0.0, 0.0)  # red
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
                pts_3d = _apply_imu_rotation(pts_3d, frame.orientation)
                if frame_num == 0:
                    _validate_ground_plane(pts_3d, "detections")
                det_pts = pts_3d
                data = _pack_points_rgba_vectorized(pts_3d, 1.0, 1.0, 0.0)  # yellow
                foxglove.log(
                    "/perception/pointcloud/detections",
                    _make_pointcloud(data, frame.timestamp),
                )

        # Publish BEV occupancy grid images (XY plane, Z discarded)
        bev_drivable = _points_to_occupancy_image(
            drivable_pts if drivable_pts is not None else np.empty((0, 3)),
            color=(0, 255, 0),
        )
        foxglove.log(
            "/perception/bev/drivable",
            _make_compressed_image(bev_drivable, frame.timestamp),
        )

        bev_lanes = _points_to_occupancy_image(
            lane_pts if lane_pts is not None else np.empty((0, 3)),
            color=(0, 0, 255),
        )
        foxglove.log(
            "/perception/bev/lanes",
            _make_compressed_image(bev_lanes, frame.timestamp),
        )

        bev_detections = _points_to_occupancy_image(
            det_pts if det_pts is not None else np.empty((0, 3)),
            color=(0, 255, 255),
        )
        foxglove.log(
            "/perception/bev/detections",
            _make_compressed_image(bev_detections, frame.timestamp),
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
