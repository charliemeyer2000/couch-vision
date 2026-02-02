"""Read sensor data from MCAP bag files."""

from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


def read_compressed_images(bag_path: str | Path, topic_suffix: str = "image/compressed") -> Iterator[tuple[float, np.ndarray]]:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    with Path(bag_path).open("rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _schema, channel, message, ros_msg in reader.iter_decoded_messages():
            if not channel.topic.endswith(topic_suffix):
                continue
            if not (hasattr(ros_msg, "format") and hasattr(ros_msg, "data")):
                continue
            buf = np.frombuffer(ros_msg.data, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if img is not None:
                yield message.log_time / 1e9, img


@dataclass
class CameraIntrinsics:
    """Camera intrinsic parameters from sensor_msgs/CameraInfo."""
    width: int
    height: int
    K: np.ndarray  # 3x3 intrinsic matrix
    D: np.ndarray  # distortion coefficients
    distortion_model: str
    frame_id: str


@dataclass
class GpsFix:
    """A GPS fix from the bag."""
    timestamp: float
    latitude: float
    longitude: float
    altitude: float
    position_covariance: list[float]


@dataclass
class ImuSample:
    """An IMU sample from the bag."""
    timestamp: float
    orientation: np.ndarray  # quaternion (x, y, z, w)
    accel: np.ndarray  # (ax, ay, az)
    gyro: np.ndarray  # (wx, wy, wz)


@dataclass
class OdomSample:
    """An odometry sample (ARKit VIO)."""
    timestamp: float
    position: np.ndarray      # (3,) xyz in ARKit world frame
    orientation: np.ndarray   # (4,) quaternion (x, y, z, w)
    pose_covariance: np.ndarray  # (6, 6)


@dataclass
class SyncedFrame:
    """A time-synchronized set of image, depth, camera intrinsics, and IMU orientation."""
    timestamp: float
    image: np.ndarray
    depth: np.ndarray
    intrinsics: CameraIntrinsics
    orientation: np.ndarray | None = None  # quaternion (x, y, z, w) from IMU


def read_synced_frames(
    bag_path: str | Path,
    image_suffix: str = "image/compressed",
    depth_suffix: str = "depth/image",
    camera_info_suffix: str = "camera_info",
    imu_suffix: str = "imu",
    max_time_diff: float = 0.1,
) -> Iterator[SyncedFrame]:
    """Read time-synchronized compressed images, depth maps, camera info, and IMU.

    Yields SyncedFrame tuples where each compressed image is matched to
    the nearest depth image and IMU orientation within max_time_diff seconds.
    Camera intrinsics are read once (assumed static).
    """
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    images: list[tuple[float, np.ndarray]] = []
    depths: list[tuple[float, np.ndarray]] = []
    imu_orientations: list[tuple[float, np.ndarray]] = []
    intrinsics: CameraIntrinsics | None = None

    with Path(bag_path).open("rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _schema, channel, message, ros_msg in reader.iter_decoded_messages():
            ts = message.log_time / 1e9

            if channel.topic.endswith(camera_info_suffix) and intrinsics is None:
                intrinsics = CameraIntrinsics(
                    width=ros_msg.width,
                    height=ros_msg.height,
                    K=np.array(ros_msg.k, dtype=np.float64).reshape(3, 3),
                    D=np.array(ros_msg.d, dtype=np.float64),
                    distortion_model=ros_msg.distortion_model,
                    frame_id=ros_msg.header.frame_id,
                )

            elif channel.topic.endswith(image_suffix):
                if hasattr(ros_msg, "format") and hasattr(ros_msg, "data"):
                    buf = np.frombuffer(ros_msg.data, dtype=np.uint8)
                    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                    if img is not None:
                        images.append((ts, img))

            elif channel.topic.endswith(depth_suffix):
                if ros_msg.encoding == "32FC1":
                    depth = np.frombuffer(ros_msg.data, dtype=np.float32).reshape(
                        ros_msg.height, ros_msg.width
                    )
                elif ros_msg.encoding == "16UC1":
                    depth = np.frombuffer(ros_msg.data, dtype=np.uint16).reshape(
                        ros_msg.height, ros_msg.width
                    ).astype(np.float32) / 1000.0  # mm to meters
                else:
                    continue
                depths.append((ts, depth))

            elif channel.topic.endswith(imu_suffix) and hasattr(ros_msg, "orientation"):
                o = ros_msg.orientation
                quat = np.array([o.x, o.y, o.z, o.w], dtype=np.float64)
                imu_orientations.append((ts, quat))

    if intrinsics is None:
        raise ValueError(f"No CameraInfo found matching suffix '{camera_info_suffix}'")

    # Match each image to nearest depth and IMU orientation by timestamp
    depth_idx = 0
    imu_idx = 0
    for img_ts, img in images:
        # Advance depth index to closest match
        while depth_idx < len(depths) - 1 and abs(depths[depth_idx + 1][0] - img_ts) < abs(depths[depth_idx][0] - img_ts):
            depth_idx += 1
        if depth_idx >= len(depths) or abs(depths[depth_idx][0] - img_ts) > max_time_diff:
            continue

        # Advance IMU index to closest match
        orientation = None
        if imu_orientations:
            while imu_idx < len(imu_orientations) - 1 and abs(imu_orientations[imu_idx + 1][0] - img_ts) < abs(imu_orientations[imu_idx][0] - img_ts):
                imu_idx += 1
            if abs(imu_orientations[imu_idx][0] - img_ts) <= max_time_diff:
                orientation = imu_orientations[imu_idx][1]

        yield SyncedFrame(
            timestamp=img_ts,
            image=img,
            depth=depths[depth_idx][1],
            intrinsics=intrinsics,
            orientation=orientation,
        )


def read_gps_imu_and_odom(
    bag_path: str | Path,
    gps_suffix: str = "gps/fix",
    imu_suffix: str = "imu",
    odom_suffix: str = "odom",
) -> tuple[list[GpsFix], list[ImuSample], list[OdomSample]]:
    """Read all GPS fixes, IMU samples, and odometry from an MCAP bag.

    Returns (gps_fixes, imu_samples, odom_samples) sorted by timestamp.
    """
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    gps_fixes: list[GpsFix] = []
    imu_samples: list[ImuSample] = []
    odom_samples: list[OdomSample] = []

    with Path(bag_path).open("rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _schema, channel, message, ros_msg in reader.iter_decoded_messages():
            ts = message.log_time / 1e9

            if channel.topic.endswith(gps_suffix):
                gps_fixes.append(GpsFix(
                    timestamp=ts,
                    latitude=ros_msg.latitude,
                    longitude=ros_msg.longitude,
                    altitude=ros_msg.altitude,
                    position_covariance=list(ros_msg.position_covariance),
                ))

            elif channel.topic.endswith(imu_suffix) and hasattr(ros_msg, "orientation"):
                o = ros_msg.orientation
                imu_samples.append(ImuSample(
                    timestamp=ts,
                    orientation=np.array([o.x, o.y, o.z, o.w], dtype=np.float64),
                    accel=np.array([
                        ros_msg.linear_acceleration.x,
                        ros_msg.linear_acceleration.y,
                        ros_msg.linear_acceleration.z,
                    ], dtype=np.float64),
                    gyro=np.array([
                        ros_msg.angular_velocity.x,
                        ros_msg.angular_velocity.y,
                        ros_msg.angular_velocity.z,
                    ], dtype=np.float64),
                ))

            elif channel.topic.endswith(odom_suffix) and hasattr(ros_msg, "pose"):
                p = ros_msg.pose.pose.position
                q = ros_msg.pose.pose.orientation
                odom_samples.append(OdomSample(
                    timestamp=ts,
                    position=np.array([p.x, p.y, p.z], dtype=np.float64),
                    orientation=np.array([q.x, q.y, q.z, q.w], dtype=np.float64),
                    pose_covariance=np.array(
                        ros_msg.pose.covariance, dtype=np.float64
                    ).reshape(6, 6),
                ))

    gps_fixes.sort(key=lambda g: g.timestamp)
    imu_samples.sort(key=lambda s: s.timestamp)
    odom_samples.sort(key=lambda o: o.timestamp)
    return gps_fixes, imu_samples, odom_samples
