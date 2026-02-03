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


def read_all_streams(
    bag_path: str | Path,
    image_suffix: str = "image/compressed",
    depth_suffix: str = "depth/image",
    camera_info_suffix: str = "camera_info",
    imu_suffix: str = "imu",
    gps_suffix: str = "gps/fix",
    odom_suffix: str = "odom",
    max_time_diff: float = 0.1,
) -> tuple[Iterator[SyncedFrame], list[GpsFix], list[ImuSample], list[OdomSample]]:
    """Single-pass streaming bag reader. Reads GPS/IMU/odom (small) into memory,
    then yields synced image+depth frames on-the-fly without buffering all images.

    Returns (frames_iterator, gps_fixes, imu_samples, odom_samples).
    GPS/IMU/odom are fully loaded; frames are streamed lazily.
    """
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    # First pass: read only lightweight scalar topics (GPS, IMU, odom, camera_info).
    # Skip images/depth to avoid OOM.
    gps_fixes: list[GpsFix] = []
    imu_samples: list[ImuSample] = []
    odom_samples: list[OdomSample] = []
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

            elif channel.topic.endswith(gps_suffix):
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

    if intrinsics is None:
        raise ValueError(f"No CameraInfo found matching suffix '{camera_info_suffix}'")

    # Build IMU timestamp index for fast nearest-neighbor lookup
    imu_times = np.array([s.timestamp for s in imu_samples]) if imu_samples else np.array([])

    def _stream_frames() -> Iterator[SyncedFrame]:
        """Second pass: stream image+depth, sync on-the-fly, yield one frame at a time."""
        pending_depth: tuple[float, np.ndarray] | None = None
        imu_idx = 0

        with Path(bag_path).open("rb") as f2:
            reader2 = make_reader(f2, decoder_factories=[DecoderFactory()])
            for _schema, channel, message, ros_msg in reader2.iter_decoded_messages():
                ts = message.log_time / 1e9

                if channel.topic.endswith(depth_suffix):
                    if ros_msg.encoding == "32FC1":
                        depth = np.frombuffer(ros_msg.data, dtype=np.float32).reshape(
                            ros_msg.height, ros_msg.width
                        )
                    elif ros_msg.encoding == "16UC1":
                        depth = np.frombuffer(ros_msg.data, dtype=np.uint16).reshape(
                            ros_msg.height, ros_msg.width
                        ).astype(np.float32) / 1000.0
                    else:
                        continue
                    pending_depth = (ts, depth)

                elif channel.topic.endswith(image_suffix):
                    if not (hasattr(ros_msg, "format") and hasattr(ros_msg, "data")):
                        continue
                    buf = np.frombuffer(ros_msg.data, dtype=np.uint8)
                    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                    if img is None:
                        continue

                    # Match to nearest depth
                    if pending_depth is None:
                        continue
                    depth_ts, depth = pending_depth
                    if abs(depth_ts - ts) > max_time_diff:
                        continue

                    # Match to nearest IMU orientation
                    orientation = None
                    if len(imu_times) > 0:
                        while imu_idx < len(imu_times) - 1 and imu_times[imu_idx] < ts:
                            imu_idx += 1
                        if abs(imu_times[imu_idx] - ts) <= max_time_diff:
                            orientation = imu_samples[imu_idx].orientation

                    yield SyncedFrame(
                        timestamp=ts,
                        image=img,
                        depth=depth,
                        intrinsics=intrinsics,
                        orientation=orientation,
                    )

    return _stream_frames(), gps_fixes, imu_samples, odom_samples


# Legacy API wrappers for backward compatibility
def read_synced_frames(
    bag_path: str | Path,
    image_suffix: str = "image/compressed",
    depth_suffix: str = "depth/image",
    camera_info_suffix: str = "camera_info",
    imu_suffix: str = "imu",
    max_time_diff: float = 0.1,
) -> Iterator[SyncedFrame]:
    frames, _, _, _ = read_all_streams(
        bag_path, image_suffix, depth_suffix, camera_info_suffix, imu_suffix,
        max_time_diff=max_time_diff,
    )
    return frames


def read_gps_imu_and_odom(
    bag_path: str | Path,
    gps_suffix: str = "gps/fix",
    imu_suffix: str = "imu",
    odom_suffix: str = "odom",
) -> tuple[list[GpsFix], list[ImuSample], list[OdomSample]]:
    _, gps, imu, odom = read_all_streams(
        bag_path, gps_suffix=gps_suffix, imu_suffix=imu_suffix, odom_suffix=odom_suffix,
    )
    return gps, imu, odom
