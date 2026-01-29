from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from mcap.reader import make_reader

from .cdr import CdrReader


@dataclass(slots=True)
class ImuSample:
    t: float
    qx: float
    qy: float
    qz: float
    qw: float
    wx: float
    wy: float
    wz: float
    ax: float
    ay: float
    az: float
    orientation_cov: list[float]
    angular_vel_cov: list[float]
    linear_acc_cov: list[float]


@dataclass(slots=True)
class GpsFix:
    t: float
    latitude: float
    longitude: float
    altitude: float
    position_covariance: list[float]
    covariance_type: int
    status: int


@dataclass(slots=True)
class GpsVel:
    t: float
    vx: float
    vy: float
    vz: float


@dataclass(slots=True)
class Heading:
    t: float
    yaw: float


def _parse_stamp(r: CdrReader) -> float:
    sec = r.int32()
    nsec = r.uint32()
    return sec + nsec * 1e-9


def _parse_imu(data: bytes) -> ImuSample:
    r = CdrReader(data)
    t = _parse_stamp(r)
    _ = r.string()  # frame_id
    qx, qy, qz, qw = r.float64(), r.float64(), r.float64(), r.float64()
    orientation_cov = r.float64_array(9)
    wx, wy, wz = r.float64(), r.float64(), r.float64()
    angular_vel_cov = r.float64_array(9)
    ax, ay, az = r.float64(), r.float64(), r.float64()
    linear_acc_cov = r.float64_array(9)
    return ImuSample(
        t=t, qx=qx, qy=qy, qz=qz, qw=qw,
        wx=wx, wy=wy, wz=wz, ax=ax, ay=ay, az=az,
        orientation_cov=orientation_cov,
        angular_vel_cov=angular_vel_cov,
        linear_acc_cov=linear_acc_cov,
    )


def _parse_gps(data: bytes) -> GpsFix:
    r = CdrReader(data)
    t = _parse_stamp(r)
    _ = r.string()  # frame_id
    status = r.int8()
    _ = r.uint16()  # service
    lat = r.float64()
    lon = r.float64()
    alt = r.float64()
    cov = r.float64_array(9)
    cov_type = r.uint8()
    return GpsFix(
        t=t, latitude=lat, longitude=lon, altitude=alt,
        position_covariance=cov, covariance_type=cov_type, status=status,
    )


def _parse_gps_vel(data: bytes) -> GpsVel:
    r = CdrReader(data)
    t = _parse_stamp(r)
    _ = r.string()  # frame_id
    vx = r.float64()
    vy = r.float64()
    vz = r.float64()
    # Skip angular
    _ = r.float64()
    _ = r.float64()
    _ = r.float64()
    return GpsVel(t=t, vx=vx, vy=vy, vz=vz)


def _parse_heading(data: bytes, log_time: int) -> Heading:
    r = CdrReader(data)
    yaw = r.float64()
    return Heading(t=log_time * 1e-9, yaw=yaw)


def read_bag(path: str | Path) -> tuple[list[ImuSample], list[GpsFix], list[GpsVel], list[Heading]]:
    """Read all IMU, GPS, GPS Velocity, and Heading messages from an MCAP file."""
    imu_samples: list[ImuSample] = []
    gps_fixes: list[GpsFix] = []
    gps_vels: list[GpsVel] = []
    headings: list[Heading] = []

    with open(path, "rb") as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages():
            if channel is None:
                continue
            topic = channel.topic
            if topic.endswith("/imu"):
                imu_samples.append(_parse_imu(message.data))
            elif topic.endswith("/gps/fix"):
                gps_fixes.append(_parse_gps(message.data))
            elif topic.endswith("/gps/velocity"):
                gps_vels.append(_parse_gps_vel(message.data))
            elif topic.endswith("/heading"):
                headings.append(_parse_heading(message.data, message.log_time))

    imu_samples.sort(key=lambda s: s.t)
    gps_fixes.sort(key=lambda s: s.t)
    gps_vels.sort(key=lambda s: s.t)
    headings.sort(key=lambda s: s.t)
    return imu_samples, gps_fixes, gps_vels, headings
