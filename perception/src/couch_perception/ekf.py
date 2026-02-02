from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

# State indices
POS = slice(0, 3)
VEL = slice(3, 6)
ORI = slice(6, 9)   # roll, pitch, yaw (rad)
BA = slice(9, 12)    # accelerometer bias
BG = slice(12, 15)   # gyro bias

GRAVITY = np.array([0.0, 0.0, -9.81])
STATE_DIM = 15


def _wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class EKF:
    def __init__(
        self,
        accel_noise: float = 2.0,
        gyro_noise: float = 0.05,
        accel_bias_noise: float = 1e-3,
        gyro_bias_noise: float = 1e-4,
        initial_pos_cov: float = 1.0,
        heading_noise: float = 0.15,
        gps_heading_noise: float = 0.3,
        velocity_damping: float = 0.95,
        use_imu: bool = True,
        force_2d: bool = False,
        max_speed: float = 3.0,
    ) -> None:
        self.x: NDArray[np.float64] = np.zeros(STATE_DIM)
        self.P: NDArray[np.float64] = np.eye(STATE_DIM)
        self.P[POS, POS] *= initial_pos_cov
        self.P[VEL, VEL] *= 10.0
        self.P[ORI, ORI] *= 1.0
        self.P[BA, BA] *= 1.0
        self.P[BG, BG] *= 0.1

        self.accel_noise = accel_noise
        self.gyro_noise = gyro_noise
        self.accel_bias_noise = accel_bias_noise
        self.gyro_bias_noise = gyro_bias_noise
        self.initial_pos_cov = initial_pos_cov
        self.heading_noise = heading_noise
        self.gps_heading_noise = gps_heading_noise
        self.velocity_damping = velocity_damping
        self.use_imu = use_imu
        self.force_2d = force_2d
        self.max_speed = max_speed
        self._initialized = False

    def initialize(
        self,
        position_enu: NDArray[np.float64],
        orientation_quat: NDArray[np.float64],
    ) -> None:
        self.x[POS] = position_enu
        self.x[ORI] = Rotation.from_quat(orientation_quat).as_euler("xyz")
        self._initialized = True

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def yaw(self) -> float:
        return float(self.x[8])

    def predict(
        self,
        accel: NDArray[np.float64],
        gyro: NDArray[np.float64],
        orientation_quat: NDArray[np.float64],
        dt: float,
    ) -> None:
        if dt <= 0 or dt > 1.0:
            return

        rot = Rotation.from_quat(orientation_quat)
        R = rot.as_matrix()

        # Integrate gyro for orientation prediction (bias-corrected)
        gyro_corrected = gyro - self.x[BG]
        self.x[ORI] += gyro_corrected * dt
        # Wrap yaw
        self.x[8] = _wrap_angle(self.x[8])

        if self.use_imu:
            accel_corrected = accel - self.x[BA]
            accel_enu = R @ accel_corrected - GRAVITY
        else:
            accel_enu = np.zeros(3)

        if self.force_2d:
            accel_enu[2] = 0.0
            self.x[VEL][2] = 0.0

        # Velocity damping — walking doesn't sustain velocity without continuous input
        self.x[VEL] *= self.velocity_damping

        self.x[VEL] += accel_enu * dt
        speed = np.linalg.norm(self.x[VEL])
        if speed > self.max_speed:
            self.x[VEL] *= self.max_speed / speed
        self.x[POS] += self.x[VEL] * dt + 0.5 * accel_enu * dt * dt

        F = np.eye(STATE_DIM)
        F[POS, VEL] = np.eye(3) * dt
        F[3:6, 9:12] = -R * dt
        # Gyro bias affects orientation prediction
        F[6:9, 12:15] = -np.eye(3) * dt

        Q = np.zeros((STATE_DIM, STATE_DIM))
        Q[POS, POS] = np.eye(3) * (self.accel_noise * dt ** 2) ** 2
        Q[VEL, VEL] = np.eye(3) * (self.accel_noise * dt) ** 2
        Q[ORI, ORI] = np.eye(3) * (self.gyro_noise * dt) ** 2
        Q[BA, BA] = np.eye(3) * (self.accel_bias_noise * dt) ** 2
        Q[BG, BG] = np.eye(3) * (self.gyro_bias_noise * dt) ** 2

        if self.force_2d:
            Q[2, 2] = 0.0   # Pos Z
            Q[5, 5] = 0.0   # Vel Z

        self.P = F @ self.P @ F.T + Q

    def update_gps(
        self,
        position_enu: NDArray[np.float64],
        R_cov: NDArray[np.float64],
    ) -> None:
        """GPS position measurement update. R_cov is 3x3 measurement noise."""
        H = np.zeros((3, STATE_DIM))
        H[:3, :3] = np.eye(3)

        y = position_enu - self.x[POS]
        S = H @ self.P @ H.T + R_cov
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x += K @ y
        self.P = (np.eye(STATE_DIM) - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)

    def update_heading(
        self,
        orientation_quat: NDArray[np.float64],
    ) -> None:
        """Fuse IMU/compass heading (yaw from device quaternion) as a measurement.

        The iOS CoreMotion quaternion already fuses magnetometer internally,
        so this acts as a compass heading observation.
        """
        measured_yaw = Rotation.from_quat(orientation_quat).as_euler("xyz")[2]

        H = np.zeros((1, STATE_DIM))
        H[0, 8] = 1.0  # yaw is state index 8

        innovation = _wrap_angle(measured_yaw - self.x[8])
        R = np.array([[self.heading_noise ** 2]])

        S = H @ self.P @ H.T + R
        K = (self.P @ H.T) / S[0, 0]

        self.x += (K * innovation).flatten()
        self.x[8] = _wrap_angle(self.x[8])
        self.P = (np.eye(STATE_DIM) - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)

    def update_gps_heading(
        self,
        prev_enu: NDArray[np.float64],
        curr_enu: NDArray[np.float64],
        min_dist: float = 0.5,
    ) -> None:
        """Derive heading from consecutive GPS fixes (course over ground).

        Only updates if the displacement exceeds min_dist to avoid noisy
        headings from GPS jitter when stationary.
        """
        de = curr_enu[0] - prev_enu[0]
        dn = curr_enu[1] - prev_enu[1]
        dist = math.sqrt(de * de + dn * dn)
        if dist < min_dist:
            return

        gps_yaw = math.atan2(de, dn)  # ENU: yaw = atan2(east, north)

        H = np.zeros((1, STATE_DIM))
        H[0, 8] = 1.0

        innovation = _wrap_angle(gps_yaw - self.x[8])
        R = np.array([[self.gps_heading_noise ** 2]])

        S = H @ self.P @ H.T + R
        K = (self.P @ H.T) / S[0, 0]

        self.x += (K * innovation).flatten()
        self.x[8] = _wrap_angle(self.x[8])
        self.P = (np.eye(STATE_DIM) - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)
