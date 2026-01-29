from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

# State indices
POS = slice(0, 3)
VEL = slice(3, 6)
ORI = slice(6, 9)   # roll, pitch, yaw (rad)
BA = slice(9, 12)    # accelerometer bias
BG = slice(12, 15)   # gyro bias

GRAVITY = np.array([0.0, 0.0, 9.81])
STATE_DIM = 15


class EKF:
    def __init__(
        self,
        accel_noise: float = 2.0,
        gyro_noise: float = 0.05,
        accel_bias_noise: float = 1e-3,
        gyro_bias_noise: float = 1e-4,
    ) -> None:
        self.x: NDArray[np.float64] = np.zeros(STATE_DIM)
        self.P: NDArray[np.float64] = np.eye(STATE_DIM)
        self.P[POS, POS] *= 100.0
        self.P[VEL, VEL] *= 10.0
        self.P[ORI, ORI] *= 0.1
        self.P[BA, BA] *= 1.0
        self.P[BG, BG] *= 0.1

        self.accel_noise = accel_noise
        self.gyro_noise = gyro_noise
        self.accel_bias_noise = accel_bias_noise
        self.gyro_bias_noise = gyro_bias_noise
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
        self.x[ORI] = rot.as_euler("xyz")

        accel_corrected = accel - self.x[BA]
        accel_enu = R @ accel_corrected - GRAVITY

        self.x[VEL] += accel_enu * dt
        speed = np.linalg.norm(self.x[VEL])
        max_speed = 50.0
        if speed > max_speed:
            self.x[VEL] *= max_speed / speed
        self.x[POS] += self.x[VEL] * dt + 0.5 * accel_enu * dt * dt

        F = np.eye(STATE_DIM)
        F[POS, VEL] = np.eye(3) * dt
        F[3:6, 9:12] = -R * dt

        Q = np.zeros((STATE_DIM, STATE_DIM))
        Q[POS, POS] = np.eye(3) * (self.accel_noise * dt**2) ** 2
        Q[VEL, VEL] = np.eye(3) * (self.accel_noise * dt) ** 2
        Q[ORI, ORI] = np.eye(3) * (self.gyro_noise * dt) ** 2
        Q[BA, BA] = np.eye(3) * (self.accel_bias_noise * dt) ** 2
        Q[BG, BG] = np.eye(3) * (self.gyro_bias_noise * dt) ** 2

        self.P = F @ self.P @ F.T + Q

    def update_gps(
        self,
        position_enu: NDArray[np.float64],
        R_cov: NDArray[np.float64],
    ) -> None:
        """GPS measurement update. R_cov is 3x3 measurement noise."""
        H = np.zeros((3, STATE_DIM))
        H[:3, :3] = np.eye(3)

        y = position_enu - self.x[POS]
        S = H @ self.P @ H.T + R_cov
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x += K @ y
        self.P = (np.eye(STATE_DIM) - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)
