from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation
from scipy.linalg import cholesky

# State indices
POS = slice(0, 3)
VEL = slice(3, 6)
ORI = slice(6, 9)   # roll, pitch, yaw (rad)
BA = slice(9, 12)    # accelerometer bias
BG = slice(12, 15)   # gyro bias

GRAVITY = np.array([0.0, 0.0, -9.81])
STATE_DIM = 15

class UKF:
    def __init__(
        self,
        accel_noise: float = 50.0,
        gyro_noise: float = 0.05,
        accel_bias_noise: float = 1e-3,
        gyro_bias_noise: float = 1e-4,
        initial_pos_cov: float = 0.1,
        use_imu: bool = True,
        force_2d: bool = False,
        alpha: float = 0.001,
        beta: float = 2.0,
        kappa: float = 0.0,
    ) -> None:
        self.x: NDArray[np.float64] = np.zeros(STATE_DIM)
        self.P: NDArray[np.float64] = np.eye(STATE_DIM)
        self.P[POS, POS] *= initial_pos_cov
        self.P[VEL, VEL] *= 10.0
        self.P[ORI, ORI] *= 0.1
        self.P[BA, BA] *= 1.0
        self.P[BG, BG] *= 0.1

        self.accel_noise = accel_noise
        self.gyro_noise = gyro_noise
        self.accel_bias_noise = accel_bias_noise
        self.gyro_bias_noise = gyro_bias_noise
        self.initial_pos_cov = initial_pos_cov
        self.use_imu = use_imu
        self.force_2d = force_2d

        # UKF Parameters
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.n = STATE_DIM
        self.lambda_ = alpha**2 * (self.n + kappa) - self.n

        # Weights
        self.wm = np.full(2 * self.n + 1, 0.5 / (self.n + self.lambda_))
        self.wc = np.full(2 * self.n + 1, 0.5 / (self.n + self.lambda_))
        self.wm[0] = self.lambda_ / (self.n + self.lambda_)
        self.wc[0] = self.lambda_ / (self.n + self.lambda_) + (1 - alpha**2 + beta)

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

    def _generate_sigma_points(self) -> NDArray[np.float64]:
        # P must be positive definite. Numerical errors might cause issues, so ensure symmetry/SPD
        self.P = (self.P + self.P.T) / 2
        try:
            L = cholesky((self.n + self.lambda_) * self.P, lower=True)
        except np.linalg.LinAlgError:
            # Fallback if Cholesky fails (e.g., negative eigenvalues due to precision)
            # Add small epsilon to diagonal
            self.P += np.eye(self.n) * 1e-9
            L = cholesky((self.n + self.lambda_) * self.P, lower=True)

        sigmas = np.zeros((2 * self.n + 1, self.n))
        sigmas[0] = self.x
        for i in range(self.n):
            sigmas[i + 1] = self.x + L[:, i]
            sigmas[self.n + i + 1] = self.x - L[:, i]
        return sigmas

    def predict(
        self,
        accel: NDArray[np.float64],
        gyro: NDArray[np.float64],
        orientation_quat: NDArray[np.float64],
        dt: float,
    ) -> None:
        if dt <= 0 or dt > 1.0:
            return

        sigmas = self._generate_sigma_points()
        sigmas_f = np.zeros_like(sigmas)

        # Precompute constants
        rot_input = Rotation.from_quat(orientation_quat)
        input_euler = rot_input.as_euler("xyz")
        R = rot_input.as_matrix()

        # Propagate sigma points
        for i in range(len(sigmas)):
            s = sigmas[i]
            p = s[POS]
            v = s[VEL]
            ba = s[BA]
            bg = s[BG]
            # ori = s[ORI] # Unused in this specific EKF model logic for accel rotation
                           # because we use the input orientation_quat directly.

            if self.use_imu:
                accel_corrected = accel - ba
                accel_enu = R @ accel_corrected - GRAVITY
            else:
                accel_enu = np.zeros(3)

            if self.force_2d:
                accel_enu[2] = 0.0
                v[2] = 0.0

            # State transition
            p_next = p + v * dt + 0.5 * accel_enu * dt**2
            v_next = v + accel_enu * dt

            # Bias walk
            ba_next = ba
            bg_next = bg

            # Orientation: Matches EKF logic of forcing input orientation
            ori_next = input_euler

            sigmas_f[i, POS] = p_next
            sigmas_f[i, VEL] = v_next
            sigmas_f[i, ORI] = ori_next
            sigmas_f[i, BA] = ba_next
            sigmas_f[i, BG] = bg_next

        # Predicted Mean
        x_pred = np.zeros(STATE_DIM)
        for i in range(len(sigmas_f)):
            x_pred += self.wm[i] * sigmas_f[i]

        # Predicted Covariance
        P_pred = np.zeros((STATE_DIM, STATE_DIM))
        for i in range(len(sigmas_f)):
            y = sigmas_f[i] - x_pred
            # Handle angle wrapping if necessary?
            # In this specific model where ORI is forced to input_euler for ALL points,
            # the variance for ORI will be 0 before adding Q. No wrapping issues here.
            P_pred += self.wc[i] * np.outer(y, y)

        # Add Process Noise Q
        Q = np.zeros((STATE_DIM, STATE_DIM))
        Q[POS, POS] = np.eye(3) * (self.accel_noise * dt**2) ** 2
        Q[VEL, VEL] = np.eye(3) * (self.accel_noise * dt) ** 2
        Q[ORI, ORI] = np.eye(3) * (self.gyro_noise * dt) ** 2
        Q[BA, BA] = np.eye(3) * (self.accel_bias_noise * dt) ** 2
        Q[BG, BG] = np.eye(3) * (self.gyro_bias_noise * dt) ** 2

        if self.force_2d:
            Q[2, 2] = 0.0
            Q[5, 5] = 0.0

        self.x = x_pred
        self.P = P_pred + Q

    def update_gps(
        self,
        position_enu: NDArray[np.float64],
        R_cov: NDArray[np.float64],
    ) -> None:
        # Generate sigma points from predicted state
        sigmas = self._generate_sigma_points()

        # Transform to measurement space (Z)
        # Z is just position
        sigmas_h = sigmas[:, POS]

        # Mean measurement
        z_mean = np.zeros(3)
        for i in range(len(sigmas_h)):
            z_mean += self.wm[i] * sigmas_h[i]

        # Measurement Covariance S
        S = np.zeros((3, 3))
        for i in range(len(sigmas_h)):
            y = sigmas_h[i] - z_mean
            S += self.wc[i] * np.outer(y, y)
        S += R_cov

        # Cross Covariance Pxz
        Pxz = np.zeros((STATE_DIM, 3))
        for i in range(len(sigmas)):
            dx = sigmas[i] - self.x
            dz = sigmas_h[i] - z_mean
            Pxz += self.wc[i] * np.outer(dx, dz)

        # Kalman Gain
        K = Pxz @ np.linalg.inv(S)

        # Update
        y_residual = position_enu - z_mean
        self.x += K @ y_residual
        self.P -= K @ S @ K.T

    def update_gps_vel(
        self,
        velocity_enu: NDArray[np.float64],
        R_cov: NDArray[np.float64],
    ) -> None:
        # Generate sigma points from predicted state
        sigmas = self._generate_sigma_points()

        # Transform to measurement space (Z)
        # Z is velocity (VEL is slice(3,6))
        sigmas_h = sigmas[:, VEL]

        # Mean measurement
        z_mean = np.zeros(3)
        for i in range(len(sigmas_h)):
            z_mean += self.wm[i] * sigmas_h[i]

        # Measurement Covariance S
        S = np.zeros((3, 3))
        for i in range(len(sigmas_h)):
            y = sigmas_h[i] - z_mean
            S += self.wc[i] * np.outer(y, y)
        S += R_cov

        # Cross Covariance Pxz
        Pxz = np.zeros((STATE_DIM, 3))
        for i in range(len(sigmas)):
            dx = sigmas[i] - self.x
            dz = sigmas_h[i] - z_mean
            Pxz += self.wc[i] * np.outer(dx, dz)

        # Kalman Gain
        K = Pxz @ np.linalg.inv(S)

        # Update
        y_residual = velocity_enu - z_mean
        self.x += K @ y_residual
        self.P -= K @ S @ K.T

    def update_heading(
        self,
        yaw: float,
        R_cov: float,
    ) -> None:
        # Generate sigma points from predicted state
        sigmas = self._generate_sigma_points()

        # Z is yaw (3rd element of ORI which is slice(6,9))
        # So index is 8
        sigmas_h = sigmas[:, 8]

        # Mean measurement with angle wrapping
        sin_sum = 0.0
        cos_sum = 0.0
        for i in range(len(sigmas_h)):
            sin_sum += self.wm[i] * np.sin(sigmas_h[i])
            cos_sum += self.wm[i] * np.cos(sigmas_h[i])
        z_mean = np.arctan2(sin_sum, cos_sum)

        # Measurement Covariance S
        S = 0.0
        for i in range(len(sigmas_h)):
            y = sigmas_h[i] - z_mean
            # Wrap y
            y = (y + np.pi) % (2 * np.pi) - np.pi
            S += self.wc[i] * (y * y)
        S += R_cov

        # Cross Covariance Pxz
        Pxz = np.zeros(STATE_DIM)
        for i in range(len(sigmas)):
            dx = sigmas[i] - self.x
            dz = sigmas_h[i] - z_mean
            # Wrap dz
            dz = (dz + np.pi) % (2 * np.pi) - np.pi
            Pxz += self.wc[i] * (dx * dz)

        # Kalman Gain
        K = Pxz / S

        # Update
        y_residual = yaw - z_mean
        y_residual = (y_residual + np.pi) % (2 * np.pi) - np.pi

        self.x += K * y_residual
        self.P -= np.outer(K, K) * S
