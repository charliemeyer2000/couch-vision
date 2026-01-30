from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass
class EkfResult:
    times: NDArray[np.float64]
    positions: NDArray[np.float64]    # (N, 3) ENU
    velocities: NDArray[np.float64]   # (N, 3)
    orientations: NDArray[np.float64] # (N, 3) euler
    pos_cov: NDArray[np.float64]      # (N, 3) diagonal
    vel_cov: NDArray[np.float64]      # (N, 3)
    bias_accel: NDArray[np.float64]   # (N, 3)
    bias_gyro: NDArray[np.float64]    # (N, 3)
    gps_times: NDArray[np.float64]
    gps_enu: NDArray[np.float64]      # (M, 3)
