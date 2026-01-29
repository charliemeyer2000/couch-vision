import sys
import numpy as np
from pathlib import Path
# Add src to path so we can import couch_ekf
sys.path.append("src")

from couch_ekf.bag_reader import read_bag
from couch_ekf.runner import run_ekf

def calculate_rmse(result):
    # Interpolate EKF positions to GPS times
    ekf_e = np.interp(result.gps_times, result.times, result.positions[:, 0])
    ekf_n = np.interp(result.gps_times, result.times, result.positions[:, 1])
    ekf_u = np.interp(result.gps_times, result.times, result.positions[:, 2])

    ekf_pos_at_gps = np.stack([ekf_e, ekf_n, ekf_u], axis=1)

    diff = ekf_pos_at_gps - result.gps_enu

    sq_err = np.sum(diff**2, axis=1)
    rmse = np.sqrt(np.mean(sq_err))
    return rmse

def main():
    # Bag path relative to ekf/ directory
    bag_path = Path("../bags/university_intersect_gps_only.mcap")
    if not bag_path.exists():
        # Try absolute path based on workspace
        bag_path = Path("/Users/wkaiser/Coding/couch-vision/bags/university_intersect_gps_only.mcap")

    if not bag_path.exists():
        print(f"Bag not found at {bag_path}")
        return

    print(f"Reading {bag_path}...")
    imu, gps, _, _ = read_bag(bag_path)

    # Grid search ranges
    # We suspect initial covariance is too high (100.0), so we try lower values.
    init_covs = [0.01, 0.1, 1.0, 10.0, 100.0]
    # accel_noise default was 2.0.
    accel_noises = [1.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0]
    # gyro_noise default was 0.05. It seems to have no effect in current implementation.
    gyro_noises = [0.05]

    best_rmse = float('inf')
    best_params = {}

    print(f"Starting grid search...")
    print(f"{'InitCov':<10} {'Accel':<10} {'Gyro':<10} {'RMSE':<10}")

    for cov in init_covs:
        for acc in accel_noises:
            for gyr in gyro_noises:
                try:
                    res = run_ekf(imu, gps, accel_noise=acc, gyro_noise=gyr, initial_pos_cov=cov)
                    rmse = calculate_rmse(res)
                    print(f"{cov:<10.1f} {acc:<10.2f} {gyr:<10.3f} {rmse:<10.3f}")

                    if rmse < best_rmse:
                        best_rmse = rmse
                        best_params = {'init_cov': cov, 'accel': acc, 'gyro': gyr}
                except Exception as e:
                    print(f"Failed for {cov}, {acc}, {gyr}: {e}")

    print("-" * 40)
    print(f"Best RMSE: {best_rmse:.3f}")
    print(f"Best Params: {best_params}")

if __name__ == "__main__":
    main()
