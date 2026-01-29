from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from .bag_reader import GpsFix, ImuSample, read_bag
from .dashboard import plot_dashboard
from .ekf import EKF
from .geo import geodetic_to_enu
from .result import EkfResult


def run_ekf(
    imu_samples: list[ImuSample],
    gps_fixes: list[GpsFix],
    accel_noise: float = 2.0,
    gyro_noise: float = 0.05,
) -> EkfResult:
    if not imu_samples or not gps_fixes:
        raise ValueError("Need both IMU and GPS data")

    ekf = EKF(accel_noise=accel_noise, gyro_noise=gyro_noise)

    n_gps = len(gps_fixes)
    gps_enu = np.empty((n_gps, 3))
    gps_cov = np.empty((n_gps, 3, 3))
    gps_times = np.empty(n_gps)
    for i, g in enumerate(gps_fixes):
        gps_enu[i] = geodetic_to_enu(g.latitude, g.longitude, g.altitude)
        cov = g.position_covariance
        gps_cov[i] = np.diag([cov[0], cov[4], cov[8]])
        gps_times[i] = g.t

    first_imu = imu_samples[0]
    quat = np.array([first_imu.qx, first_imu.qy, first_imu.qz, first_imu.qw])
    ekf.initialize(gps_enu[0], quat)

    n = len(imu_samples)
    times = np.empty(n)
    positions = np.empty((n, 3))
    velocities = np.empty((n, 3))
    orientations = np.empty((n, 3))
    pos_cov = np.empty((n, 3))
    vel_cov = np.empty((n, 3))
    bias_accel = np.empty((n, 3))
    bias_gyro = np.empty((n, 3))

    gps_idx = 0
    prev_t = imu_samples[0].t

    for i, imu in enumerate(imu_samples):
        dt = imu.t - prev_t if i > 0 else 0.0
        prev_t = imu.t

        accel = np.array([imu.ax, imu.ay, imu.az])
        gyro = np.array([imu.wx, imu.wy, imu.wz])
        quat = np.array([imu.qx, imu.qy, imu.qz, imu.qw])

        ekf.predict(accel, gyro, quat, dt)

        while gps_idx < n_gps and gps_times[gps_idx] <= imu.t:
            ekf.update_gps(gps_enu[gps_idx], gps_cov[gps_idx])
            gps_idx += 1

        times[i] = imu.t
        positions[i] = ekf.x[:3]
        velocities[i] = ekf.x[3:6]
        orientations[i] = ekf.x[6:9]
        pos_cov[i] = np.diag(ekf.P[:3, :3])
        vel_cov[i] = np.diag(ekf.P[3:6, 3:6])
        bias_accel[i] = ekf.x[9:12]
        bias_gyro[i] = ekf.x[12:15]

    return EkfResult(
        times=times,
        positions=positions,
        velocities=velocities,
        orientations=orientations,
        pos_cov=pos_cov,
        vel_cov=vel_cov,
        bias_accel=bias_accel,
        bias_gyro=bias_gyro,
        gps_times=gps_times,
        gps_enu=gps_enu,
    )


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Run EKF on MCAP bag")
    parser.add_argument("bag", type=Path, help="Path to MCAP bag file")
    parser.add_argument("--output", "-o", type=Path, default=None, help="Output PNG path")
    parser.add_argument("--no-show", action="store_true", help="Don't show interactive plot")
    parser.add_argument("--accel-noise", type=float, default=2.0)
    parser.add_argument("--gyro-noise", type=float, default=0.05)
    args = parser.parse_args(argv)

    print(f"Reading bag: {args.bag}")
    imu_samples, gps_fixes = read_bag(args.bag)
    print(f"  IMU samples: {len(imu_samples)}")
    print(f"  GPS fixes:   {len(gps_fixes)}")

    if not imu_samples or not gps_fixes:
        print("ERROR: Need both IMU and GPS data in the bag.", file=sys.stderr)
        sys.exit(1)

    duration = imu_samples[-1].t - imu_samples[0].t
    print(f"  Duration:    {duration:.1f}s")
    print(f"  IMU rate:    {len(imu_samples) / duration:.0f} Hz")
    print(f"  GPS rate:    {len(gps_fixes) / duration:.1f} Hz")

    result = run_ekf(
        imu_samples, gps_fixes,
        accel_noise=args.accel_noise,
        gyro_noise=args.gyro_noise,
    )

    final_pos = result.positions[-1]
    print(f"\nFinal position (ENU relative to Rotunda):")
    print(f"  East:  {final_pos[0]:.1f} m")
    print(f"  North: {final_pos[1]:.1f} m")
    print(f"  Up:    {final_pos[2]:.1f} m")

    output_path = args.output or args.bag.with_suffix(".png")
    plot_dashboard(result, save_path=output_path, show=not args.no_show)
    print(f"\nDashboard saved to: {output_path}")


if __name__ == "__main__":
    main()
