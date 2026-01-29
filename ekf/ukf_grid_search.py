import sys
import numpy as np
import itertools
from pathlib import Path
sys.path.append("src")

from couch_ekf.bag_reader import read_bag, GpsFix, ImuSample, GpsVel, Heading
from couch_ekf.ukf import UKF
from couch_ekf.geo import geodetic_to_enu
from couch_ekf.result import EkfResult

def calculate_rmse(result):
    ekf_e = np.interp(result.gps_times, result.times, result.positions[:, 0])
    ekf_n = np.interp(result.gps_times, result.times, result.positions[:, 1])
    ekf_u = np.interp(result.gps_times, result.times, result.positions[:, 2])
    ekf_pos_at_gps = np.stack([ekf_e, ekf_n, ekf_u], axis=1)
    diff = ekf_pos_at_gps - result.gps_enu
    sq_err = np.sum(diff**2, axis=1)
    rmse = np.sqrt(np.mean(sq_err))
    return rmse

def run_ukf(imu_samples, gps_fixes, gps_vels, headings, accel_noise, gyro_noise, vel_noise, heading_noise, initial_pos_cov, use_imu, force_2d):
    ukf = UKF(
        accel_noise=accel_noise,
        gyro_noise=gyro_noise,
        initial_pos_cov=initial_pos_cov,
        use_imu=use_imu,
        force_2d=force_2d,
    )

    n_gps = len(gps_fixes)
    gps_enu = np.empty((n_gps, 3))
    gps_cov = np.empty((n_gps, 3, 3))
    gps_times = np.empty(n_gps)
    for i, g in enumerate(gps_fixes):
        gps_enu[i] = geodetic_to_enu(g.latitude, g.longitude, g.altitude)
        cov = g.position_covariance
        gps_cov[i] = np.diag([cov[0], cov[4], cov[8]])
        gps_times[i] = g.t

    # Pre-process Velocity
    n_vel = len(gps_vels)
    vel_data = np.empty((n_vel, 3))
    vel_times = np.empty(n_vel)
    for i, v in enumerate(gps_vels):
        vel_data[i] = [v.vx, v.vy, v.vz]
        vel_times[i] = v.t

    # Pre-process Heading
    n_head = len(headings)
    head_data = np.empty(n_head)
    head_times = np.empty(n_head)
    for i, h in enumerate(headings):
        head_data[i] = h.yaw
        head_times[i] = h.t

    first_imu = imu_samples[0]
    quat = np.array([first_imu.qx, first_imu.qy, first_imu.qz, first_imu.qw])
    ukf.initialize(gps_enu[0], quat)

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
    vel_idx = 0
    head_idx = 0
    prev_t = imu_samples[0].t

    for i, imu in enumerate(imu_samples):
        dt = imu.t - prev_t if i > 0 else 0.0
        prev_t = imu.t

        accel = np.array([imu.ax, imu.ay, imu.az])
        gyro = np.array([imu.wx, imu.wy, imu.wz])
        quat = np.array([imu.qx, imu.qy, imu.qz, imu.qw])

        ukf.predict(accel, gyro, quat, dt)

        while gps_idx < n_gps and gps_times[gps_idx] <= imu.t:
            ukf.update_gps(gps_enu[gps_idx], gps_cov[gps_idx])
            gps_idx += 1

        while vel_idx < n_vel and vel_times[vel_idx] <= imu.t:
            R_vel = np.eye(3) * (vel_noise**2)
            ukf.update_gps_vel(vel_data[vel_idx], R_vel)
            vel_idx += 1

        while head_idx < n_head and head_times[head_idx] <= imu.t:
            R_head = heading_noise**2
            ukf.update_heading(head_data[head_idx], R_head)
            head_idx += 1

        times[i] = imu.t
        positions[i] = ukf.x[:3]
        velocities[i] = ukf.x[3:6]
        orientations[i] = ukf.x[6:9]
        pos_cov[i] = np.diag(ukf.P[:3, :3])
        vel_cov[i] = np.diag(ukf.P[3:6, 3:6])
        bias_accel[i] = ukf.x[9:12]
        bias_gyro[i] = ukf.x[12:15]

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

def main():
    bag_path = Path("../bags/university_intersect_gps_only.mcap")
    if not bag_path.exists():
        bag_path = Path("/Users/wkaiser/Coding/couch-vision/bags/university_intersect_gps_only.mcap")

    print(f"Reading {bag_path}...")
    imu, gps, gps_vels, headings = read_bag(bag_path)

    print(f"Data counts:")
    print(f"  IMU: {len(imu)}")
    print(f"  GPS: {len(gps)}")
    print(f"  GPS Vel: {len(gps_vels)}")
    print(f"  Headings: {len(headings)}")

    # Coarse Grid Search
    accel_noises = [50.0]
    gyro_noises = [0.05]
    vel_noises = [1.0, 1000.0]
    heading_noises = [0.1, 1000.0]

    print(f"Grid Search over:")
    print(f"  Accel: {accel_noises}")
    print(f"  Gyro: {gyro_noises}")
    print(f"  Vel: {vel_noises}")
    print(f"  Heading: {heading_noises}")

    best_rmse = float('inf')
    best_params = None

    total_runs = len(accel_noises) * len(gyro_noises) * len(vel_noises) * len(heading_noises)
    count = 0

    for accel, gyro, vel, heading in itertools.product(accel_noises, gyro_noises, vel_noises, heading_noises):
        count += 1
        print(f"[{count}/{total_runs}] Testing: Accel={accel}, Gyro={gyro}, Vel={vel}, Head={heading}")
        try:
            res = run_ukf(imu, gps, gps_vels, headings,
                          accel_noise=accel, gyro_noise=gyro,
                          vel_noise=vel, heading_noise=heading,
                          initial_pos_cov=100.0, use_imu=True, force_2d=False)
            rmse = calculate_rmse(res)
            print(f"  RMSE: {rmse:.4f}")
            if rmse < best_rmse:
                best_rmse = rmse
                best_params = (accel, gyro, vel, heading)
        except Exception as e:
            print(f"  Failed: {e}")

    print(f"\nBest RMSE: {best_rmse:.4f}")
    if best_params:
        print(f"Best Params: Accel={best_params[0]}, Gyro={best_params[1]}, Vel={best_params[2]}, Head={best_params[3]}")

if __name__ == "__main__":
    main()
