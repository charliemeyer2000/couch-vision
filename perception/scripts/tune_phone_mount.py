"""Tune phone mount heading mapping from a real bag using GPS/IMU.

Usage:
  cd perception
  uv run python scripts/tune_phone_mount.py \
    --bag ../bags/walk_around_university_all_data.mcap
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from couch_perception.bag_reader import read_gps_imu_and_odom
from couch_perception.frame_model import load_mount_frame_model
from couch_perception.geo import geodetic_to_enu


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _circular_mean(angles: np.ndarray) -> float:
    return math.atan2(float(np.mean(np.sin(angles))), float(np.mean(np.cos(angles))))


def _enu_to_body_xy(de: float, dn: float, heading_enu: float) -> tuple[float, float]:
    sin_h = math.sin(heading_enu)
    cos_h = math.cos(heading_enu)
    x_forward = de * sin_h + dn * cos_h
    y_left = -de * cos_h + dn * sin_h
    return x_forward, y_left


@dataclass(frozen=True)
class MappingScore:
    name: str
    use_inverse: bool
    samples: int
    heading_offset_rad: float
    heading_mae_deg: float
    mean_abs_goal_y_m: float
    median_goal_y_m: float
    mean_goal_x_m: float


def _heading_from_axis(
    orientation_quat: np.ndarray, axis_imu: np.ndarray, use_inverse: bool
) -> float | None:
    rot = Rotation.from_quat(orientation_quat)
    forward_world = rot.inv().apply(axis_imu) if use_inverse else rot.apply(axis_imu)
    east = float(forward_world[0])
    north = float(forward_world[1])
    norm = math.hypot(east, north)
    if norm < 1e-6:
        return None
    return math.atan2(east / norm, north / norm)


def _score_mapping(
    gps_enu: np.ndarray,
    gps_t: np.ndarray,
    imu_t: np.ndarray,
    imu_q: np.ndarray,
    axis_imu: np.ndarray,
    use_inverse: bool,
    lookahead_m: float,
    min_step_m: float,
    max_skew_s: float,
) -> MappingScore | None:
    seg = np.linalg.norm(np.diff(gps_enu[:, :2], axis=0), axis=1)
    cumulative_s = np.concatenate([[0.0], np.cumsum(seg)])

    # Course samples where GPS moved enough to trust direction.
    sample_gps_idx: list[int] = []
    sample_course: list[float] = []
    sample_heading_imu: list[float] = []
    last_idx = 0
    for i in range(1, len(gps_enu)):
        de = float(gps_enu[i, 0] - gps_enu[last_idx, 0])
        dn = float(gps_enu[i, 1] - gps_enu[last_idx, 1])
        dist = math.hypot(de, dn)
        if dist < min_step_m:
            continue

        course = math.atan2(de, dn)
        j = int(np.argmin(np.abs(imu_t - gps_t[i])))
        if abs(float(imu_t[j] - gps_t[i])) > max_skew_s:
            continue

        heading = _heading_from_axis(imu_q[j], axis_imu, use_inverse=use_inverse)
        if heading is None:
            continue

        sample_gps_idx.append(i)
        sample_course.append(course)
        sample_heading_imu.append(heading)
        last_idx = i

    if len(sample_gps_idx) < 20:
        return None

    course_arr = np.array(sample_course, dtype=np.float64)
    heading_arr = np.array(sample_heading_imu, dtype=np.float64)

    diffs = np.array([_wrap(float(c - h)) for c, h in zip(course_arr, heading_arr)], dtype=np.float64)
    heading_offset = _circular_mean(diffs)
    residual = np.array([_wrap(float(d - heading_offset)) for d in diffs], dtype=np.float64)
    heading_mae_deg = math.degrees(float(np.mean(np.abs(residual))))

    goal_xs: list[float] = []
    goal_ys: list[float] = []
    for gps_idx, heading in zip(sample_gps_idx, heading_arr):
        target_s = float(cumulative_s[gps_idx] + lookahead_m)
        ahead_idx = int(np.searchsorted(cumulative_s, target_s))
        if ahead_idx >= len(gps_enu):
            continue
        de = float(gps_enu[ahead_idx, 0] - gps_enu[gps_idx, 0])
        dn = float(gps_enu[ahead_idx, 1] - gps_enu[gps_idx, 1])
        x, y = _enu_to_body_xy(de, dn, _wrap(float(heading + heading_offset)))
        goal_xs.append(x)
        goal_ys.append(y)

    if len(goal_ys) < 20:
        return None

    y_arr = np.array(goal_ys, dtype=np.float64)
    x_arr = np.array(goal_xs, dtype=np.float64)

    return MappingScore(
        name=f"axis={axis_imu.tolist()}",
        use_inverse=use_inverse,
        samples=len(goal_ys),
        heading_offset_rad=heading_offset,
        heading_mae_deg=heading_mae_deg,
        mean_abs_goal_y_m=float(np.mean(np.abs(y_arr))),
        median_goal_y_m=float(np.median(y_arr)),
        mean_goal_x_m=float(np.mean(x_arr)),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Tune phone mount heading mapping from bag data.")
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--lookahead", type=float, default=8.0)
    parser.add_argument("--min-step", type=float, default=0.8)
    parser.add_argument("--max-skew", type=float, default=1.0)
    args = parser.parse_args()

    gps, imu, _ = read_gps_imu_and_odom(args.bag)
    if len(gps) < 3 or len(imu) < 3:
        raise RuntimeError("Not enough GPS/IMU samples for tuning.")

    gps_enu = np.array(
        [geodetic_to_enu(g.latitude, g.longitude, g.altitude) for g in gps],
        dtype=np.float64,
    )
    gps_t = np.array([g.timestamp for g in gps], dtype=np.float64)
    imu_t = np.array([s.timestamp for s in imu], dtype=np.float64)
    imu_q = np.array([s.orientation for s in imu], dtype=np.float64)

    print(f"Loaded bag: {args.bag}")
    print(f"GPS samples: {len(gps)}, IMU samples: {len(imu)}")

    candidates: list[tuple[str, np.ndarray]] = [
        ("+X", np.array([1.0, 0.0, 0.0], dtype=np.float64)),
        ("-X", np.array([-1.0, 0.0, 0.0], dtype=np.float64)),
        ("+Y", np.array([0.0, 1.0, 0.0], dtype=np.float64)),
        ("-Y", np.array([0.0, -1.0, 0.0], dtype=np.float64)),
        ("+Z", np.array([0.0, 0.0, 1.0], dtype=np.float64)),
        ("-Z", np.array([0.0, 0.0, -1.0], dtype=np.float64)),
    ]

    scores: list[tuple[str, MappingScore]] = []
    for label, axis in candidates:
        for use_inverse in (False, True):
            score = _score_mapping(
                gps_enu=gps_enu,
                gps_t=gps_t,
                imu_t=imu_t,
                imu_q=imu_q,
                axis_imu=axis,
                use_inverse=use_inverse,
                lookahead_m=args.lookahead,
                min_step_m=args.min_step,
                max_skew_s=args.max_skew,
            )
            if score is not None:
                scores.append((label, score))

    scores.sort(key=lambda x: x[1].mean_abs_goal_y_m)
    print("\nTop candidate mappings:")
    for label, score in scores[:8]:
        print(
            f"  {label:>2} inv={score.use_inverse:<5} "
            f"|mean(goal_y)|={score.mean_abs_goal_y_m:.2f}m "
            f"median(goal_y)={score.median_goal_y_m:+.2f}m "
            f"mean(goal_x)={score.mean_goal_x_m:.2f}m "
            f"heading_mae={score.heading_mae_deg:.1f}deg "
            f"offset={math.degrees(score.heading_offset_rad):+.1f}deg"
        )

    mount = load_mount_frame_model()
    urdf_axis = mount.base_forward_axis_in_imu
    urdf_score = _score_mapping(
        gps_enu=gps_enu,
        gps_t=gps_t,
        imu_t=imu_t,
        imu_q=imu_q,
        axis_imu=urdf_axis,
        use_inverse=False,
        lookahead_m=args.lookahead,
        min_step_m=args.min_step,
        max_skew_s=args.max_skew,
    )
    if urdf_score is None:
        raise RuntimeError("Failed to score URDF mount model.")

    print("\nURDF mount check:")
    print(f"  URDF: {mount.urdf_path}")
    print(f"  base_forward_axis_in_imu: {np.array2string(urdf_axis, precision=3)}")
    print(f"  base_up_axis_in_imu:      {np.array2string(mount.base_up_axis_in_imu, precision=3)}")
    print(f"  imu height above base:    {mount.translation_base_from_imu_m[2]:.3f} m")
    print(
        "  Metrics: "
        f"|mean(goal_y)|={urdf_score.mean_abs_goal_y_m:.2f}m, "
        f"median(goal_y)={urdf_score.median_goal_y_m:+.2f}m, "
        f"heading_mae={urdf_score.heading_mae_deg:.1f}deg, "
        f"offset={math.degrees(urdf_score.heading_offset_rad):+.1f}deg"
    )


if __name__ == "__main__":
    main()
