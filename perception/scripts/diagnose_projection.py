"""Diagnostic: print average 3D point positions at each stage of the projection pipeline.

Processes a few frames from a bag, shows camera-frame and world-frame stats
so we can verify which axis is forward, lateral, and up.
"""

import argparse
import numpy as np

from couch_perception.bag_reader import read_synced_frames
from couch_perception.camera_model import make_camera_model, CameraModel
from couch_perception.yolov8_detector import YOLOv8Detector
from couch_perception.yolop_detector import YOLOPDetector
from couch_perception.projection import (
    apply_imu_rotation,
    quat_to_rotation_matrix,
    extract_mask_pixels,
    extract_bbox_pixels,
    build_depth_camera_model,
)


def _stats(name: str, pts: np.ndarray) -> None:
    if len(pts) == 0:
        print(f"  {name}: no points")
        return
    mean = pts.mean(axis=0)
    std = pts.std(axis=0)
    mn = pts.min(axis=0)
    mx = pts.max(axis=0)
    print(f"  {name} ({len(pts)} pts):")
    print(f"    mean  x={mean[0]:+.3f}  y={mean[1]:+.3f}  z={mean[2]:+.3f}")
    print(f"    std   x={std[0]:.3f}   y={std[1]:.3f}   z={std[2]:.3f}")
    print(f"    min   x={mn[0]:+.3f}  y={mn[1]:+.3f}  z={mn[2]:+.3f}")
    print(f"    max   x={mx[0]:+.3f}  y={mx[1]:+.3f}  z={mx[2]:+.3f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--device", default=None)
    parser.add_argument("--max-frames", type=int, default=5)
    args = parser.parse_args()

    yolo = YOLOv8Detector(conf_threshold=0.3, device=args.device)
    yolop = YOLOPDetector(device=args.device)

    frames = read_synced_frames(args.bag)
    cam_model = None
    depth_cam_model = None

    for i, frame in enumerate(frames):
        if i >= args.max_frames:
            break

        if cam_model is None:
            cam_model = make_camera_model(frame.intrinsics)
            depth_cam_model = build_depth_camera_model(cam_model, frame.depth.shape[:2])

        print(f"\n{'='*60}")
        print(f"Frame {i} (t={frame.timestamp:.3f})")
        if frame.orientation is not None:
            q = frame.orientation
            print(f"  IMU quat (x,y,z,w): {q[0]:.4f} {q[1]:.4f} {q[2]:.4f} {q[3]:.4f}")
            R = quat_to_rotation_matrix(q)
            print(f"  R.T (device-to-world):")
            for row in R.T:
                print(f"    [{row[0]:+.4f} {row[1]:+.4f} {row[2]:+.4f}]")

        detections = yolo.detect(frame.image)
        yolop_result = yolop.detect(frame.image)

        # Drivable
        if yolop_result and yolop_result.drivable_mask is not None:
            pixels, depths = extract_mask_pixels(yolop_result.drivable_mask, frame.depth, subsample=4)
            if len(pixels) > 0:
                cam_pts = depth_cam_model.project_pixels_to_3d(pixels, depths)
                _stats("drivable (camera-frame)", cam_pts)
                world_pts = apply_imu_rotation(cam_pts, frame.orientation)
                _stats("drivable (world-frame)", world_pts)
                # Expected: drivable should be mostly in front (x>0), spread laterally (y),
                # and near ground (z≈0)

        # Lanes
        if yolop_result and yolop_result.lane_mask is not None:
            pixels, depths = extract_mask_pixels(yolop_result.lane_mask, frame.depth, subsample=2)
            if len(pixels) > 0:
                cam_pts = depth_cam_model.project_pixels_to_3d(pixels, depths)
                _stats("lanes (camera-frame)", cam_pts)
                world_pts = apply_imu_rotation(cam_pts, frame.orientation)
                _stats("lanes (world-frame)", world_pts)

        # Detections
        if detections:
            pixels, depths = extract_bbox_pixels(detections, frame.depth, frame.image.shape[:2], subsample=8)
            if len(pixels) > 0:
                cam_pts = depth_cam_model.project_pixels_to_3d(pixels, depths)
                _stats("detections (camera-frame)", cam_pts)
                world_pts = apply_imu_rotation(cam_pts, frame.orientation)
                _stats("detections (world-frame)", world_pts)

        print(f"\n  Camera frame convention: x=right, y=down, z=forward")
        print(f"  Expected world frame: x=forward(+), y=lateral, z=up(+)")
        print(f"  Drivable area should have: mean x>0 (in front), z≈0 (ground)")


if __name__ == "__main__":
    main()
