#!/usr/bin/env python3
"""ROS2 perception node — runs YOLOv8 + optional YOLOP on live camera frames."""

import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from couch_perception.yolov8_detector import YOLOv8Detector

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class PerceptionNode(Node):  # type: ignore[misc]
    def __init__(
        self,
        input_topic: str,
        conf: float,
        device: str | None,
        skip_yolop: bool,
    ) -> None:
        super().__init__("perception_node")

        self._yolo = YOLOv8Detector(conf_threshold=conf, device=device)
        self.get_logger().info(
            f"YOLOv8 loaded (conf={conf}, device={self._yolo.device}, "
            f"model={self._yolo.model.model_name})"
        )

        self._yolop = None
        if not skip_yolop:
            self.get_logger().info("Loading YOLOP...")
            from couch_perception.yolop_detector import YOLOPDetector
            self._yolop = YOLOPDetector(device=device)

        self._det_pub = self.create_publisher(
            Detection2DArray, "/perception/detections", 10
        )
        self._overlay_pub = self.create_publisher(
            CompressedImage, "/perception/overlay/compressed", _SENSOR_QOS
        )
        self._lane_pub = self.create_publisher(
            Image, "/perception/lane_mask", _SENSOR_QOS
        )
        self._drivable_pub = self.create_publisher(
            Image, "/perception/drivable_mask", _SENSOR_QOS
        )

        self.create_subscription(
            CompressedImage, input_topic, self._on_image, _SENSOR_QOS
        )

        self._frame_count = 0
        self._last_log_time = time.monotonic()
        self.get_logger().info(
            f"Perception node ready — subscribing to {input_topic}"
        )

    def _on_image(self, msg: CompressedImage) -> None:
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if bgr is None:
            return

        detections = self._yolo.detect(bgr)

        det_array = Detection2DArray()
        det_array.header = msg.header
        for d in detections:
            det = Detection2D()
            det.bbox.center.position.x = float((d.x1 + d.x2) / 2)
            det.bbox.center.position.y = float((d.y1 + d.y2) / 2)
            det.bbox.size_x = float(d.x2 - d.x1)
            det.bbox.size_y = float(d.y2 - d.y1)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.class_name
            hyp.hypothesis.score = d.confidence
            det.results.append(hyp)
            det_array.detections.append(det)
        self._det_pub.publish(det_array)

        if self._yolop is not None:
            yolop_result = self._yolop.detect(bgr)
            self._publish_mask(
                self._lane_pub, msg.header, yolop_result.lane_mask
            )
            self._publish_mask(
                self._drivable_pub, msg.header, yolop_result.drivable_mask
            )

        overlay = bgr.copy()
        for d in detections:
            cv2.rectangle(overlay, (d.x1, d.y1), (d.x2, d.y2), (0, 255, 0), 2)
            label = f"{d.class_name} {d.confidence:.2f}"
            cv2.putText(
                overlay, label, (d.x1, d.y1 - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
            )
        if self._yolop is not None:
            da_color = np.zeros_like(overlay)
            da_color[yolop_result.drivable_mask == 1] = (0, 128, 0)
            overlay = cv2.addWeighted(overlay, 1.0, da_color, 0.3, 0)
            overlay[yolop_result.lane_mask == 1] = (255, 100, 0)

        _, jpeg = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, 70])
        out = CompressedImage()
        out.header = msg.header
        out.format = "jpeg"
        out.data = jpeg.tobytes()
        self._overlay_pub.publish(out)

        self._frame_count += 1
        now = time.monotonic()
        elapsed = now - self._last_log_time
        if elapsed >= 5.0:
            fps = self._frame_count / elapsed
            self.get_logger().info(f"Perception: {fps:.1f} FPS ({len(detections)} detections)")
            self._frame_count = 0
            self._last_log_time = now

    def _publish_mask(self, pub, header, mask: np.ndarray) -> None:
        img = Image()
        img.header = header
        img.height, img.width = mask.shape[:2]
        img.encoding = "mono8"
        img.is_bigendian = 0
        img.step = img.width
        img.data = (mask * 255).astype(np.uint8).tobytes()
        pub.publish(img)


def main() -> None:
    parser = argparse.ArgumentParser(description="CouchVision Perception Node")
    parser.add_argument(
        "--topic", default="/iphone_charlie/camera/arkit/image/compressed",
        help="Input compressed image topic",
    )
    parser.add_argument("--conf", type=float, default=0.3, help="YOLO confidence threshold")
    parser.add_argument("--device", default=None, help="Inference device (cpu, cuda, mps)")
    parser.add_argument("--skip-yolop", action="store_true", help="Skip YOLOP segmentation")
    args = parser.parse_args()

    rclpy.init()
    node = PerceptionNode(
        input_topic=args.topic,
        conf=args.conf,
        device=args.device,
        skip_yolop=args.skip_yolop,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
