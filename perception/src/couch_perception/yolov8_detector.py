"""YOLOv8n object detection wrapper using ultralytics."""

from dataclasses import dataclass

import numpy as np
from ultralytics import YOLO

RELEVANT_CLASSES: dict[int, str] = {
    0: "person",
    1: "bicycle",
    2: "car",
    3: "motorcycle",
    5: "bus",
    7: "truck",
    9: "traffic_light",
    11: "stop_sign",
}


@dataclass(frozen=True, slots=True)
class Detection:
    x1: int
    y1: int
    x2: int
    y2: int
    confidence: float
    class_id: int
    class_name: str


class YOLOv8Detector:
    def __init__(self, model_path: str = "yolov8n.pt", conf_threshold: float = 0.3, device: str | None = None) -> None:
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.device = device
        self._relevant_class_ids = list(RELEVANT_CLASSES.keys())

    def detect(self, frame: np.ndarray) -> list[Detection]:
        results = self.model(
            frame,
            conf=self.conf_threshold,
            classes=self._relevant_class_ids,
            device=self.device,
            verbose=False,
        )
        detections: list[Detection] = []
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                detections.append(Detection(
                    x1=x1, y1=y1, x2=x2, y2=y2,
                    confidence=float(box.conf[0]),
                    class_id=cls_id,
                    class_name=RELEVANT_CLASSES.get(cls_id, str(cls_id)),
                ))
        return detections
