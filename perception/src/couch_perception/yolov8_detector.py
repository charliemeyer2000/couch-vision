"""YOLOv8n object detection wrapper using ultralytics."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
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


def _auto_device() -> str:
    if torch.cuda.is_available():
        return "cuda"
    if hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
        return "mps"
    return "cpu"


_WEIGHTS_DIR = Path(__file__).resolve().parent.parent.parent / "weights"


def _find_model(model_path: str, device: str) -> str:
    """Prefer TensorRT engine; auto-export on CUDA if .pt exists."""
    p = Path(model_path)
    engine = p.with_suffix(".engine")
    if engine.exists():
        return str(engine)
    if device == "cuda":
        # Check weights dir for .pt even if model_path is a bare name
        pt = p if p.exists() else _WEIGHTS_DIR / p.name
        if pt.exists():
            print(f"Exporting TensorRT engine from {pt} (this takes ~10 min on Jetson Orin)...")
            model = YOLO(str(pt))
            exported = model.export(format="engine", half=True)
            if exported and Path(exported).exists():
                return str(exported)
            if engine.exists():
                return str(engine)
    return model_path


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
        self.conf_threshold = conf_threshold
        self.device = device or _auto_device()
        resolved = _find_model(model_path, self.device)
        self.model = YOLO(resolved)
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
