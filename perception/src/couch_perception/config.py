"""Pipeline configuration with YAML loading and preset profiles."""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from pathlib import Path

import yaml

_CONFIGS_DIR = Path(__file__).resolve().parent.parent.parent / "configs"


@dataclass
class PipelineConfig:
    """Configuration for the perception pipeline.

    Controls which models to run, inference settings, and subsampling rates.
    Load from YAML with ``PipelineConfig.from_yaml("configs/fast.yaml")``.
    """

    # Segmentation model: "yolop" or "none"
    segmentation_model: str = "yolop"
    # Path to TensorRT engine (auto-detected from weights/ if None)
    segmentation_engine: str | None = None

    # Object detection model: "yolov8n", "yolov8s", or "none"
    detection_model: str = "yolov8n"
    # Path to model weights or engine (auto-detected if None)
    detection_engine: str | None = None
    detection_confidence: float = 0.3

    # Subsampling (higher = fewer points = faster projection)
    subsample_drivable: int = 4
    subsample_lane: int = 2
    subsample_bbox: int = 8

    # Inference device: "cuda", "mps", "cpu", or None (auto-detect)
    device: str | None = None
    # Run detection + segmentation on separate CUDA streams
    cuda_streams: bool = True

    @classmethod
    def from_yaml(cls, path: str | Path) -> PipelineConfig:
        p = Path(path)
        if not p.is_absolute() and not p.exists():
            # Try relative to configs/ directory
            p = _CONFIGS_DIR / p
        with open(p) as f:
            data = yaml.safe_load(f) or {}
        valid_fields = {f.name for f in fields(cls)}
        filtered = {k: v for k, v in data.items() if k in valid_fields}
        return cls(**filtered)

    @classmethod
    def preset(cls, name: str) -> PipelineConfig:
        """Load a named preset config (e.g. 'default', 'fast', 'accurate')."""
        return cls.from_yaml(_CONFIGS_DIR / f"{name}.yaml")
