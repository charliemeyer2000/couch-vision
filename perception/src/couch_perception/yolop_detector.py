"""YOLOP drivable area + lane line segmentation wrapper.

Ported from wkaisertexas/yolop-demo branch's lane/analyze_video.py.
Downloads the hustvl/YOLOP repo and weights on first run.
"""

import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import torch
import torchvision.transforms as transforms

_WEIGHTS_DIR = Path(__file__).resolve().parent.parent.parent / "weights"
_YOLOP_REPO_DIR = _WEIGHTS_DIR / "yolop_repo"
_WEIGHTS_PATH = _YOLOP_REPO_DIR / "weights" / "End-to-end.pth"

_normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
_transform = transforms.Compose([transforms.ToTensor(), _normalize])


def _ensure_yolop_repo() -> None:
    if (_YOLOP_REPO_DIR / "lib").is_dir():
        return
    _WEIGHTS_DIR.mkdir(parents=True, exist_ok=True)
    print(f"Cloning hustvl/YOLOP to {_YOLOP_REPO_DIR}...")
    subprocess.run(
        ["git", "clone", "--depth=1", "https://github.com/hustvl/YOLOP.git", str(_YOLOP_REPO_DIR)],
        check=True,
    )


def _ensure_weights() -> None:
    if _WEIGHTS_PATH.is_file():
        return
    raise FileNotFoundError(
        f"YOLOP weights not found at {_WEIGHTS_PATH}. "
        "Run _ensure_yolop_repo() first — weights ship with the repo clone."
    )


def _get_device() -> torch.device:
    if torch.cuda.is_available():
        return torch.device("cuda")
    if torch.backends.mps.is_available():
        return torch.device("mps")
    return torch.device("cpu")


@dataclass(frozen=True, slots=True)
class YOLOPResult:
    drivable_mask: np.ndarray
    lane_mask: np.ndarray


class YOLOPDetector:
    def __init__(self, device: str | None = None) -> None:
        _ensure_yolop_repo()
        _ensure_weights()

        repo_str = str(_YOLOP_REPO_DIR)
        if repo_str not in sys.path:
            sys.path.insert(0, repo_str)

        from lib.config import cfg
        from lib.models import get_net

        self.device = torch.device(device) if device else _get_device()
        print(f"YOLOP using device: {self.device}")

        self.model = get_net(cfg)
        checkpoint = torch.load(_WEIGHTS_PATH, map_location=self.device, weights_only=False)
        self.model.load_state_dict(checkpoint["state_dict"])
        self.model.to(self.device)
        self.model.eval()

    def detect(self, frame_bgr: np.ndarray) -> YOLOPResult:
        img_h, img_w = frame_bgr.shape[:2]
        img_size = 640

        r = min(img_size / img_h, img_size / img_w)
        new_h, new_w = int(round(img_h * r)), int(round(img_w * r))
        pad_h = (img_size - new_h) / 2
        pad_w = (img_size - new_w) / 2

        resized = cv2.resize(frame_bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(pad_h - 0.1)), int(round(pad_h + 0.1))
        left, right = int(round(pad_w - 0.1)), int(round(pad_w + 0.1))
        padded = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))

        rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        inputs = _transform(rgb).to(self.device).unsqueeze(0)

        with torch.no_grad():
            _det_out, da_seg_out, ll_seg_out = self.model(inputs)

        _, _, height, width = inputs.shape
        pad_h_int = int(round(pad_h))
        pad_w_int = int(round(pad_w))

        da_crop = da_seg_out[:, :, pad_h_int : (height - pad_h_int), pad_w_int : (width - pad_w_int)]
        da_up = torch.nn.functional.interpolate(da_crop, size=(img_h, img_w), mode="bilinear")
        da_mask = torch.max(da_up, 1)[1].squeeze().cpu().numpy().astype(np.uint8)

        ll_crop = ll_seg_out[:, :, pad_h_int : (height - pad_h_int), pad_w_int : (width - pad_w_int)]
        ll_up = torch.nn.functional.interpolate(ll_crop, size=(img_h, img_w), mode="bilinear")
        ll_mask = torch.max(ll_up, 1)[1].squeeze().cpu().numpy().astype(np.uint8)

        return YOLOPResult(drivable_mask=da_mask, lane_mask=ll_mask)
