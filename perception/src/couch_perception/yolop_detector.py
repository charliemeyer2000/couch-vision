"""YOLOP drivable area + lane line segmentation wrapper.

Ported from wkaisertexas/yolop-demo branch's lane/analyze_video.py.
Downloads the hustvl/YOLOP repo and weights on first run.

Supports TensorRT backend: if weights/yolop_seg.engine exists, uses it
for FP16 inference. Otherwise falls back to PyTorch.
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
_TRT_ENGINE_PATH = _WEIGHTS_DIR / "yolop_seg.engine"

_normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
_transform = transforms.Compose([transforms.ToTensor(), _normalize])

_IMG_SIZE = 640


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


def _preprocess(frame_bgr: np.ndarray) -> tuple[torch.Tensor, int, int, float, float]:
    """Resize, pad, normalize. Returns (tensor, img_h, img_w, pad_h, pad_w)."""
    img_h, img_w = frame_bgr.shape[:2]
    r = min(_IMG_SIZE / img_h, _IMG_SIZE / img_w)
    new_h, new_w = int(round(img_h * r)), int(round(img_w * r))
    pad_h = (_IMG_SIZE - new_h) / 2
    pad_w = (_IMG_SIZE - new_w) / 2

    resized = cv2.resize(frame_bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(pad_h - 0.1)), int(round(pad_h + 0.1))
    left, right = int(round(pad_w - 0.1)), int(round(pad_w + 0.1))
    padded = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))

    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
    tensor = _transform(rgb).unsqueeze(0)
    return tensor, img_h, img_w, pad_h, pad_w


def _postprocess(
    da_seg_out: torch.Tensor,
    ll_seg_out: torch.Tensor,
    img_h: int,
    img_w: int,
    pad_h: float,
    pad_w: float,
) -> "YOLOPResult":
    """Crop padding, upsample to original size, argmax."""
    pad_h_int = int(round(pad_h))
    pad_w_int = int(round(pad_w))

    # Argmax at model resolution, then nearest-neighbor upsample class IDs.
    # Cheaper than bilinear-upsampling float logits then argmax.
    da_crop = da_seg_out[:, :, pad_h_int : (_IMG_SIZE - pad_h_int), pad_w_int : (_IMG_SIZE - pad_w_int)]
    da_cls = torch.max(da_crop, 1)[1].unsqueeze(1).float()
    da_mask = torch.nn.functional.interpolate(da_cls, size=(img_h, img_w), mode="nearest")
    da_mask = da_mask.squeeze().to(torch.uint8).cpu().numpy()

    ll_crop = ll_seg_out[:, :, pad_h_int : (_IMG_SIZE - pad_h_int), pad_w_int : (_IMG_SIZE - pad_w_int)]
    ll_cls = torch.max(ll_crop, 1)[1].unsqueeze(1).float()
    ll_mask = torch.nn.functional.interpolate(ll_cls, size=(img_h, img_w), mode="nearest")
    ll_mask = ll_mask.squeeze().to(torch.uint8).cpu().numpy()

    return YOLOPResult(drivable_mask=da_mask, lane_mask=ll_mask)


@dataclass(frozen=True, slots=True)
class YOLOPResult:
    drivable_mask: np.ndarray
    lane_mask: np.ndarray


class YOLOPDetector:
    def __init__(self, device: str | None = None) -> None:
        self.device = torch.device(device) if device else _get_device()
        self._use_trt = False

        if _TRT_ENGINE_PATH.exists() and self.device.type == "cuda":
            self._init_trt()
        else:
            self._init_pytorch()

    def _init_trt(self) -> None:
        import tensorrt as trt

        print(f"YOLOP using TensorRT engine: {_TRT_ENGINE_PATH}")
        logger = trt.Logger(trt.Logger.WARNING)
        with open(_TRT_ENGINE_PATH, "rb") as f:
            runtime = trt.Runtime(logger)
            self._engine = runtime.deserialize_cuda_engine(f.read())
        self._context = self._engine.create_execution_context()
        self._stream = torch.cuda.Stream()

        # Pre-allocate device buffers
        self._d_input = torch.empty(1, 3, _IMG_SIZE, _IMG_SIZE, dtype=torch.float32, device="cuda")
        self._d_da = torch.empty(1, 2, _IMG_SIZE, _IMG_SIZE, dtype=torch.float32, device="cuda")
        self._d_ll = torch.empty(1, 2, _IMG_SIZE, _IMG_SIZE, dtype=torch.float32, device="cuda")

        self._context.set_tensor_address("input", self._d_input.data_ptr())
        self._context.set_tensor_address("drivable", self._d_da.data_ptr())
        self._context.set_tensor_address("lane", self._d_ll.data_ptr())

        self._use_trt = True

    def _init_pytorch(self) -> None:
        _ensure_yolop_repo()
        _ensure_weights()

        repo_str = str(_YOLOP_REPO_DIR)
        if repo_str not in sys.path:
            sys.path.insert(0, repo_str)

        from lib.config import cfg
        from lib.models import get_net

        print(f"YOLOP using PyTorch on {self.device}")
        self.model = get_net(cfg)
        checkpoint = torch.load(_WEIGHTS_PATH, map_location=self.device, weights_only=False)
        self.model.load_state_dict(checkpoint["state_dict"])
        self.model.to(self.device)
        self.model.eval()

    def detect(self, frame_bgr: np.ndarray) -> YOLOPResult:
        tensor, img_h, img_w, pad_h, pad_w = _preprocess(frame_bgr)

        if self._use_trt:
            return self._detect_trt(tensor, img_h, img_w, pad_h, pad_w)
        return self._detect_pytorch(tensor, img_h, img_w, pad_h, pad_w)

    def _detect_trt(
        self, tensor: torch.Tensor, img_h: int, img_w: int, pad_h: float, pad_w: float
    ) -> YOLOPResult:
        self._d_input.copy_(tensor)
        self._context.execute_async_v3(stream_handle=self._stream.cuda_stream)
        self._stream.synchronize()
        return _postprocess(self._d_da, self._d_ll, img_h, img_w, pad_h, pad_w)

    def _detect_pytorch(
        self, tensor: torch.Tensor, img_h: int, img_w: int, pad_h: float, pad_w: float
    ) -> YOLOPResult:
        inputs = tensor.to(self.device)
        with torch.no_grad():
            _det_out, da_seg_out, ll_seg_out = self.model(inputs)
        return _postprocess(da_seg_out, ll_seg_out, img_h, img_w, pad_h, pad_w)
