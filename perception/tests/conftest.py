"""Shared fixtures for perception benchmarks."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import torch


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        "--bag",
        default=None,
        help="Path to .mcap bag file for integration tests",
    )
    parser.addoption(
        "--weights-dir",
        default=str(Path(__file__).resolve().parent.parent / "weights"),
        help="Directory containing model weights",
    )


@pytest.fixture(scope="session")
def device() -> str:
    return "cuda" if torch.cuda.is_available() else "cpu"


@pytest.fixture(scope="session")
def weights_dir(request: pytest.FixtureRequest) -> Path:
    return Path(request.config.getoption("--weights-dir"))


@pytest.fixture(scope="session")
def bag_path(request: pytest.FixtureRequest) -> Path | None:
    val = request.config.getoption("--bag")
    if val is None:
        return None
    p = Path(val)
    if not p.exists():
        pytest.skip(f"Bag file not found: {p}")
    return p


@pytest.fixture
def dummy_image() -> np.ndarray:
    return np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)


@pytest.fixture
def dummy_depth() -> np.ndarray:
    return np.random.rand(192, 256).astype(np.float32) * 5.0


@pytest.fixture
def dummy_intrinsics():
    from couch_perception.bag_reader import CameraIntrinsics

    K = np.array([
        [500.0, 0.0, 320.0],
        [0.0, 500.0, 240.0],
        [0.0, 0.0, 1.0],
    ])
    return CameraIntrinsics(
        width=640,
        height=480,
        K=K,
        D=np.zeros(5),
        distortion_model="plumb_bob",
        frame_id="camera",
    )


@pytest.fixture
def dummy_synced_frame(dummy_image, dummy_depth, dummy_intrinsics):
    from couch_perception.bag_reader import SyncedFrame

    return SyncedFrame(
        timestamp=0.0,
        image=dummy_image,
        depth=dummy_depth,
        intrinsics=dummy_intrinsics,
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
