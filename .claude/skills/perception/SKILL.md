---
name: perception
description: CouchVision perception stack — YOLOv8 + YOLOP + TensorRT. Use when working on object detection, lane segmentation, model export, or running the perception node on Mac or Jetson.
---

# CouchVision Perception Stack

## Architecture

The perception stack lives in `perception/` and has two modes:

1. **Offline** (`make perception BAG=path.mcap`) — processes MCAP bag files, outputs annotated video + dashboard
2. **Live ROS2 node** (`make perception-node`) — subscribes to camera topic, publishes detections/overlay/masks

### Models

| Model | Purpose | Input | Output |
|-------|---------|-------|--------|
| YOLOv8n | Object detection | BGR frame | Bounding boxes + class + confidence |
| YOLOP | Drivable area + lane lines | BGR frame | Two binary masks |

### Key Files

- `src/couch_perception/yolov8_detector.py` — YOLOv8 wrapper. Auto-detects device (cuda > mps > cpu) and prefers `.engine` over `.pt` if available. Auto-exports TRT engine on first CUDA run (~10 min on Orin).
- `src/couch_perception/yolop_detector.py` — YOLOP wrapper. Clones hustvl/YOLOP repo on first run. Auto-exports TRT FP16 engine on first CUDA run (PyTorch → ONNX → TRT, ~8 min on Orin).
- `src/couch_perception/ros_node.py` — ROS2 node. Publishes to `/perception/detections`, `/perception/overlay/compressed`, `/perception/lane_mask`, `/perception/drivable_mask`.
- `src/couch_perception/runner.py` — Offline CLI entry point.
- `src/couch_perception/bag_reader.py` — MCAP bag reader. Streaming two-pass: scalar data in memory, image+depth streamed lazily. Critical for Jetson (avoids OOM on large bags).
- `src/couch_perception/frame_source.py` — `BagSource` (bag replay with pacing) + `LiveSource` (ROS2 subscriptions). Both yield `SensorStreams`.
- `scripts/export_tensorrt.py` — Export YOLOv8n to TensorRT INT8 engine (manual, usually not needed since auto-export exists).
- `scripts/benchmark.py` — Benchmark PyTorch vs TensorRT inference speed.

## Platform Differences

### Mac (darwin, Apple Silicon)

- **Python**: 3.12 (matches ROS2 Jazzy rclpy)
- **Torch**: CPU from `https://download.pytorch.org/whl/cpu`
- **Device**: `mps` (Metal Performance Shaders) — auto-detected
- **No TensorRT** — `.engine` files won't exist, uses `.pt` weights
- **ROS2**: Built from source at `~/ros2_jazzy/`, requires `source ~/ros2_jazzy/install/setup.zsh`

### Jetson Orin Nano (Linux aarch64)

- **Python**: 3.10 (system Python — NVIDIA only publishes cp310 CUDA torch wheels)
- **Torch**: CUDA from `https://pypi.jetson-ai-lab.io/jp6/cu126`
- **torch version pinned** to `<=2.8.0` because 2.9.1 requires `libcudss` not available on JetPack 6
- **numpy pinned** to `<2` because Jetson torch 2.8.0 was compiled against NumPy 1.x
- **Device**: `cuda` — auto-detected
- **TensorRT**: Export with `uv run python scripts/export_tensorrt.py`, creates `yolov8n.engine` (~5MB INT8)
- **ROS2**: Built from source at `~/ros2_jazzy/`
- **JetPack**: 6 (L4T R36.4.4), CUDA 12.6

### How Platform Resolution Works

`pyproject.toml` uses uv's `[tool.uv.sources]` with platform markers:

```toml
[tool.uv.sources]
torch = [
    { index = "pytorch-cpu", marker = "sys_platform == 'darwin'" },
    { index = "jetson-cuda", marker = "sys_platform == 'linux'" },
]
```

Each index is declared as `explicit = true` so it only serves torch/torchvision. `uv sync` resolves the correct torch automatically — no environment variables needed.

The Makefile auto-detects `PERC_PYTHON` (3.10 on Jetson, 3.12 on Mac) for venv creation.

## Common Operations

```bash
# Run offline perception on a bag
make perception BAG=bags/2026-01-29_12-10-44/walk_around_university_all_data.mcap

# Run live ROS2 perception node (auto-detects device)
make perception-node

# Override device or skip YOLOP
make perception-node ARGS="--device cpu --skip-yolop"

# Export TensorRT engine (Jetson only, requires CUDA)
cd perception && uv run python scripts/export_tensorrt.py

# Benchmark inference
cd perception && uv run python scripts/benchmark.py --pytorch yolov8n.pt --engine yolov8n.engine

# Run Docker (Jetson deployment)
make perception-docker
```

## Device Auto-Detection

`YOLOv8Detector` auto-detects the best device and prefers TensorRT:

1. If `yolov8n.engine` exists alongside `yolov8n.pt`, loads the engine (TensorRT)
2. If `--device` not specified: picks `cuda` > `mps` > `cpu`
3. On Jetson with TensorRT: **~57 FPS**. CUDA PyTorch: **~37 FPS**. CPU: **~2 FPS**.

## ROS2 Topics Published

| Topic | Type | QoS |
|-------|------|-----|
| `/perception/detections` | `vision_msgs/Detection2DArray` | Reliable, depth 10 |
| `/perception/overlay/compressed` | `sensor_msgs/CompressedImage` | Best effort, depth 1 |
| `/perception/lane_mask` | `sensor_msgs/Image` (mono8) | Best effort, depth 1 |
| `/perception/drivable_mask` | `sensor_msgs/Image` (mono8) | Best effort, depth 1 |

Subscribes to: `/iphone_charlie/camera/arkit/image/compressed` (configurable via `--topic`)

## Troubleshooting

### torch.cuda.is_available() returns False on Jetson

The venv has CPU-only torch from PyPI. Fix: `rm -rf .venv uv.lock && uv venv --python python3.10 --system-site-packages && uv sync`. The `[tool.uv.sources]` in pyproject.toml should pull CUDA torch from the Jetson index automatically.

### rclpy not found

ROS2 must be sourced before creating the venv: `source ~/ros2_jazzy/install/setup.bash` (or `.zsh` on Mac). Then recreate: `uv venv --python python3.10 --system-site-packages`. The `make perception-node` target handles this.

### TensorRT export fails with missing onnx

`onnx` and `onnxslim` are in pyproject.toml dependencies. Run `uv sync` to install them. Do NOT install `onnxruntime-gpu` — it doesn't exist for aarch64.

### NumPy crash on Jetson

Jetson torch 2.8.0 needs `numpy<2`. This is enforced via `sys_platform == 'linux'` marker in pyproject.toml.

### Jetson OOM on large bags

The streaming bag reader (`bag_reader.py`) fixed this. If you see OOM, check that `read_all_streams()` is being used (not the old `read_synced_frames` which loaded everything into memory). A 6.9GB bag now uses ~2.1GB RAM.

### TRT engines not persisting across container restarts

Engines must be saved to the volume-mounted `weights/` directory. `yolov8_detector.py` resolves `.pt` paths relative to `_WEIGHTS_DIR` so ultralytics downloads and exports into the mounted volume. If engines disappear, check that `./weights:/perception/weights` is in docker-compose.yml.
