---
name: jetson-deploy
description: Deploying and benchmarking on Jetson Orin Nano. Use when deploying code to Jetson, running TensorRT inference, monitoring GPU usage, or benchmarking performance.
---

# Jetson Orin Nano Deployment

## Connecting to Jetson

```bash
ssh jetson-nano   # Via Tailscale
# or
ssh charlie@100.95.16.119
```

The Jetson hostname is `charlie-jetson-orin-nano`.

## Device Specs

- **Model:** Jetson Orin Nano (8GB)
- **JetPack:** 6 (L4T R36.4.4)
- **CUDA:** 12.6
- **TensorRT:** 10.x
- **Python:** 3.10 (system)
- **RAM:** 8GB shared CPU/GPU

## Deploying Code

```bash
# From Mac: deploy latest code
make deploy-jetson

# This runs: ssh jetson-nano 'cd ~/couch-vision && git pull'
```

The repo is cloned at `~/couch-vision/` on the Jetson.

## Running the Perception Stack

```bash
ssh jetson-nano
cd ~/couch-vision

# Run with bag file
make full-stack BAG=bags/walk_around_university_all_data.mcap

# First run is slow — TRT engines auto-export (~10 min each)
# Subsequent runs start in seconds
```

## TensorRT Engine Auto-Export

On first CUDA run, TensorRT engines are built automatically:

| Model | Export Time | Engine Size |
|-------|------------|-------------|
| YOLOv8n | ~10 min | ~9 MB (INT8) |
| YOLOP | ~8 min | ~13 MB (FP16) |

Engines are saved to `perception/weights/` (volume-mounted). Delete `*.engine` to force re-export.

## Monitoring Performance

### tegrastats (GPU/Memory/Power)

```bash
# Real-time monitoring
tegrastats --interval 1000

# Log to file
tegrastats --interval 1000 > /tmp/tegrastats.log &
```

Output format:
```
RAM 2100/7620MB (lfb 1x4MB) SWAP 0/3810MB CPU [38%@1510,37%@1510,...]
GR3D_FREQ 99%@624 VIC_FREQ 0%@115 APE 174 CV0@47.3C CPU@48.8C SOC2@46.1C
SOC0@45.9C CV1@46.5C GPU@46.7C tj@48.8C SOC1@47.5C CV2@46.5C VDD_IN 5939mW
VDD_CPU_GPU_CV 2016mW VDD_SOC 1612mW
```

Key metrics:
- `RAM` — Memory usage
- `GR3D_FREQ 99%` — GPU utilization (target: >90% during inference)
- `CPU [38%@1510]` — CPU utilization per core
- `tj@48.8C` — Junction temperature (throttles at 97°C)
- `VDD_IN 5939mW` — Total power draw

### jtop (interactive)

```bash
sudo jtop
```

Interactive TUI showing GPU, memory, power, temperature. Press `q` to quit.

## Benchmarking

### Quick benchmark

```bash
ssh jetson-nano
cd ~/couch-vision

# Start tegrastats in background
tegrastats --interval 1000 > /tmp/tegrastats.log &

# Run perception for 90 seconds
timeout 90 make full-stack BAG=bags/walk_around_university_all_data.mcap

# Check logs for FPS
docker compose -f perception/docker-compose.nav2.yml logs | grep "FPS"

# Check tegrastats
tail -20 /tmp/tegrastats.log

# Stop tegrastats
pkill tegrastats
```

### Expected Performance

| Metric | Value |
|--------|-------|
| Perception FPS | 12-17 |
| GPU utilization | ~99% |
| RAM usage | ~2.1 / 7.6 GB |
| Power draw | ~6W total |
| Temperature | ~47°C |

## Docker on Jetson

Jetson uses nvidia container runtime:

```bash
# Verify nvidia runtime
docker info | grep -i runtime

# Run with GPU access (automatic via docker-compose.yml)
docker run --runtime=nvidia ...
```

The `docker-compose.nav2.yml` automatically uses the nvidia runtime on Jetson (detected via platform).

## Common Issues

### Permission denied on Docker socket

```bash
sudo usermod -aG docker $USER
# Log out and back in
```

### TensorRT engine incompatible

Delete engines and re-export:
```bash
rm perception/weights/*.engine
make full-stack BAG=...  # Will re-export
```

### Out of memory

- Close other applications
- Reduce batch size (already 1)
- Use `fast.yaml` config (skips YOLOP)

### Slow first run

Expected — TRT engine export takes ~10 min per model. Subsequent runs are fast.

### Can't SSH to Jetson

Check Tailscale:
```bash
tailscale status
# Jetson should show as "charlie-jetson-orin-nano"
```

## File Locations on Jetson

| Path | Contents |
|------|----------|
| `~/couch-vision/` | This repo |
| `~/ros2_jazzy/` | ROS2 Jazzy (built from source) |
| `/tmp/tegrastats.log` | Performance logs |
| `perception/weights/` | Model weights + TRT engines |

## Power Modes

Jetson Orin Nano supports different power modes:

```bash
# Check current mode
sudo nvpmodel -q

# Set to max performance (15W)
sudo nvpmodel -m 0
sudo jetson_clocks

# Set to power-saving (7W)
sudo nvpmodel -m 1
```

For benchmarking, use max performance mode.
