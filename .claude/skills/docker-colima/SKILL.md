---
name: docker-colima
description: Docker on macOS via Colima. Use when running Docker containers, debugging container issues, or working with docker-compose on Mac.
---

# Docker on macOS (Colima)

## Why Colima, Not Docker Desktop

Docker Desktop on macOS has licensing restrictions and performance issues. Colima provides a lightweight alternative using Lima VMs.

**CouchVision uses Colima for all Docker operations on Mac.**

## Setup

```bash
# Install Colima (if not already installed)
brew install colima docker docker-compose

# Start Colima with sufficient resources
colima start --cpu 4 --memory 8 --disk 50

# Verify
docker ps
```

## Starting Colima

**Before any Docker command on Mac, ensure Colima is running:**

```bash
colima status        # Check if running
colima start         # Start if stopped
```

If `docker` commands hang or fail with "Cannot connect to Docker daemon", Colima is not running.

## Resource Allocation

The perception stack needs sufficient memory:

```bash
# Stop and restart with more resources
colima stop
colima start --cpu 4 --memory 8 --disk 50
```

**Recommended minimums:**
- CPU: 4 cores (perception is CPU-bound on Mac due to no CUDA)
- Memory: 8GB (PyTorch models need ~2GB, plus overhead)
- Disk: 50GB (Docker images are large)

## Common Issues

### "Cannot connect to the Docker daemon"

```bash
colima start
# Wait for "Done" message, then retry
```

### Container OOM killed

Increase Colima memory:
```bash
colima stop
colima start --memory 12
```

### Very slow PyTorch loading

Expected on Mac — PyTorch under amd64 emulation (via Rosetta 2) is slow. First model load takes 60-90 seconds. Subsequent runs are faster due to caching.

### Port already in use

```bash
# Find what's using port 8765 (Foxglove)
lsof -i :8765
# Kill it or use a different port
```

### Colima VM corrupted

```bash
colima delete
colima start --cpu 4 --memory 8 --disk 50
# Will need to rebuild images
```

## Architecture: amd64 vs arm64

CouchVision Docker images are **amd64 only** (x86_64). On Apple Silicon Macs (arm64), Colima runs them via Rosetta 2 emulation.

**Implications:**
- Slower than native arm64 (~2-5x)
- No GPU acceleration (Mac MPS not available inside amd64 container)
- CUDA not available (expected — Mac has no NVIDIA GPU)

This is fine for development/testing. Production runs on Jetson (native arm64 + CUDA).

## Docker Compose

The perception stack uses `docker-compose`:

```bash
# Build and run (handles Colima automatically via Makefile)
make full-stack BAG=bags/your_bag.mcap

# Manual docker-compose
docker compose -f perception/docker-compose.nav2.yml build
docker compose -f perception/docker-compose.nav2.yml up

# View logs
docker compose -f perception/docker-compose.nav2.yml logs -f

# Stop
docker compose -f perception/docker-compose.nav2.yml down
```

## Volume Mounts

The docker-compose.yml mounts:
- `./bags:/perception/bags` — bag files
- `./weights:/perception/weights` — model weights + TRT engines
- `./config:/perception/config` — YAML configs (hot-reloadable)

Changes to mounted files are visible immediately without rebuilding.

## Debugging Inside Container

```bash
# Shell into running container
docker exec -it perception-nav2-planner-1 bash

# Run ROS2 commands inside
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo /map
```

## Colima vs Docker Desktop

| Feature | Colima | Docker Desktop |
|---------|--------|----------------|
| License | Free (open source) | Paid for large orgs |
| Performance | Good | Good |
| GPU passthrough | No | No (Mac) |
| Resource control | CLI flags | GUI |
| Stability | Good | Good |
