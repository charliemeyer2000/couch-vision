#!/bin/bash
# SLAM Backend Benchmark Script for Jetson
# Usage: ./benchmark_slam.sh [duration_seconds]

set -e

BAG="bags/2026-01-29_12-10-44/walk_around_university_all_data.mcap"
DURATION=${1:-60}
RESULTS_DIR="/tmp/slam_benchmark_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$RESULTS_DIR"

echo "=== SLAM Backend Benchmark ==="
echo "Bag: $BAG"
echo "Duration: ${DURATION}s per backend"
echo "Results: $RESULTS_DIR"
echo ""

run_benchmark() {
    local backend=$1
    echo "========================================"
    echo "Testing SLAM_BACKEND=$backend"
    echo "========================================"

    # Start tegrastats in background
    tegrastats --interval 500 > "$RESULTS_DIR/tegrastats_${backend}.log" 2>&1 &
    TEGRA_PID=$!

    # Small delay to let tegrastats start
    sleep 2

    # Run full-stack, capture output
    timeout "$DURATION" make full-stack BAG="$BAG" SLAM_BACKEND="$backend" 2>&1 | tee "$RESULTS_DIR/fullstack_${backend}.log" || true

    # Stop tegrastats
    kill $TEGRA_PID 2>/dev/null || true
    sleep 1

    echo ""
    echo "=== Results for $backend ==="

    # Extract and calculate FPS
    FPS_VALUES=$(grep -oP '[\d.]+(?= FPS)' "$RESULTS_DIR/fullstack_${backend}.log" || echo "")
    if [ -n "$FPS_VALUES" ]; then
        AVG_FPS=$(echo "$FPS_VALUES" | awk '{sum+=$1; count++} END {if(count>0) printf "%.1f", sum/count; else print "N/A"}')
        MIN_FPS=$(echo "$FPS_VALUES" | sort -n | head -1)
        MAX_FPS=$(echo "$FPS_VALUES" | sort -n | tail -1)
        echo "FPS: avg=$AVG_FPS min=$MIN_FPS max=$MAX_FPS"
    else
        echo "FPS: No data"
    fi

    # Frame count
    FRAME_COUNT=$(grep -oP 'Frame \d+' "$RESULTS_DIR/fullstack_${backend}.log" | tail -1 | grep -oP '\d+' || echo "0")
    echo "Frames processed: $FRAME_COUNT"

    # Parse tegrastats for GPU/memory
    if [ -f "$RESULTS_DIR/tegrastats_${backend}.log" ]; then
        echo ""
        echo "Resource usage (from tegrastats):"

        # RAM usage - extract and average
        RAM_USAGE=$(grep -oP 'RAM \d+/\d+MB' "$RESULTS_DIR/tegrastats_${backend}.log" | head -20 | \
            sed 's/RAM //' | cut -d'/' -f1 | awk '{sum+=$1; count++} END {if(count>0) printf "%.0f", sum/count}')
        RAM_TOTAL=$(grep -oP 'RAM \d+/\d+MB' "$RESULTS_DIR/tegrastats_${backend}.log" | head -1 | \
            sed 's/RAM //' | cut -d'/' -f2 | sed 's/MB//')
        echo "  RAM: ${RAM_USAGE}/${RAM_TOTAL} MB"

        # GPU usage
        GPU_USAGE=$(grep -oP 'GR3D_FREQ \d+%' "$RESULTS_DIR/tegrastats_${backend}.log" | head -20 | \
            grep -oP '\d+' | awk '{sum+=$1; count++} END {if(count>0) printf "%.0f", sum/count}')
        echo "  GPU: ${GPU_USAGE}%"

        # CPU usage
        CPU_USAGE=$(grep -oP 'CPU \[.*?\]' "$RESULTS_DIR/tegrastats_${backend}.log" | head -1)
        echo "  CPU: $CPU_USAGE"
    fi

    echo ""
}

# Cleanup any running containers
docker compose -f perception/docker-compose.nav2.yml down 2>/dev/null || true

echo "Starting benchmarks..."
echo ""

# Test 1: No SLAM (baseline)
run_benchmark "none"

# Test 2: RTAB-Map
run_benchmark "rtabmap"

# Summary
echo "========================================"
echo "BENCHMARK SUMMARY"
echo "========================================"
echo ""
echo "Results saved to: $RESULTS_DIR"
echo ""

# Create summary file
cat > "$RESULTS_DIR/summary.txt" << EOF
SLAM Backend Benchmark Results
==============================
Date: $(date)
Bag: $BAG
Duration: ${DURATION}s per test

Backend: none (no SLAM)
$(grep -E '(FPS:|Frames|RAM:|GPU:)' "$RESULTS_DIR/fullstack_none.log" 2>/dev/null || echo "No data")

Backend: rtabmap
$(grep -E '(FPS:|Frames|RAM:|GPU:)' "$RESULTS_DIR/fullstack_rtabmap.log" 2>/dev/null || echo "No data")
EOF

cat "$RESULTS_DIR/summary.txt"
