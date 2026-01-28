#!/bin/bash
# CouchVision Development Environment Setup

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=== CouchVision Development Setup ==="
echo ""

# Detect platform
OS="$(uname -s)"
case "$OS" in
    Darwin) PLATFORM="macos" ;;
    Linux)  PLATFORM="linux" ;;
    *)      PLATFORM="unknown" ;;
esac

echo "Platform: $PLATFORM"
echo "Project root: $PROJECT_ROOT"
echo ""

# ============================================
# macOS Setup
# ============================================
if [[ "$PLATFORM" == "macos" ]]; then
    echo "Running macOS setup (iOS app + bridge)..."
    echo ""

    # 1. Check for Homebrew
    echo "[1/8] Checking Homebrew..."
    if ! command -v brew &>/dev/null; then
        echo "Error: Homebrew not found. Install from https://brew.sh"
        exit 1
    fi
    echo "  Homebrew OK"

    # 2. Check for uv
    echo ""
    echo "[2/8] Checking uv..."
    if ! command -v uv &>/dev/null; then
        echo "  Installing uv..."
        brew install uv
    fi
    echo "  uv OK: $(uv --version)"

    # 3. Check for xcodegen
    echo ""
    echo "[3/8] Checking xcodegen..."
    if ! command -v xcodegen &>/dev/null; then
        echo "  Installing xcodegen..."
        brew install xcodegen
    fi
    echo "  xcodegen OK: $(xcodegen --version)"

    # 4. Check for SwiftLint
    echo ""
    echo "[4/8] Checking SwiftLint..."
    brew upgrade swiftlint 2>/dev/null || brew install swiftlint
    echo "  SwiftLint OK: $(swiftlint version)"

    # 5. Check for SwiftFormat
    echo ""
    echo "[5/8] Checking SwiftFormat..."
    brew upgrade swiftformat 2>/dev/null || brew install swiftformat
    echo "  SwiftFormat OK: $(swiftformat --version)"

    # 6. Set up Python bridge
    echo ""
    echo "[6/8] Setting up Python bridge..."
    cd "$PROJECT_ROOT/bridge"
    uv sync
    echo "  Bridge dependencies installed"

    # 7. Install pre-commit hooks
    echo ""
    echo "[7/8] Setting up pre-commit hooks..."
    cd "$PROJECT_ROOT"
    if ! command -v pre-commit &>/dev/null; then
        echo "  Installing pre-commit..."
        brew install pre-commit
    fi
    pre-commit install
    echo "  Pre-commit hooks installed"

    # 8. Generate Xcode project
    echo ""
    echo "[8/8] Generating Xcode project..."
    cd "$PROJECT_ROOT"
    if [[ -f "project.yml" ]]; then
        xcodegen generate
        echo "  Xcode project generated"
    else
        echo "  Skipping (no project.yml found)"
    fi

    # ROS2 setup (optional)
    echo ""
    echo "=== ROS2 Setup (Optional) ==="
    echo ""
    echo "CouchVision requires ROS2 Jazzy to receive sensor data."
    echo ""
    read -p "Set up ROS2 now? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        "$SCRIPT_DIR/setup-ros2.sh"
    fi

# ============================================
# Linux Setup (bridge only)
# ============================================
elif [[ "$PLATFORM" == "linux" ]]; then
    echo "Running Linux setup (bridge only)..."
    echo ""
    echo "Note: The iOS app requires macOS + Xcode."
    echo "This setup configures the ROS2 bridge for receiving data."
    echo ""

    # 1. Check for Python
    echo "[1/4] Checking Python..."
    if ! command -v python3 &>/dev/null; then
        echo "Error: Python 3 not found. Install with: sudo apt install python3 python3-pip"
        exit 1
    fi
    echo "  Python OK: $(python3 --version)"

    # 2. Install uv
    echo ""
    echo "[2/4] Checking uv..."
    if ! command -v uv &>/dev/null; then
        echo "  Installing uv..."
        curl -LsSf https://astral.sh/uv/install.sh | sh
        export PATH="$HOME/.local/bin:$PATH"
    fi
    echo "  uv OK: $(uv --version)"

    # 3. Set up Python bridge
    echo ""
    echo "[3/4] Setting up Python bridge..."
    cd "$PROJECT_ROOT/bridge"
    uv sync
    echo "  Bridge dependencies installed"

    # 4. Check for ROS2
    echo ""
    echo "[4/4] Checking ROS2..."
    if command -v ros2 &>/dev/null; then
        echo "  ROS2 OK: $(ros2 --version 2>/dev/null | head -1)"
    elif [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
        echo "  ROS2 Jazzy found at /opt/ros/jazzy"
        echo "  Run: source /opt/ros/jazzy/setup.bash"
    elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
        echo "  ROS2 Humble found at /opt/ros/humble"
        echo "  Run: source /opt/ros/humble/setup.bash"
    else
        echo "  ROS2 not found."
        echo ""
        echo "  Install ROS2 Jazzy (Ubuntu 24.04):"
        echo "    https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"
        echo ""
        echo "  Install ROS2 Humble (Ubuntu 22.04):"
        echo "    https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html"
    fi

# ============================================
# Unknown Platform
# ============================================
else
    echo "Unsupported platform: $OS"
    echo ""
    echo "Supported platforms:"
    echo "  - macOS (full setup: iOS app + bridge)"
    echo "  - Linux (bridge only)"
    echo ""
    echo "For Windows, use WSL2 with Ubuntu."
    exit 1
fi

echo ""
echo "=== Setup Complete ==="
echo ""

if [[ "$PLATFORM" == "macos" ]]; then
    echo "Quick start:"
    echo "  1. make xcode        # Open Xcode, build to iPhone"
    echo "  2. make bridge       # Start ROS2 bridge"
    echo "  3. make topics       # List ROS2 topics"
elif [[ "$PLATFORM" == "linux" ]]; then
    echo "Quick start:"
    echo "  1. source /opt/ros/jazzy/setup.bash  # or humble"
    echo "  2. make bridge                       # Start ROS2 bridge"
    echo "  3. ros2 topic list                   # Verify topics"
fi
echo ""
echo "Run 'make help' for all commands."
