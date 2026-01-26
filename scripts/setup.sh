#!/bin/bash
# CouchVision Development Environment Setup
# Run this once to set up everything needed for development

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=== CouchVision Development Setup ==="
echo ""
echo "Project root: $PROJECT_ROOT"
echo ""

# 1. Check for Homebrew
echo "[1/8] Checking Homebrew..."
if ! command -v brew &>/dev/null; then
    echo "Error: Homebrew not found. Install from https://brew.sh"
    exit 1
fi
echo "  Homebrew OK"

# 2. Check for uv (Python package manager)
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
if ! command -v swiftlint &>/dev/null; then
    echo "  Installing SwiftLint..."
    brew install swiftlint
fi
echo "  SwiftLint OK: $(swiftlint version)"

# 5. Check for SwiftFormat
echo ""
echo "[5/8] Checking SwiftFormat..."
if ! command -v swiftformat &>/dev/null; then
    echo "  Installing SwiftFormat..."
    brew install swiftformat
fi
echo "  SwiftFormat OK: $(swiftformat --version)"

# 6. Set up Python bridge
echo ""
echo "[6/8] Setting up Python bridge..."
cd "$PROJECT_ROOT/bridge"
uv sync
uv add --dev ruff
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

echo ""
echo "=== Setup Complete ==="
echo ""
echo "Quick start:"
echo "  1. make xcode        # Open Xcode, build to iPhone"
echo "  2. make bridge       # Start ROS2 bridge (requires ROS2)"
echo "  3. make topics       # List ROS2 topics"
echo ""
echo "Run 'make help' for all commands."
echo ""
