#!/bin/bash
# ROS2 Jazzy setup for macOS Apple Silicon
# Reference: https://github.com/IOES-Lab/ROS2_Jazzy_MacOS_Native_AppleSilicon

set -e

ROS2_WS="${ROS2_WS:-$HOME/ros2_jazzy}"
ROS2_VENV="${ROS2_VENV:-$HOME/.ros2_venv}"

echo "=== CouchVision ROS2 Setup for macOS ==="
echo ""
echo "This script sets up ROS2 Jazzy natively on Apple Silicon."
echo "Expected locations:"
echo "  ROS2 workspace: $ROS2_WS"
echo "  Python venv:    $ROS2_VENV"
echo ""

# Check architecture
if [[ "$(uname -m)" != "arm64" ]]; then
    echo "Error: This script is for Apple Silicon (arm64) only."
    echo "For Intel Macs, consider using Docker instead."
    exit 1
fi

# Check if ROS2 is already installed
if [[ -f "$ROS2_WS/install/setup.bash" ]]; then
    echo "ROS2 Jazzy appears to be installed at $ROS2_WS"
    echo ""
    echo "To source your environment, add to your ~/.zshrc or ~/.bashrc:"
    echo "  source $ROS2_WS/install/setup.bash"
    echo ""

    # Verify it works
    if source "$ROS2_WS/install/setup.bash" 2>/dev/null && command -v ros2 &>/dev/null; then
        echo "ROS2 is working! Version:"
        ros2 --version
        exit 0
    else
        echo "Warning: ROS2 install found but 'ros2' command not available."
        echo "You may need to reinstall or check your Python venv."
    fi
fi

echo "ROS2 Jazzy not found. Would you like to install it?"
echo ""
echo "IMPORTANT: Installation requires:"
echo "  - XCode 16.2 (NOT the default Tahoe version)"
echo "  - ~45 minutes for full install"
echo "  - ~10GB disk space"
echo ""
echo "See: https://github.com/IOES-Lab/ROS2_Jazzy_MacOS_Native_AppleSilicon"
echo ""

read -p "Install ROS2 Jazzy now? [y/N] " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Skipping ROS2 installation."
    echo ""
    echo "To install manually later, run:"
    echo '  /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/IOES-Lab/ROS2_Jazzy_MacOS_Native_AppleSilicon/main/install.sh)"'
    exit 0
fi

echo ""
echo "Starting ROS2 Jazzy installation..."
echo "This will take approximately 30-45 minutes."
echo ""

/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/IOES-Lab/ROS2_Jazzy_MacOS_Native_AppleSilicon/main/install.sh)"

echo ""
echo "=== ROS2 Installation Complete ==="
echo ""
echo "Add to your shell profile (~/.zshrc or ~/.bashrc):"
echo "  source $ROS2_WS/install/setup.bash"
echo ""
