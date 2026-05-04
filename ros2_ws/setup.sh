# !/bin/bash
# set -e

PLATFORM="humble"

echo "=== Cleaning old build artifacts ==="
rm -rf build install log

echo "=== Removing all __pycache__ and .pyc files ==="
find src -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
find src -name "*.pyc" -delete 2>/dev/null || true
find src -name "*.pyo" -delete 2>/dev/null || true

echo "=== Enabling post-link scripts (if needed) ==="
pixi config set --local run-post-link-scripts insecure

echo "=== Installing/updating $PLATFORM environment ==="
pixi install -e $PLATFORM

echo "=== Building ROS 2 workspace ==="
pixi run -e $PLATFORM build

echo "=== Build finished! ==="
echo "To start working, run:"
echo "    pixi shell -e $PLATFORM"
echo "Then launch SLAM with:"
echo "    ros2 launch mapping nav2.slam.launch.py"