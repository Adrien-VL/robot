#!/bin/bash
# set -e

PLATFORM="humble"
ENV_TYPE="host"   # default = host (development machine)

# Parse command line arguments
while [[ "$#" -gt 0 ]]; do
  case $1 in
    --robot)
      ENV_TYPE="robot"
      ;;
    --host)
      ENV_TYPE="host"
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 [--host | --robot]"
      exit 1
      ;;
  esac
  shift
done

ENV_NAME="${PLATFORM}-${ENV_TYPE}"

echo "=== Using environment: $ENV_NAME ($ENV_TYPE) ==="

echo "=== Cleaning old build artifacts ==="
rm -rf build install log

echo "=== Removing all __pycache__ and .pyc files ==="
find src -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
find src -name "*.pyc" -delete 2>/dev/null || true
find src -name "*.pyo" -delete 2>/dev/null || true

echo "=== Enabling post-link scripts (if needed) ==="
pixi config set --local run-post-link-scripts insecure

echo "=== Installing/updating $ENV_NAME environment ==="
pixi install -e $ENV_NAME

echo "=== Building ROS 2 workspace ==="
pixi run -e $ENV_NAME build

echo "=== Build finished! ==="
echo "To activate this environment, run:"
echo "    pixi shell -e $ENV_NAME"
echo ""
if [[ "$ENV_TYPE" == "robot" ]]; then
  echo "Then launch SLAM with:"
  echo "    ros2 launch mapping nav2.slam.launch.py"
else
  echo "Development environment ready with RViz, Gazebo, etc."
fi