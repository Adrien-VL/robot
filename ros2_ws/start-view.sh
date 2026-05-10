#!/bin/bash

echo "========================================"
echo "Starting Robot Vis"
echo "========================================"

echo "Starting Foxglove Bridge on ws://0.0.0.0:9090"
ros2 run foxglove_bridge foxglove_bridge \
  --ros-args \
  -p port:=9090 \
  -p address:="0.0.0.0"

echo "Robot vis started!"