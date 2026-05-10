#!/bin/bash

echo "========================================"
echo "Stopping Robot Vis"
echo "========================================"

echo "Stopping Foxglove Bridge..."
pkill -INT -f foxglove_bridge 2>/dev/null || true

echo "All processes stopped."