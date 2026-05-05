#!/bin/bash
set -e

ROS_DISTRO=humble
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

echo "=== ROS 2 ${ROS_DISTRO} robot setup (apt) ==="

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "This script only supports Linux."
  exit 1
fi

if ! command -v apt >/dev/null 2>&1; then
  echo "apt not found. This script expects Ubuntu 22.04."
  exit 1
fi

echo "=== Updating apt package lists ==="
sudo apt update

echo "=== Installing base tools ==="
sudo apt install -y \
  curl \
  gnupg2 \
  lsb-release \
  software-properties-common \
  build-essential \
  cmake \
  pkg-config \
  make \
  ninja-build \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-argcomplete

if ! dpkg -s ros-${ROS_DISTRO}-ros-base >/dev/null 2>&1; then
  echo "=== Installing ROS 2 ${ROS_DISTRO} robot packages ==="
  sudo apt install -y \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-launch \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher
else
  echo "=== ROS 2 ${ROS_DISTRO} base already installed ==="
  sudo apt install -y \
    ros-${ROS_DISTRO}-launch \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  echo "=== Initializing rosdep ==="
  sudo rosdep init
fi

echo "=== Updating rosdep ==="
rosdep update || true

echo "=== Cleaning old build artifacts ==="
rm -rf build install log

echo "=== Removing all __pycache__ and .pyc files ==="
find src -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
find src -name "*.pyc" -delete 2>/dev/null || true
find src -name "*.pyo" -delete 2>/dev/null || true

echo "=== Installing workspace dependencies with rosdep ==="
source "${ROS_SETUP}"
rosdep install --from-paths src --ignore-src -r -y

echo "=== Building ROS 2 workspace ==="
colcon build \
  --symlink-install \
  --cmake-args \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo "=== Ensuring ROS setup is available in bash ==="
if ! grep -q "source ${ROS_SETUP}" ~/.bashrc; then
  echo "source ${ROS_SETUP}" >> ~/.bashrc
fi

WORKSPACE_SETUP="$(pwd)/install/setup.bash"
if ! grep -q "source ${WORKSPACE_SETUP}" ~/.bashrc; then
  echo "source ${WORKSPACE_SETUP}" >> ~/.bashrc
fi

echo "=== Build finished! ==="
echo "Open a new shell or run:"
echo "    source ${ROS_SETUP}"
echo "    source install/setup.bash"
echo ""
echo "Then launch SLAM with:"
echo "    ros2 launch mapping nav2.slam.launch.py"