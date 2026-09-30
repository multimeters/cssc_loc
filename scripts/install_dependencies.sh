#!/usr/bin/env bash
set -eo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo '请先安装 ROS 2 Humble（Ubuntu 22.04）。' >&2
  exit 2
fi
if [[ "$EUID" -eq 0 ]]; then
  ELEVATE=()
else
  ELEVATE=(sudo)
fi
"${ELEVATE[@]}" apt-get update
"${ELEVATE[@]}" apt-get install -y build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep python3-yaml \
  python3-numpy python3-scipy python3-matplotlib python3-pytest \
  ros-humble-rosbag2-storage-default-plugins
source /opt/ros/humble/setup.bash
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  "${ELEVATE[@]}" rosdep init
fi
rosdep update --rosdistro humble
rosdep install --from-paths "$ROOT_DIR/src" --ignore-src --rosdistro humble \
  -t build -t buildtool -t build_export -t buildtool_export -t exec -y
