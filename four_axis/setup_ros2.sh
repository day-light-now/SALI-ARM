#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
cd "$(dirname "$0")"
if ! id xuyu >/dev/null 2>&1; then
  useradd -m -s /bin/bash -G sudo,dialout,video xuyu
fi
install -m 644 wsl.conf /etc/wsl.conf
dpkg -i ../wsl_backups/ros2-apt-source_1.3.0.noble_all.deb
apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=30 update
apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=30 install -y \
  ros-jazzy-desktop ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-xacro python3-colcon-common-extensions python3-rosdep \
  build-essential git
install -m 644 ros_profile.sh /etc/profile.d/slaihand_ros.sh
mkdir -p /home/xuyu/slaihand_ros2_ws
cp -a src /home/xuyu/slaihand_ros2_ws/
chown -R xuyu:xuyu /home/xuyu/slaihand_ros2_ws
echo 'ROS 2 installation complete.'
