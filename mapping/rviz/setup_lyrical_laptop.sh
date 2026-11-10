#!/usr/bin/env bash
# One-time ROOT setup on a laptop: ROS 2 Lyrical, the robot's distro and the only ROS there
# (2026-11-01; the RViz laptop, then the Isaac laptop).
#
# Why: the robot (Jetson + Pi 5) is on Lyrical (Fast DDS 3.6) since 2026-10-25; this
# laptop was still Jazzy (Fast DDS 2.14). Small topics crossed (scan, pose, maps), the
# big ones did not: measured 2026-11-01, the camera (5.3 Hz on the Pi 5) and the global
# costmap (0.9 Hz) reached RViz here at 0 Hz -- the same fault that blinded the Pi 5's
# brain until it moved to Lyrical (rover OPEN_ISSUES.md, 2026-10-25).
#
# What: NVIDIA's Lyrical repo (the Jetson image and the Pi 5 use it; plus packages.ros.org
# when missing, for two of its Python deps), RViz with its
# default displays (all rover_live.rviz uses), and the Jetson's DDS socket buffers.
# Any ROS 2 Jazzy is then removed (owner, 2026-11-01: one ROS everywhere, as on the Pi 5).
# rover_live.sh and kino_sim's ./sim use Lyrical only.
#
# Run on the laptop:  sudo bash ~/setup_lyrical_laptop.sh
# Then on the Jetson: ./rover view --restart
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run it with sudo"; exit 1; }

echo "[1/2] NVIDIA ROS Lyrical repo for Ubuntu 24.04 amd64 + RViz"
# The official ROS source too: Lyrical's python3-catkin-pkg-modules and
# python3-rosdistro-modules come only from packages.ros.org. A laptop that never
# had system ROS (the Isaac laptop, 2026-11-01) lacks it: "held broken packages".
if ! grep -rqs "packages.ros.org/ros2" /etc/apt/sources.list.d/; then
  r=/usr/share/keyrings/ros-archive-keyring.gpg
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$r"
  echo "deb [arch=amd64 signed-by=$r] http://packages.ros.org/ros2/ubuntu noble main" \
    > /etc/apt/sources.list.d/ros2.list
fi
k=/usr/share/keyrings/nvidia-isaac-ros.gpg
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | gpg --dearmor --yes -o "$k"
echo "deb [arch=amd64 signed-by=$k] https://isaac.download.nvidia.com/isaac-ros/ubuntu/main noble main" \
  > /etc/apt/sources.list.d/nvidia-ros-lyrical.list
apt-get update
apt-get install -y --no-install-recommends \
  ros-lyrical-ros-base ros-lyrical-rviz2 ros-lyrical-rviz-default-plugins \
  ros-lyrical-compressed-image-transport   # the camera display: without it RViz takes ".../compressed" for a raw Image topic

if dpkg -l 'ros-jazzy-*' 2>/dev/null | grep -q '^ii'; then
  echo "      removing ROS 2 Jazzy (Lyrical is the only ROS here)"
  apt-get purge -y -q 'ros-jazzy-*' > /dev/null
  apt-get autoremove -y -q --purge > /dev/null
fi

echo "[2/2] DDS socket buffers as on the Jetson and the Pi 5 (this laptop had 208 KB default)"
printf "net.core.rmem_max=67108864\nnet.core.rmem_default=8388608\n" > /etc/sysctl.d/60-kino-dds.conf
sysctl --system > /dev/null

echo "done. /opt/ros: $(ls /opt/ros | tr '\n' ' ')-- now on the Jetson: ./rover view --restart"
