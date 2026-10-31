#!/usr/bin/env bash
# One-time ROOT setup on the RViz laptop: ROS 2 Lyrical beside Jazzy (2026-11-01).
#
# Why: the robot (Jetson + Pi 5) is on Lyrical (Fast DDS 3.6) since 2026-10-25; this
# laptop was still Jazzy (Fast DDS 2.14). Small topics crossed (scan, pose, maps), the
# big ones did not: measured 2026-11-01, the camera (5.3 Hz on the Pi 5) and the global
# costmap (0.9 Hz) reached RViz here at 0 Hz -- the same fault that blinded the Pi 5's
# brain until it moved to Lyrical (rover OPEN_ISSUES.md, 2026-10-25).
#
# What: NVIDIA's Lyrical repo (the Jetson image and the Pi 5 use it), RViz with its
# default displays (all rover_live.rviz uses), and the Jetson's DDS socket buffers.
# Jazzy stays installed as the rollback; nothing is removed. rover_live.sh picks
# Lyrical when it is there.
#
# Run on the laptop:  sudo bash ~/setup_lyrical_laptop.sh
# Then on the Jetson: ./rover view --restart
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run it with sudo"; exit 1; }

echo "[1/2] NVIDIA ROS Lyrical repo for Ubuntu 24.04 amd64 + RViz"
k=/usr/share/keyrings/nvidia-isaac-ros.gpg
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | gpg --dearmor --yes -o "$k"
echo "deb [arch=amd64 signed-by=$k] https://isaac.download.nvidia.com/isaac-ros/ubuntu/main noble main" \
  > /etc/apt/sources.list.d/nvidia-ros-lyrical.list
apt-get update
apt-get install -y --no-install-recommends \
  ros-lyrical-ros-base ros-lyrical-rviz2 ros-lyrical-rviz-default-plugins \
  ros-lyrical-compressed-image-transport   # the camera display: without it RViz takes ".../compressed" for a raw Image topic

echo "[2/2] DDS socket buffers as on the Jetson and the Pi 5 (this laptop had 208 KB default)"
printf "net.core.rmem_max=67108864\nnet.core.rmem_default=8388608\n" > /etc/sysctl.d/60-kino-dds.conf
sysctl --system > /dev/null

echo "done. /opt/ros: $(ls /opt/ros | tr '\n' ' ')-- now on the Jetson: ./rover view --restart"
