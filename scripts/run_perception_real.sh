#!/bin/bash
# Launch LangRobo perception (D555 + cuVSLAM + nvblox + Nav2 + detections_3d),
# REAL profile, inside the Isaac ROS container. Run from the Jetson host:
#   docker exec -d isaac_ros /workspaces/isaac_ros-dev/src/langrobo_perception/scripts/run_perception_real.sh
# Prerequisites: D555 on the jumbo-frame switch (or direct ethernet), static
# IP set via rs-eth-config (DEPTH_CAMERA.md), servos centred (ESP32 boots so).
#
# Camera mount extrinsics: pass cam_x/cam_y/cam_z/cam_pitch once measured,
# e.g.  run_perception_real.sh cam_x:=0.12 cam_z:=0.22
set -eo pipefail

export ROS_DOMAIN_ID=0
# Discovery-Server mode: the image-baked FASTRTPS profile must stay blank
# (gotcha 7 in ~/robot/CLAUDE.md) — docker-compose already sets both envs;
# re-assert here so a manual `docker exec bash` shell works the same.
export FASTRTPS_DEFAULT_PROFILES_FILE=
PI5_IP=$(getent ahostsv4 rakhi24-desktop.local 2>/dev/null | awk 'NR==1{print $1}')
export ROS_DISCOVERY_SERVER="${PI5_IP:-192.168.2.10}:11811"
echo "discovery server: $ROS_DISCOVERY_SERVER"

# Ensure D555 DDS context config (survives container recreation via workspace volume).
cat > ~/.realsense-config.json <<'_RS_CFG'
{"context":{"dds":{"enabled":true,"domain":0}}}
_RS_CFG

source /opt/ros/jazzy/setup.bash
if [ -f /workspaces/isaac_ros-dev/install/setup.bash ]; then
    source /workspaces/isaac_ros-dev/install/setup.bash
else
    echo "workspace not built — run:" >&2
    echo "  cd /workspaces/isaac_ros-dev && colcon build --symlink-install --packages-select langrobo_perception" >&2
    exit 1
fi

# DDS-capable librealsense must be prepended AFTER setup.bash so it beats
# /opt/ros/jazzy/lib/aarch64-linux-gnu/librealsense2.so (apt, no DDS).
# Uses the cmake-install tree (has Targets.cmake) that realsense2_camera was
# compiled against; build/Release also works but install is canonical.
export LD_LIBRARY_PATH=/root/librealsense/install/lib:${LD_LIBRARY_PATH}

exec ros2 launch langrobo_perception perception.launch.py mode:=real use_sim_time:=False "$@"
