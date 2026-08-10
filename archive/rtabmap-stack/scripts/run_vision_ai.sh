#!/bin/bash
# Launch the Pi5-facing vision AI layer. Run from HOST after the perception
# stack (camera + cuVSLAM at minimum) is up.
#
#   detections_3d — YOLO objects → /vision/detections_3d (map-frame JSON)
#                   + /camera/color/image_raw/compressed (2Hz JPEG, VLM look())
#   pixel_to_goal — VLM pixel (u,v) on /vision/pixel_query
#                   → JSON result on /vision/pixel_result (always answered)
#                   + Nav2-ready PoseStamped on /vision/pixel_goal (success only)
#
# Pi5 (192.168.1.16, WiFi) reaches these over the Jetson's wlP1p1s0
# (192.168.1.15) via default multicast discovery — no discovery server.
# Pi5-side consumer: pi5/langrobo_client.py (deploy with deploy_pi5.sh).
set -eo pipefail

docker exec isaac_ros bash -c 'pkill -f "[d]etections_3d" 2>/dev/null; pkill -f "[p]ixel_to_goal" 2>/dev/null; true'
sleep 2

docker exec -d isaac_ros bash -c '
    unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
    export ROS_DOMAIN_ID=0
    source /opt/ros/jazzy/setup.bash
    source /workspaces/isaac_ros-dev/install/setup.bash
    exec ros2 run langrobo_perception detections_3d > /tmp/detections_3d.log 2>&1'

docker exec -d isaac_ros bash -c '
    unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
    export ROS_DOMAIN_ID=0
    source /opt/ros/jazzy/setup.bash
    source /workspaces/isaac_ros-dev/install/setup.bash
    exec ros2 run langrobo_perception pixel_to_goal > /tmp/pixel_to_goal.log 2>&1'

echo "vision AI starting — logs: /tmp/detections_3d.log /tmp/pixel_to_goal.log (in isaac_ros)"
