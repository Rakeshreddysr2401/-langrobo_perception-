#!/bin/bash
# Launch nvblox (GPU 3D reconstruction + costmaps) fed by cuVSLAM pose.
# Run from HOST. Prerequisites: run_d555_stereo.sh + run_cuvslam_sidecar.sh.
set -eo pipefail

docker exec isaac_ros bash -c 'pkill -f "[n]vblox_node"; true'
sleep 3

docker exec -d isaac_ros bash -c '
    unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
    export ROS_DOMAIN_ID=0
    source /opt/ros/jazzy/setup.bash
    source /workspaces/isaac_ros-dev/install/setup.bash
    NVBLOX_BASE=$(python3 -c "from ament_index_python.packages import get_package_share_directory as g; print(g(\"nvblox_examples_bringup\")+\"/config/nvblox/nvblox_base.yaml\")")
    exec ros2 run nvblox_ros nvblox_node --ros-args \
        --params-file $NVBLOX_BASE \
        --params-file /workspaces/isaac_ros-dev/src/langrobo_perception/config/nvblox_real.yaml \
        -p use_color:=true \
        -r camera_0/depth/image:=/camera/camera0/depth/image_rect_raw \
        -r camera_0/depth/camera_info:=/camera/camera0/depth/camera_info \
        -r camera_0/color/image:=/camera/camera0/color/image_raw \
        -r camera_0/color/camera_info:=/camera/camera0/color/camera_info \
        > /tmp/nvblox.log 2>&1'

echo "nvblox starting — log: docker exec isaac_ros tail -f /tmp/nvblox.log"
