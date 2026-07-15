#!/bin/bash
# Start the D555 in stereo-IR mode for cuVSLAM inside the jazzy isaac_ros
# container. Run from the Jetson HOST.
#
# infra1/infra2 @ 896x504x30 (D555 native), emitter OFF so the IR dot pattern
# does not corrupt cuVSLAM feature tracking. Depth enabled for nvblox (noisier
# with emitter off — revisit with emitter_on_off toggling if quality hurts).
# Color still off: Orin bandwidth/CPU budget.
set -eo pipefail

docker exec isaac_ros bash -c 'pkill -f "[r]ealsense2_camera_node" 2>/dev/null; true'
sleep 2

docker exec -d isaac_ros bash -c '
    unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
    export ROS_DOMAIN_ID=0
    cat > ~/.realsense-config.json <<EOF
{"context":{"dds":{"enabled":true,"domain":0}}}
EOF
    source /opt/ros/jazzy/setup.bash
    source /workspaces/isaac_ros-dev/install/setup.bash
    export LD_LIBRARY_PATH=/root/librealsense/install/lib:$LD_LIBRARY_PATH
    exec ros2 launch realsense2_camera rs_launch.py \
        camera_name:=camera0 \
        enable_infra1:=true enable_infra2:=true \
        depth_module.infra_profile:=896x504x30 \
        depth_module.emitter_enabled:=0 \
        enable_color:=true enable_depth:=true \
        pointcloud.enable:=true \
        enable_motion:=true \
        enable_sync:=true \
        > /tmp/rs_infra.log 2>&1'

# Robot geometry: camera mount relative to base_link (servos centred), same
# values as perception.launch.py cam_x/cam_y/cam_z defaults.
docker exec isaac_ros bash -c 'pkill -f "[s]tatic_transform_publisher" 2>/dev/null; true'
docker exec -d isaac_ros bash -c '
    unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
    export ROS_DOMAIN_ID=0
    source /opt/ros/jazzy/setup.bash
    exec ros2 run tf2_ros static_transform_publisher \
        --x 0.10 --y 0.0 --z 0.25 \
        --frame-id base_link --child-frame-id camera0_link \
        > /tmp/static_tf.log 2>&1'

echo "D555 stereo IR starting — log: docker exec isaac_ros tail -f /tmp/rs_infra.log"
