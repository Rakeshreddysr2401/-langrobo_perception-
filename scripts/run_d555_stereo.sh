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

# CRITICAL: the emitter_enabled launch arg above is silently DROPPED — the DDS
# driver only declares depth_module.* params after the device connects (launch
# log even warns "not supported"). Found live 2026-07-16: emitter was ON at
# laser_power 150 and its camera-fixed dot pattern corrupted cuVSLAM the moment
# the camera moved (stationary looked perfect; any motion exploded to ±100m).
# Set it at runtime once the node is up, with retries.
# global_time_enabled must ALSO be off: its host-clock conversion runs an
# independent drift model per stream, so infra1/infra2 stamps tick at
# slightly different rates (33.2535 vs 33.3745 ms/frame measured) and
# cuVSLAM's timestamp pairing matches frames from different instants —
# static looks fine, any motion explodes. Raw device stamps are identical
# for the synced stereo shutter (and the D555 clock tracks host within ms).
for i in $(seq 1 24); do
    if docker exec isaac_ros bash -c '
        unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
        source /opt/ros/jazzy/setup.bash
        ros2 param set /camera/camera0 depth_module.emitter_enabled false >/dev/null &&
        ros2 param set /camera/camera0 depth_module.global_time_enabled false' \
        2>/dev/null | grep -q "successful"; then
        echo "emitter OFF + global_time OFF (runtime param set)"
        break
    fi
    [ "$i" = 24 ] && echo "WARNING: could not set camera params — cuVSLAM WILL corrupt on motion" >&2
    sleep 5
done

echo "D555 stereo IR starting — log: docker exec isaac_ros tail -f /tmp/rs_infra.log"
