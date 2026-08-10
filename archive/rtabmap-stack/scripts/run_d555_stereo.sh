#!/bin/bash
# Start the D555 inside the jazzy isaac_ros container. Run from the Jetson HOST.
#
# infra1 + depth + color @ 896x504, emitter OFF so the IR dot pattern does not
# corrupt feature tracking. Depth feeds nvblox (noisier with emitter off —
# revisit with emitter_on_off toggling if quality hurts).
#
# infra2 (the second stereo-IR channel) is DISABLED: it exists only for
# cuVSLAM stereo, which is PARKED (SIGILL on Orin Nano — see
# CUVSLAM_ORIN_GUIDE.md). RTAB-Map localizes on color+depth (RGBD), nvblox
# uses depth, and the parity contract exposes infra1 only, so infra2 had ZERO
# subscribers while streaming a full 30Hz 896x504 mono channel — measured
# 2026-07-18 as a real drag on the camera driver's CPU, which was starving the
# depth stream below its 10Hz floor under load. Re-enable it (and the stereo
# infra_profile) the day cuVSLAM is re-adopted.
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
        enable_infra1:=true enable_infra2:=false \
        depth_module.infra_profile:=896x504x30 \
        depth_module.emitter_enabled:=0 \
        enable_color:=true enable_depth:=true \
        enable_motion:=true \
        enable_sync:=true \
        > /tmp/rs_infra.log 2>&1'

# Robot geometry (base_link -> camera0_link) moved to run_robot_tf.sh: it
# belongs to the robot model, not the camera, and must also run in sim mode.
# Kept here for standalone use — idempotent.
"$(cd "$(dirname "$0")" && pwd)/run_robot_tf.sh"

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
