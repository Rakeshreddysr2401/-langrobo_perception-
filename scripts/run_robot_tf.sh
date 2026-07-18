#!/bin/bash
# Robot-geometry TF: base_link -> camera0_link (camera mount, servos centred).
# Run from HOST. Runs in BOTH hardware modes (real | sim): this transform is
# a property of the ROBOT MODEL, not of the camera — the simulated rover must
# mount its camera at exactly these offsets (see SIM_REAL_PARITY.md §2).
#
# PLACEHOLDER values — measure from the wheel-axle midpoint and update here
# (forward = x, left = y, up = z, metres). Keep in sync with
# launch/perception.launch.py cam_x/cam_y/cam_z defaults and the rover_sim
# camera mount.
set -eo pipefail

docker exec isaac_ros bash -c 'pkill -f "[s]tatic_transform_publisher" 2>/dev/null; true'
docker exec -d isaac_ros bash -c '
    unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
    export ROS_DOMAIN_ID=0
    source /opt/ros/jazzy/setup.bash
    exec ros2 run tf2_ros static_transform_publisher \
        --x 0.10 --y 0.0 --z 0.25 \
        --frame-id base_link --child-frame-id camera0_link \
        > /tmp/static_tf.log 2>&1'

echo "robot TF up: base_link -> camera0_link (x=0.10 z=0.25 — placeholders, measure!)"
