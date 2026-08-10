#!/bin/bash
# Robot-geometry TF: base_link -> camera0_link (camera mount, servos centred).
# Run from HOST. Runs in BOTH hardware modes (real | sim): this transform is
# a property of the ROBOT MODEL, not of the camera — the simulated rover must
# mount its camera at exactly these offsets (see SIM_REAL_PARITY.md §2).
#
# Values are from the OWNER's setup (2026-07-18): a market 4-wheel chassis,
# camera ~15 cm off the ground -> z=0.15. z is the one that matters most (it
# sets the height nvblox slices the costmap at). x (forward offset from the
# wheel-axle midpoint) is still a rough 0.10 estimate; y assumed centred.
# Refine x/y with a tape measure when convenient. Keep in sync with the
# rover_sim camera mount (SIM_REAL_PARITY.md §2).
#
# NOTE on the chassis: a cheap 4-wheel chassis is almost always SKID-STEER,
# which is kinematically the same as differential drive (it turns by skidding
# left vs right wheels) — the diff-drive Nav2 + ESP32 firmware handle it as-is.
# No change needed unless it turns out to be car-like (Ackermann) steering.
set -eo pipefail

docker exec isaac_ros bash -c 'pkill -f "[s]tatic_transform_publisher" 2>/dev/null; true'
docker exec -d isaac_ros bash -c '
    unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
    export ROS_DOMAIN_ID=0
    source /opt/ros/jazzy/setup.bash
    exec ros2 run tf2_ros static_transform_publisher \
        --x 0.10 --y 0.0 --z 0.15 \
        --frame-id base_link --child-frame-id camera0_link \
        > /tmp/static_tf.log 2>&1'

echo "robot TF up: base_link -> camera0_link (x=0.10 y=0.0 z=0.15 — camera ~15cm off ground; x rough)"
