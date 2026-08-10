#!/bin/bash
# rover_sim entry point — run ON THE LAPTOP (deployed to ~/langrobo/rover_sim
# by scripts/deploy_rover_sim.sh; repo copy is the source of truth).
#
#   gz sim (headless server, RTF 1.0)  — the world + rover
#   ros_gz parameter_bridge            — gz <-> ROS on internal names
#   contract_bridge.py                 — the D555/ESP32 impersonation layer
#
# Then, on the Jetson: echo sim > config/hardware && robot restart
# Verify: scripts/check_contract.sh (must be as green as real mode).
set -eo pipefail
DIR=$(cd "$(dirname "$0")" && pwd)

source /opt/ros/jazzy/setup.bash
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=0
export GZ_SIM_RESOURCE_PATH="$DIR/models"

"$DIR/stop_sim.sh" >/dev/null 2>&1 || true
sleep 1

echo "[1/4] gz sim (headless, $(basename "$DIR/worlds/langrobo_home.sdf"))"
# NOTE: gz stamps start at sim-time 0; the restamper (step 3) rewrites them
# to the wall clock. Do NOT use --initial-sim-time with an epoch value —
# it silently breaks all sensor/stats scheduling (found live 2026-07-17).
nohup setsid gz sim -s -r --headless-rendering \
    "$DIR/worlds/langrobo_home.sdf" > /tmp/gz_sim.log 2>&1 < /dev/null &
for i in $(seq 1 30); do
    gz topic -l 2>/dev/null | grep -q "/rover_sim/infra1/image" && break
    [ "$i" = 30 ] && { echo "  [FAIL] gz sensors never came up — /tmp/gz_sim.log" >&2; exit 1; }
    sleep 2
done
echo "  [ok] world + sensors up"

echo "[2/4] ros_gz parameter_bridge (gz <-> ROS, internal /rover_sim names)"
nohup ros2 run ros_gz_bridge parameter_bridge \
    /rover_sim/infra1/image@sensor_msgs/msg/Image[gz.msgs.Image \
    /rover_sim/infra1/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
    /rover_sim/depth/image@sensor_msgs/msg/Image[gz.msgs.Image \
    /rover_sim/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
    /rover_sim/color/image@sensor_msgs/msg/Image[gz.msgs.Image \
    /rover_sim/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
    /rover_sim/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
    /rover_sim/drive_cmd@geometry_msgs/msg/Twist]gz.msgs.Twist \
    > /tmp/gz_bridge.log 2>&1 &
sleep 3
echo "  [ok] bridge up"

echo "[3/4] restamper (C++: contract names, wall stamps, depth 16UC1)"
if [ ! -x "$DIR/restamper/build/restamper" ] \
        || [ "$DIR/restamper/restamper.cpp" -nt "$DIR/restamper/build/restamper" ]; then
    echo "  building..."
    cmake -S "$DIR/restamper" -B "$DIR/restamper/build" \
        -DCMAKE_BUILD_TYPE=Release > /tmp/restamper_build.log 2>&1 \
        && cmake --build "$DIR/restamper/build" -j2 >> /tmp/restamper_build.log 2>&1 \
        || { echo "  [FAIL] restamper build — /tmp/restamper_build.log" >&2; exit 1; }
fi
nohup "$DIR/restamper/build/restamper" > /tmp/restamper.log 2>&1 &
echo "  [ok] restamper up"

echo "[4/4] contract bridge (motion: imu + drive emulation + tf + marker)"
nohup python3 "$DIR/bridge/contract_bridge.py" \
    --contract "$DIR/d555_contract" --role motion \
    > /tmp/contract_bridge_motion.log 2>&1 &
sleep 3
timeout 10 ros2 topic echo --once /rover_sim/status >/dev/null 2>&1 \
    && echo "  [ok] contract topics live" \
    || { echo "  [FAIL] contract bridge — /tmp/contract_bridge_*.log" >&2; exit 1; }

echo "rover_sim UP — contract topics publishing on domain 0"
echo "  watch RTF:  gz topic -e -t /stats -n 1 | grep real_time_factor"
echo "  logs:       /tmp/gz_sim.log /tmp/gz_bridge.log /tmp/contract_bridge_{rgbd,color,motion}.log"
