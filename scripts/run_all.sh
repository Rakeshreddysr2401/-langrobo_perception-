#!/bin/bash
# LangRobo Jetson perception stack — one-command bring-up with health gates.
# Run from the HOST (or via systemd langrobo-perception.service / Pi5 `robot start`).
#
#   camera → localization → nvblox → Nav2 → vision AI (YOLO + grounding + target finder)
#
# Each stage must pass its health check before the next starts; a failed stage
# aborts with a named error so `robot status` tells you exactly what to fix.
# Total cold-start ~2.5 min. Camera not detected usually means the D555 needs
# a physical power cycle (its onboard DDS stack goes stale).
#
# Hardware mode (config/hardware: real | sim — see SIM_REAL_PARITY.md):
#   real  start the D555 driver; the ESP32 rover consumes /cmd_vel
#   sim   stage 1 instead WAITS for rover_sim (laptop) to publish the exact
#         same camera topic contract; everything from stage 2 on is
#         byte-identical — the stack cannot tell which mode it is in.
set -eo pipefail
DIR=$(cd "$(dirname "$0")" && pwd)
HW=$(tr -d '[:space:]' < "$DIR/../config/hardware" 2>/dev/null || echo real)

ros_check() {  # ros_check <timeout_s> <topic> — true if the topic is flowing
    docker exec isaac_ros bash -c "
        unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
        source /opt/ros/jazzy/setup.bash
        timeout $1 ros2 topic echo '$2' --once >/dev/null 2>&1"
}

wait_for() {  # wait_for <name> <deadline_s> <topic>
    local start=$SECONDS
    while (( SECONDS - start < $2 )); do
        if ros_check 5 "$3"; then echo "  [ok] $1"; return 0; fi
    done
    echo "  [FAIL] $1 — no data on $3 after $2s" >&2
    return 1
}

echo "[0/5] robot TF (base_link -> camera0_link, both modes)"
docker start isaac_ros >/dev/null 2>&1 || true
"$DIR/run_robot_tf.sh" >/dev/null && echo "  [ok] robot TF"

echo "[1/5] camera ($HW)"
if [ "$HW" = "sim" ]; then
    # INTERLOCK: with the simulator on /cmd_vel, the REAL rover must not be
    # listening — a live ESP32 would replay the sim mission on the floor.
    if docker exec isaac_ros bash -c "
            unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
            source /opt/ros/jazzy/setup.bash
            timeout 10 ros2 node list 2>/dev/null" | grep -q rover_esp32; then
        echo "  [FAIL] real rover (rover_esp32) is on the network — power it off (or stop langrobo-microros on the Pi5) before sim mode" >&2
        exit 1
    fi
    echo "  [ok] interlock: no real rover on the network"
    wait_for "sim camera (rover_sim on the laptop)" 120 /camera/camera0/infra1/image_rect_raw || {
        echo "  hint: start rover_sim on the laptop — it must publish the D555 topic contract (SIM_REAL_PARITY.md §2)" >&2; exit 1; }
else
    # INTERLOCK mirror: refuse to treat a running simulator as the camera.
    if ros_check 3 /rover_sim/status; then
        echo "  [FAIL] rover_sim (laptop simulator) is publishing — stop it before real mode" >&2
        exit 1
    fi
    # Only (re)start the driver if the stream is down: restarting a healthy
    # driver trips the D555's stale-DDS mode (physical power cycle to recover).
    if ros_check 5 /camera/camera0/infra1/image_rect_raw; then
        echo "  [ok] camera (already streaming — driver left alone)"
    else
        "$DIR/run_d555_stereo.sh" >/dev/null
        wait_for "camera" 60 /camera/camera0/infra1/image_rect_raw || {
            echo "  hint: check 'ip link show enP8p1s0' says mtu 9000; if so, power-cycle the D555 (unplug 5s) and rerun" >&2; exit 1; }
    fi
    # Emitter must be off for SLAM (dot pattern corrupts tracking) and the
    # launch arg is silently dropped — enforce at runtime EVERY start,
    # including the driver-left-alone path (see run_d555_stereo.sh).
    docker exec isaac_ros bash -c "
        unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
        source /opt/ros/jazzy/setup.bash
        ros2 param set /camera/camera0 depth_module.emitter_enabled false >/dev/null &&
        ros2 param set /camera/camera0 depth_module.global_time_enabled false" \
        2>/dev/null | grep -q successful \
        && echo "  [ok] IR emitter off + per-stream global time off" \
        || echo "  [WARN] could not set camera params — SLAM may corrupt on motion" >&2
fi

echo "[2/5] localization ($(tr -d '[:space:]' < "$DIR/../config/localization" 2>/dev/null || echo rtabmap))"
"$DIR/run_localization.sh" >/dev/null
wait_for "odometry" 60 "$("$DIR/localization_odom_topic.sh")" || exit 1

echo "[3/5] nvblox (3D map + costmaps)"
"$DIR/run_nvblox.sh" >/dev/null
wait_for "nvblox map slice" 60 /nvblox_node/static_map_slice || exit 1

echo "[4/5] Nav2 (navigation)"
"$DIR/run_nav2.sh" >/dev/null
deadline=$((SECONDS + 90))
until docker exec isaac_ros grep -aq "Managed nodes are active" /tmp/nav2.log 2>/dev/null; do
    (( SECONDS < deadline )) || { echo "  [FAIL] Nav2 lifecycle" >&2; exit 1; }
    sleep 3
done
echo "  [ok] Nav2 active"

echo "[5/5] vision AI (YOLO + pixel grounding + target finder)"
"$DIR/run_vision_ai.sh" >/dev/null
# sim: first frames arrive only after the laptop pipeline warms up — allow 180s
wait_for "look feed" "$([ "$HW" = sim ] && echo 180 || echo 60)" /camera/color/image_raw/compressed || exit 1

echo "ALL UP — run $DIR/status_all.sh for the full health table"
