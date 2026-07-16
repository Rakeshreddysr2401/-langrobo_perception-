#!/bin/bash
# LangRobo Jetson perception stack — one-command bring-up with health gates.
# Run from the HOST (or via systemd langrobo-perception.service / Pi5 `robot start`).
#
#   camera → cuVSLAM → nvblox → Nav2 → vision AI (YOLO + grounding + target finder)
#
# Each stage must pass its health check before the next starts; a failed stage
# aborts with a named error so `robot status` tells you exactly what to fix.
# Total cold-start ~2.5 min. Camera not detected usually means the D555 needs
# a physical power cycle (its onboard DDS stack goes stale).
set -eo pipefail
DIR=$(cd "$(dirname "$0")" && pwd)

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

echo "[1/5] camera (D555)"
docker start isaac_ros >/dev/null 2>&1 || true
# Only (re)start the driver if the stream is down: restarting a healthy
# driver trips the D555's stale-DDS mode (physical power cycle to recover).
if ros_check 5 /camera/camera0/infra1/image_rect_raw; then
    echo "  [ok] camera (already streaming — driver left alone)"
else
    "$DIR/run_d555_stereo.sh" >/dev/null
    wait_for "camera" 60 /camera/camera0/infra1/image_rect_raw || {
        echo "  hint: check 'ip link show enP8p1s0' says mtu 9000; if so, power-cycle the D555 (unplug 5s) and rerun" >&2; exit 1; }
fi
# Emitter must be off for cuVSLAM (dot pattern corrupts tracking) and the
# launch arg is silently dropped — enforce at runtime EVERY start, including
# the driver-left-alone path (see run_d555_stereo.sh, 2026-07-16).
docker exec isaac_ros bash -c "
    unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
    source /opt/ros/jazzy/setup.bash
    ros2 param set /camera/camera0 depth_module.emitter_enabled false" \
    2>/dev/null | grep -q successful \
    && echo "  [ok] IR emitter off" \
    || echo "  [WARN] could not confirm emitter off — SLAM may corrupt on motion" >&2

echo "[2/5] cuVSLAM (localization)"
"$DIR/run_cuvslam_sidecar.sh" >/dev/null
wait_for "cuVSLAM odometry" 60 /visual_slam/tracking/odometry || exit 1

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
wait_for "look feed" 60 /camera/color/image_raw/compressed || exit 1

echo "ALL UP — run $DIR/status_all.sh for the full health table"
