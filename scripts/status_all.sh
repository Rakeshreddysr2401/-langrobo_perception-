#!/bin/bash
# LangRobo health table — every link from camera to wheels, as seen from the
# Jetson. Run any time; used by `robot status` on the Pi5.
DIR=$(cd "$(dirname "$0")" && pwd)
PI5=${PI5:-192.168.1.16}

check() {  # check <label> <timeout_s> <topic>
    if docker exec isaac_ros bash -c "
        unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
        source /opt/ros/jazzy/setup.bash
        timeout $2 ros2 topic echo '$3' --once >/dev/null 2>&1"; then
        printf "  %-28s OK\n" "$1"
    else
        printf "  %-28s DOWN  (%s)\n" "$1" "$3"
    fi
}

echo "── Jetson perception ──────────────────────────"
check "camera stereo IR"      5 /camera/camera0/infra1/image_rect_raw
check "camera depth"          5 /camera/camera0/depth/image_rect_raw
check "cuVSLAM odometry"      5 /visual_slam/tracking/odometry
check "nvblox costmap slice"  5 /nvblox_node/static_map_slice
check "look feed (VLM eye)"   5 /camera/color/image_raw/compressed

if docker exec isaac_ros grep -aq "Managed nodes are active" /tmp/nav2.log 2>/dev/null; then
    printf "  %-28s OK\n" "Nav2 lifecycle"
else
    printf "  %-28s DOWN\n" "Nav2 lifecycle"
fi

echo "── Rover (ESP32 via Pi5 micro-ROS) ────────────"
SUBS=$(docker exec isaac_ros bash -c '
    unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
    source /opt/ros/jazzy/setup.bash
    timeout 6 ros2 topic info /cmd_vel 2>/dev/null' | awk -F": " "/Subscription/{print \$2}")
if [ "${SUBS:-0}" -ge 1 ] 2>/dev/null; then
    printf "  %-28s OK    (%s subscriber)\n" "/cmd_vel -> wheels" "$SUBS"
else
    printf "  %-28s DOWN  (power-cycle the rover)\n" "/cmd_vel -> wheels"
fi

echo "── Pi5 brain ──────────────────────────────────"
if ping -c1 -W2 "$PI5" >/dev/null 2>&1; then
    ssh -o BatchMode=yes -o ConnectTimeout=5 "$PI5" '
        for u in langrobo-brain langrobo-microros; do
            printf "  %-28s %s\n" "$u" "$(systemctl is-active $u)"
        done' 2>/dev/null || printf "  %-28s unreachable (ssh)\n" "pi5 services"
else
    printf "  %-28s DOWN  (no ping %s)\n" "Pi5" "$PI5"
fi

echo "── Mac mini LLM ───────────────────────────────"
if curl -sf -m 4 http://192.168.1.7:8080/health >/dev/null 2>&1; then
    printf "  %-28s OK\n" "llama.cpp (VLM)"
else
    printf "  %-28s DOWN  (http://192.168.1.7:8080)\n" "llama.cpp (VLM)"
fi
