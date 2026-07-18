#!/bin/bash
# Sim/real parity contract checker — run from the HOST, in either hardware
# mode (config/hardware: real | sim). Every check here is a property the
# perception/nav/vision stack depends on; if sim passes the exact same table
# as real, the stack CANNOT tell which mode it is in (SIM_REAL_PARITY.md).
#
# This is the promotion gate: enable the real rover only after this is fully
# green in sim AND the scripted missions pass there (§5 of the parity doc).
#
# Takes ~2 min (rate windows). Exit code 0 = contract satisfied.
set -o pipefail
DIR=$(cd "$(dirname "$0")" && pwd)
HW=$(tr -d '[:space:]' < "$DIR/../config/hardware" 2>/dev/null || echo real)

R() {  # run a ros2 command inside the container
    docker exec isaac_ros bash -c "
        unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
        source /opt/ros/jazzy/setup.bash
        source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
        $*"
}

pass=0; fail=0
ok()  { echo "  [ok]   $1"; pass=$((pass + 1)); }
bad() { echo "  [FAIL] $1"; fail=$((fail + 1)); }

check_type() {  # <topic> <expected type>
    local t
    t=$(R "timeout 8 ros2 topic type '$1' 2>/dev/null" | head -1)
    [ "$t" = "$2" ] && ok "$1 : $2" || bad "$1 : got '${t:-nothing}', want $2"
}

check_rate() {  # <topic> <min hz>
    local hz
    hz=$(R "timeout 15 ros2 topic hz -w 15 '$1' 2>/dev/null" \
         | grep -m1 -o 'average rate: [0-9.]*' | awk '{print $3}')
    if [ -n "$hz" ] && awk -v a="$hz" -v b="$2" 'BEGIN{exit !(a>=b)}'; then
        ok "$1 @ ${hz}Hz (>= $2)"
    else
        bad "$1 rate: got '${hz:-none}', want >= $2 Hz"
    fi
}

check_field() {  # <topic> <field> <expected value>
    local v  # ros2cli interleaves QoS chatter on stdout ("A message was
    # lost!!!", "Some, but not all, publishers are offering ..."): drop every
    # known warning line and the --- separator, then take the first value line
    v=$(R "timeout 8 ros2 topic echo --once '$1' --field '$2' 2>/dev/null" \
        | grep -v -i -E 'message was lost|publishers are offering|falling back to' \
        | grep -v '^---' | grep -m1 -E '^[^[:space:]]')
    [ "$v" = "$3" ] && ok "$1 $2 = $3" || bad "$1 $2: got '${v:-nothing}', want '$3'"
}

# tf2_echo loses its output when timeout kills it (block-buffered pipe), so
# resolve all pairs in one rclpy listener instead: 6s of buffer, then lookup.
check_tf_pairs() {  # <parent:child>...
    local out pair
    out=$(R "python3 - $* <<'PYEOF'
import sys, time
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
rclpy.init()
n = Node('contract_tf_probe')
buf = Buffer()
TransformListener(buf, n)
end = time.monotonic() + 6.0
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
for pair in sys.argv[1:]:
    parent, child = pair.split(':')
    ok = buf.can_transform(parent, child, rclpy.time.Time())
    print(f'{pair} {\"OK\" if ok else \"MISSING\"}', flush=True)
PYEOF
")
    for pair in "$@"; do
        if grep -q "^$pair OK$" <<< "$out"; then
            ok "TF ${pair/:/ -> }"
        else
            bad "TF ${pair/:/ -> } unresolved"
        fi
    done
}

echo "== LangRobo parity contract check (mode: $HW) =="

echo "-- mode interlocks"
if [ "$HW" = "sim" ]; then
    R "timeout 8 ros2 topic echo --once /rover_sim/status >/dev/null 2>&1" \
        && ok "rover_sim marker present" || bad "sim mode but /rover_sim/status silent"
    R "timeout 10 ros2 node list 2>/dev/null" | grep -q rover_esp32 \
        && bad "REAL rover on the network in sim mode (rover_esp32)" \
        || ok "no real rover on the network"
else
    R "timeout 3 ros2 topic echo --once /rover_sim/status >/dev/null 2>&1" \
        && bad "simulator marker present in real mode" || ok "no simulator on the network"
fi

echo "-- camera source (the ONLY thing that may differ between modes)"
check_type /camera/camera0/infra1/image_rect_raw  sensor_msgs/msg/Image
check_type /camera/camera0/infra1/camera_info     sensor_msgs/msg/CameraInfo
check_type /camera/camera0/depth/image_rect_raw   sensor_msgs/msg/Image
check_type /camera/camera0/color/image_raw        sensor_msgs/msg/Image
check_type /camera/camera0/motion/sample          sensor_msgs/msg/Imu
check_field /camera/camera0/infra1/camera_info width  896
check_field /camera/camera0/infra1/camera_info height 504
check_field /camera/camera0/infra1/camera_info header.frame_id camera0_infra1_optical_frame
check_field /camera/camera0/depth/camera_info  header.frame_id camera0_depth_optical_frame
check_field /camera/camera0/color/camera_info  header.frame_id camera0_color_optical_frame
check_field /camera/camera0/depth/image_rect_raw encoding 16UC1
# Nominal 30Hz; effective delivery on the loaded Orin swings 13-32Hz
# (measured across runs 2026-07-17). Floors = what the consumers need
# (nvblox integrates ~10Hz, RTAB-Map syncs at odom rate); sim must meet
# the SAME floors, no more required.
check_rate /camera/camera0/infra1/image_rect_raw 10
check_rate /camera/camera0/depth/image_rect_raw  10
check_rate /camera/camera0/color/image_raw       5

echo "-- clock discipline (stamps must track the Jetson wall clock)"
# take only a purely-numeric line so QoS chatter can't poison the arithmetic
stamp=$(R "timeout 8 ros2 topic echo --once /camera/camera0/depth/camera_info --field header.stamp.sec 2>/dev/null" | grep -m1 -E '^[0-9]+$')
now=$(date +%s)
if [ -n "$stamp" ] && [ $((now - stamp)) -le 2 ] && [ $((stamp - now)) -le 2 ]; then
    ok "camera stamps within 2s of wall clock (delta $((now - stamp))s)"
else
    bad "camera stamps off wall clock (stamp=${stamp:-none} now=$now) — sim must re-stamp at RTF 1.0 + chrony"
fi

echo "-- TF: camera frames + robot model + localization"
check_tf_pairs \
    camera0_link:camera0_infra1_optical_frame \
    camera0_link:camera0_depth_optical_frame \
    camera0_link:camera0_color_optical_frame \
    base_link:camera0_link \
    map:base_link

echo "-- localization (identical in both modes)"
# RTAB-Map rgbd_odometry swings 2-5Hz on the loaded Orin (measured
# 2026-07-17). Floor 1.5 = "alive"; raising this is a Phase 2 goal (IMU
# fusion), and sim must then meet the same raised floor.
check_rate /odom 1.5

echo "-- mapping / navigation / vision (identical in both modes)"
check_rate /nvblox_node/static_map_slice 1
R "timeout 10 ros2 action list 2>/dev/null" | grep -q "^/navigate_to_pose$" \
    && ok "Nav2 action server" || bad "Nav2 action server missing"
check_rate /camera/color/image_raw/compressed 1
check_rate /vision/detections_3d 0.2
R "timeout 8 ros2 topic list 2>/dev/null" | grep -q "^/vision/pixel_result$" \
    && ok "pixel grounding endpoint" || bad "/vision/pixel_result missing"

echo "-- wheels (/cmd_vel consumer)"
subs=$(R "timeout 8 ros2 topic info /cmd_vel 2>/dev/null" \
       | grep -m1 -o 'Subscription count: [0-9]*' | awk '{print $3}')
if [ "${subs:-0}" -ge 1 ]; then
    ok "/cmd_vel has ${subs} consumer(s) ($([ "$HW" = sim ] && echo simulated rover || echo ESP32 via micro-ROS))"
else
    bad "/cmd_vel has no consumer — $([ "$HW" = sim ] && echo start rover_sim || echo power the rover / check langrobo-microros on the Pi5)"
fi

echo "== $pass ok, $fail failed (mode: $HW) =="
exit $((fail > 0))
