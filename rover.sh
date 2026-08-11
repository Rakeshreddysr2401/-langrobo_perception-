#!/bin/bash
# rover.sh — ONE command, but the stack comes up in LAYERS you can prove.
#
# The old stack started camera + TF + cuVSLAM + nvblox all at once. That is why
# it was impossible to debug: when something broke, any of four layers could
# have done it. On 2026-08-11 it ran for an hour with TWO nvblox nodes competing
# and nobody noticed, because "it looks like it's running".
#
# Here each layer starts ONLY what it adds, and REFUSES to start if the layer
# below it is not healthy. You cannot accidentally build on a broken foundation.
#
#   ./rover.sh l1          camera alone
#   ./rover.sh l2          + cuVSLAM          (needs l1 healthy)
#   ./rover.sh l3          + gyro + EKF       (needs l2 healthy)
#   ./rover.sh l4          + nvblox mapping   (needs l3 healthy)
#   ./rover.sh l5          + nav2             (needs l4 healthy)  MOVES THE ROBOT
#
#   ./rover.sh status      health of every layer — measures RATES, not existence
#   ./rover.sh measure [m] tape-measure the pose (push it by hand)
#   ./rover.sh view [start] RViz on the laptop
#   ./rover.sh logs <name> tail a component log
#   ./rover.sh stop        e-stop: kill motion, zero /cmd_vel
#   ./rover.sh down        remove the container (ultimate e-stop)
#
# READ FACTS.md BEFORE CHANGING ANYTHING HERE. Most of the odd-looking flags in
# this file are load-bearing and each one cost a session to find.
set -eo pipefail

IMAGE=${IMAGE:-orin-nav:1.1}     # reusing the built image; deps are all in it
NAME=${NAME:-rover}
SRC=/opt/rover                   # where this repo is mounted inside the container
HOSTSRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/src"
CAM_NS=/camera/camera0
LAPTOP_USER=${LAPTOP_USER:-rakhi24}
LAPTOP_IP=${LAPTOP_IP:-192.168.1.10}

# cuVSLAM's own libcuvslam.so must beat Isaac ROS's Thor build in /opt/ros/jazzy,
# so its wheel directory goes FIRST, then the CUDA-12 user-space libs.
CU12='/usr/local/lib/python3.12/dist-packages/cuvslam:/usr/local/lib/python3.12/dist-packages/nvidia/cuda_runtime/lib:/usr/local/lib/python3.12/dist-packages/nvidia/cublas/lib:/usr/local/lib/python3.12/dist-packages/nvidia/cusolver/lib:/usr/local/lib/python3.12/dist-packages/nvidia/cusparse/lib:/usr/local/lib/python3.12/dist-packages/nvidia/nvjitlink/lib:/usr/local/lib/python3.12/dist-packages/nvidia/nvtx/lib'

# ── plumbing ────────────────────────────────────────────────────────────────
# Every in-container command clears the stale discovery vars first: a leftover
# ROS_DISCOVERY_SERVER makes nodes silently invisible to each other.
ENVSET='unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE; export ROS_DOMAIN_ID=0; source /opt/ros/jazzy/setup.bash;'
dexec(){ docker exec -d "$NAME" bash -lc "$ENVSET $1"; }          # background
rexec(){ docker exec    "$NAME" bash -lc "$ENVSET $1"; }          # foreground

die(){ echo "  ✗ $*" >&2; exit 1; }

running(){ [ "$(docker inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null)" = "true" ]; }

need_container(){
  running || die "container '$NAME' is not running — start with: ./rover.sh l1"
}

# Refuse to run alongside the old stack. Both would grab the same camera and the
# same ROS_DOMAIN_ID, and you would spend an evening debugging a phantom.
check_no_old_stack(){
  if [ "$(docker inspect -f '{{.State.Running}}' orin_nav 2>/dev/null)" = "true" ]; then
    echo "  ✗ the OLD stack (container 'orin_nav') is still running."
    echo "    Both stacks use the same camera and ROS_DOMAIN_ID=0 and would fight."
    echo "    Stop it first:   docker rm -f orin_nav"
    exit 1
  fi
}

# Measure a topic's publish rate. This is the ONLY honest health signal on this
# rig: every failure has been a rate collapsing, not a topic disappearing
# (FACTS §3 — cuVSLAM freezes while still reporting slam_pose_ok: true).
rate_of(){
  rexec "timeout $2 ros2 topic hz $1 --window 20 2>/dev/null | awk '/average rate/{r=\$3} END{print r+0}'" 2>/dev/null || echo 0
}

# assert_rate <topic> <min_hz> <what> — the gate between layers.
assert_rate(){
  local topic="$1" min="$2" what="$3"
  local r; r=$(rate_of "$topic" 6)
  if awk "BEGIN{exit !($r >= $min)}"; then
    printf "  ok  %-22s %6.1f Hz  (want >= %s)\n" "$what" "$r" "$min"
  else
    printf "  ✗   %-22s %6.1f Hz  (want >= %s)\n" "$what" "$r" "$min"
    echo
    echo "    The layer below is not healthy, so starting this one would only"
    echo "    hide the problem. Fix that first:  ./rover.sh status"
    exit 1
  fi
}

# ── camera ──────────────────────────────────────────────────────────────────
# enable_sync:=false  — sync ON gates infra1/2 behind colour alignment over DDS
#                       and starves IR to <1 Hz. NOT bandwidth. FACTS §1.
# emitter_enabled:=0  — emitter ON makes cuVSLAM under-read translation ~4x on
#                       low-texture floors. Depth stays metric either way. FACTS §1.
launch_cam(){
  dexec 'printf "{\"context\":{\"dds\":{\"enabled\":true,\"domain\":0}}}" > ~/.realsense-config.json;
         export LD_LIBRARY_PATH=/root/librealsense/install/lib:$LD_LIBRARY_PATH;
         exec ros2 launch realsense2_camera rs_launch.py camera_name:=camera0 \
            enable_infra1:=true enable_infra2:=true depth_module.infra_profile:=896x504x30 \
            depth_module.emitter_enabled:=0 enable_depth:=true enable_color:=true \
            enable_motion:=true enable_sync:=false \
            > /tmp/realsense.log 2>&1'
}

# The camera answers ping while completely dead, so the only real check is
# whether infra1 has a live publisher.
cam_ready(){
  rexec "n=\$(ros2 topic info $CAM_NS/infra1/image_rect_raw 2>/dev/null | awk '/Publisher count/{print \$3}'); [ \"\${n:-0}\" -ge 1 ]" >/dev/null 2>&1
}

cam_wait(){
  local t=0
  while [ "$t" -lt 40 ]; do
    cam_ready && { echo "      D555 streaming — OK"; return 0; }
    t=$((t+3)); sleep 3
  done
  echo
  echo "  ✗ D555 is NOT streaming after 40s."
  if docker exec "$NAME" grep -q "No RealSense devices were found" /tmp/realsense.log 2>/dev/null; then
    echo "    Its on-camera DDS server is offline. It still answers ping — that means nothing."
    echo "    FIX: PHYSICALLY power-cycle it — unplug the PoE cable ~5 s, replug,"
    echo "         wait ~15 s, then:  ./rover.sh l1"
  else
    echo "    Check:  ./rover.sh logs realsense"
  fi
  return 1
}

# ── nvblox teardown ─────────────────────────────────────────────────────────
# BUG FOUND 2026-08-11: `up` then `fuse` back-to-back left TWO nvblox_node
# processes alive, each building its own map and both publishing to the same
# topic. RViz drew whichever frame landed last, the GPU did double the work, and
# the D555 got two depth subscribers — which is very likely why it kept starving.
#
# Cause: nvblox starts as `ros2 run nvblox_ros nvblox_node`, and that wrapper
# takes a second or two to exec the real binary. `pgrep -x nvblox_node` matches
# only the final process, so a kill issued in that window hits nothing.
#
# So: kill the wrapper by command line AS WELL as the exec'd binary, and poll
# until neither is present. Zombies (<defunct>) still match pgrep but hold no
# CPU, GPU or subscriptions, so they are filtered out by process state.
kill_nvblox(){
  docker exec "$NAME" bash -lc '
    live(){ for p in $(pgrep -x nvblox_node; pgrep -f "[r]os2 run nvblox_ros"); do
              case "$(ps -o stat= -p $p 2>/dev/null)" in Z*|"") ;; *) echo $p ;; esac
            done; }
    for i in $(seq 1 20); do
      pids="$(live)"; [ -z "$pids" ] && exit 0
      for p in $pids; do kill -9 $p 2>/dev/null; done
      sleep 0.3
    done
    exit 1' 2>/dev/null || echo "  ⚠ nvblox would not die — check: docker exec $NAME pgrep -a nvblox_node"
}

kill_match(){ docker exec "$NAME" bash -lc "for p in \$(pgrep -f '$1'); do kill -9 \$p 2>/dev/null; done; true" || true; }

launch_nvblox(){
  dexec "NVB=\$(python3 -c 'from ament_index_python.packages import get_package_share_directory as g; print(g(\"nvblox_examples_bringup\")+\"/config/nvblox/nvblox_base.yaml\")');
         exec ros2 run nvblox_ros nvblox_node --ros-args --params-file \$NVB --params-file $SRC/config/nvblox.yaml \
            -p use_color:=false \
            -r camera_0/depth/image:=$CAM_NS/depth/image_rect_raw \
            -r camera_0/depth/camera_info:=$CAM_NS/depth/camera_info > /tmp/nvblox.log 2>&1"
}

case "${1:-help}" in

# ── L1: the camera, and nothing else ────────────────────────────────────────
l1)
  check_no_old_stack
  docker rm -f "$NAME" >/dev/null 2>&1 || true
  mkdir -p "$(dirname "$HOSTSRC")/maps"
  # src mounted READ-ONLY: edit a node or config on the host, restart the layer,
  # and it takes effect with no image rebuild. /maps is the writable map store.
  docker run -d --name "$NAME" --entrypoint /bin/bash --runtime=nvidia --network=host --privileged \
    -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_DISABLE_REQUIRE=1 \
    -e ROS_DOMAIN_ID=0 \
    -v "$HOSTSRC":$SRC:ro \
    -v "$(dirname "$HOSTSRC")/maps":/maps \
    "$IMAGE" -c 'sleep infinity' >/dev/null
  sleep 2
  echo "[L1] D555 — stereo IR + depth + motion, emitter OFF, sync OFF"
  launch_cam
  cam_wait || { echo "  Container left running so you can retry after a power-cycle."; exit 1; }
  echo
  echo "  GATE L1 — the camera must hold these for 5 minutes:"
  assert_rate "$CAM_NS/infra1/image_rect_raw" 15 "camera IR left"
  assert_rate "$CAM_NS/depth/image_rect_raw"  10 "camera depth"
  echo
  echo "  Next: ./rover.sh l2      (cuVSLAM)"
  ;;

# ── L2: cuVSLAM ─────────────────────────────────────────────────────────────
l2)
  need_container
  echo "  checking L1 is still healthy before building on it..."
  assert_rate "$CAM_NS/infra1/image_rect_raw" 15 "camera IR left"
  # The static TF must exist BEFORE cuVSLAM, or the first poses land in a frame
  # tree with a missing link. z=0.163 was MEASURED with tools/floor_probe.py;
  # it was 0.200 and that 3.7 cm error mapped the floor as a wall. FACTS §2.
  echo "[L2] base_link -> camera0_link static TF (x 0.10, z 0.163 — measured)"
  kill_match "[s]tatic_transform_publisher"
  dexec "exec ros2 run tf2_ros static_transform_publisher --x 0.10 --y 0.0 --z 0.163 \
            --frame-id base_link --child-frame-id camera0_link > /tmp/basetf.log 2>&1"
  sleep 1
  echo "[L2] cuVSLAM  (publishes /odom and TF map->odom, odom->base_link)"
  kill_match "[c]uvslam_node.py"
  dexec "export LD_LIBRARY_PATH=$CU12:\$LD_LIBRARY_PATH; exec python3 $SRC/nodes/cuvslam_node.py > /tmp/cuvslam.log 2>&1"
  sleep 8
  # Rover body + trail: without these RViz shows bare coordinate axes and the
  # first thing anyone says is "I can't see the rover".
  kill_match "[r]over_marker.py"; kill_match "[r]over_trail.py"
  dexec "exec python3 $SRC/nodes/rover_marker.py > /tmp/rover_marker.log 2>&1"
  dexec "exec python3 $SRC/nodes/rover_trail.py  > /tmp/rover_trail.log 2>&1"
  echo
  echo "  GATE L2 — rate only. The REAL gate is metric:"
  assert_rate /odom 10 "cuVSLAM odom"
  echo
  echo "  Now prove it is HONEST, not just alive:"
  echo "      ./rover.sh measure 2.00      # push it 2.00 m by hand"
  echo "  It must read 2.00 m +-5%. A frozen cuVSLAM still says slam_pose_ok: true."
  ;;

# ── L3: fusion ──────────────────────────────────────────────────────────────
l3)
  need_container
  echo "  checking L2 is still healthy before building on it..."
  assert_rate /odom 10 "cuVSLAM odom"
  # cuVSLAM must stop publishing odom->base_link: the EKF owns that TF now, and
  # two publishers of the same transform fight and neither wins.
  echo "[L3] cuVSLAM restarted with publish_odom_tf:=false (EKF owns odom->base_link)"
  kill_match "[c]uvslam_node.py"
  sleep 2
  dexec "export LD_LIBRARY_PATH=$CU12:\$LD_LIBRARY_PATH; exec python3 $SRC/nodes/cuvslam_node.py --ros-args -p publish_odom_tf:=false > /tmp/cuvslam.log 2>&1"
  sleep 6
  echo "[L3] imu_to_base  (D555 gyro, re-framed into base_link)"
  kill_match "[i]mu_to_base.py"
  dexec "exec python3 $SRC/nodes/imu_to_base.py > /tmp/imu_to_base.log 2>&1"
  sleep 1
  echo "[L3] EKF  (/odom + /imu/base + /wheel_odom -> /odometry/filtered)"
  kill_match "[e]kf_node"
  dexec "exec ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_filter_node --params-file $SRC/config/ekf.yaml > /tmp/ekf.log 2>&1"
  sleep 3
  # Cross-checks cuVSLAM against /cmd_vel and /wheel_odom. cuVSLAM fails SILENTLY
  # on low-texture floors, and this is what stops motion acting on a dead pose.
  kill_match "[o]dom_health.py"
  dexec "exec python3 $SRC/nodes/odom_health.py > /tmp/odom_health.log 2>&1"
  sleep 2
  echo
  echo "  GATE L3:"
  assert_rate /odometry/filtered 10 "EKF odom"
  assert_rate /imu/base          50 "IMU gyro"
  z=$(rexec "timeout 8 ros2 topic echo /odometry/filtered --once 2>/dev/null | awk '/^      z:/{print \$2; exit}'" 2>/dev/null || echo "?")
  echo "      /odometry/filtered z = ${z:-?}   (MUST be 0.0 — two_d_mode pins it)"
  echo
  echo "  Next: ./rover.sh l4      (mapping)"
  ;;

# ── L4: mapping ─────────────────────────────────────────────────────────────
l4)
  need_container
  echo "  checking L3 is still healthy before building on it..."
  assert_rate /odometry/filtered 10 "EKF odom"
  echo "[L4] nvblox  (depth + fused pose -> TSDF -> ESDF -> 2D slice)"
  kill_nvblox
  sleep 2
  launch_nvblox
  sleep 8
  # depth_to_cloud lives HERE, not in L5 where it belongs by function, and the
  # reason is the camera (TODO §14). It is a SECOND subscriber on the raw depth
  # topic that FACTS §1 says not to touch, and the suspected killer is repeated
  # connect/disconnect TRANSITIONS on the DDS control channel. Starting it beside
  # nvblox means:
  #   * L4 is ONE attach event on the depth stream, not two
  #   * L5 (nav2) can be restarted as often as tuning needs — and it will be —
  #     without ever cycling the depth stream
  # It publishes nothing anyone consumes until collision_monitor starts at L5, so
  # running it early is free.
  echo "[L4] depth_to_cloud  (collision_monitor's obstacle source, started early)"
  kill_match "[d]epth_to_cloud.py"
  dexec "exec python3 $SRC/nodes/depth_to_cloud.py > /tmp/depth_to_cloud.log 2>&1"
  sleep 3
  echo
  echo "  GATE L4:"
  # Two different consumers, so check both: the occupancy grid is what RViz
  # draws, the map slice is what nav2's costmap actually eats. They can diverge.
  assert_rate /nvblox_node/static_occupancy_grid 5 "nvblox grid (RViz)"
  assert_rate /nvblox_node/static_map_slice      5 "nvblox slice (nav2)"
  n=$(docker exec "$NAME" bash -lc 'pgrep -x nvblox_node | while read p; do case "$(ps -o stat= -p $p)" in Z*) ;; *) echo x;; esac; done | wc -l')
  echo "      live nvblox nodes: $n   (MUST be 1 — two of them fight, FACTS §4)"
  [ "$n" = "1" ] || die "wrong number of nvblox nodes ($n). Run ./rover.sh l4 again."
  # TODO §3's gate. collision_monitor's source_timeout is 1.5 s and a stale source
  # makes it HOLD THE ROBOT AT ZERO, so an under-rate source does not lose you
  # protection — it pins the rover while nav2 plans happily. Catch it here, one
  # layer before anything can move.
  assert_rate /perception/depth_points 5 "depth_to_cloud (collision source, timeout 1.5 s)"
  echo
  echo "  The REAL gate is visual: drive the room and check walls are LINES,"
  echo "  not blobs. ./rover.sh view start"
  ;;

# ── L5: navigation — THIS MOVES THE ROBOT ───────────────────────────────────
l5)
  need_container
  echo "  checking L4 is still healthy before building on it..."
  assert_rate /nvblox_node/static_occupancy_grid 5 "nvblox slice"
  # depth_to_cloud is started by L4, deliberately (TODO §14) — it must NOT be
  # restarted here, because that would cycle the raw depth stream on every nav2
  # restart. Check it is alive and fast instead: below 5 Hz against the 1.5 s
  # source_timeout, collision_monitor holds the robot at zero and every goal dies
  # on "Failed to make progress".
  assert_rate /perception/depth_points 5 "depth_to_cloud (from L4 — do NOT restart it)"
  echo
  echo "  ⚠  L5 MOVES THE ROBOT. RViz's '2D Goal Pose' button sends a real goal."
  echo
  echo "[L5] safety_guard  (last gate before the wheels)"
  kill_match "[s]afety_guard.py"
  dexec "exec python3 $SRC/nodes/safety_guard.py > /tmp/safety_guard.log 2>&1"
  sleep 1
  echo "[L5] nav2  (planner + MPPI controller + BT with no blind recoveries)"
  kill_match "[n]avigation_launch.py"
  dexec "exec ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false \
            params_file:=$SRC/config/nav2.yaml > /tmp/nav2.log 2>&1"
  echo
  echo "  Give it ~15 s, then: ./rover.sh status"
  ;;

# ── health ──────────────────────────────────────────────────────────────────
status)
  need_container
  rexec "exec python3 $SRC/nodes/stack_status.py ${2:-}"
  ;;

measure)
  need_container
  # Tape-measure the pose. Safe to run any time — odometry topics only, never
  # the raw camera topics (subscribing to those takes the D555 offline).
  if [ -n "${2:-}" ]; then rexec "exec python3 $SRC/tools/odom_ruler.py --expect $2"
  else                     rexec "exec python3 $SRC/tools/odom_ruler.py"; fi
  ;;

floor)
  need_container
  # Where does the floor sit, according to the robot? Must read ~0.000.
  # Re-run after ANY camera remount, before trusting the map.
  rexec "exec python3 $SRC/tools/floor_probe.py"
  ;;

logs)
  need_container
  docker exec "$NAME" bash -lc "tail -n ${3:-60} -f /tmp/${2:-nvblox}.log"
  ;;

# ── the laptop view ─────────────────────────────────────────────────────────
view)
  echo
  echo "  laptop RViz — $LAPTOP_USER@$LAPTOP_IP"
  # Retry the ping: the laptop's wifi power-saves, so RTT swings 21 -> 128 ms and
  # the FIRST icmp packet is often dropped. A single ping reports a healthy,
  # logged-in laptop as "off" and aborts the whole command. FACTS §5.
  ping -c3 -W2 "$LAPTOP_IP" >/dev/null 2>&1 || die "$LAPTOP_IP does not answer ping — laptop off/asleep, or wrong IP (.12 is the ESP32)"
  echo "  ok  host is up"
  timeout 8 ssh -o BatchMode=yes -o ConnectTimeout=4 "$LAPTOP_USER@$LAPTOP_IP" true 2>/dev/null \
    || die "ssh failed. If this is .12 you are talking to the ESP32, not the laptop."
  echo "  ok  ssh works"
  # A desktop session is one that is NOT gdm and has a graphical type. On Wayland
  # `loginctl -p Display` is EMPTY, so the authoritative source for both the
  # display number and the auth cookie is the Xwayland process cmdline. FACTS §5.
  desk=$(timeout 8 ssh -o BatchMode=yes "$LAPTOP_USER@$LAPTOP_IP" '
    for s in $(loginctl list-sessions --no-legend 2>/dev/null | awk "{print \$1}"); do
      n=$(loginctl show-session "$s" -p Name --value 2>/dev/null)
      t=$(loginctl show-session "$s" -p Type --value 2>/dev/null)
      [ "$n" = "gdm" ] && continue
      [ "$t" = "x11" ] || [ "$t" = "wayland" ] || continue
      xw=$(pgrep -a Xwayland 2>/dev/null | head -1)
      if [ -n "$xw" ]; then
        d=$(echo "$xw" | grep -oE " :[0-9]+" | head -1 | tr -d " ")
        a=$(echo "$xw" | sed -nE "s/.* -auth ([^ ]+).*/\1/p")
      elif [ "$t" = "wayland" ]; then
        # Xwayland not started yet — GNOME starts it ON DEMAND. Do NOT guess
        # ":0 + ~/.Xauthority"; that is exactly what fails silently.
        echo "$n NO_XWAYLAND NO_XWAYLAND"; break
      else
        d=$(loginctl show-session "$s" -p Display --value 2>/dev/null); a=$HOME/.Xauthority
      fi
      echo "$n ${d:-:0} ${a:-$HOME/.Xauthority}"; break
    done' 2>/dev/null)
  [ -n "$desk" ] || { echo "  ✗ NOBODY IS LOGGED IN at the laptop desktop."; echo "    RViz over ssh renders into nothing and reports no error. Log in physically."; exit 1; }
  if [ "$(echo "$desk" | awk '{print $2}')" = "NO_XWAYLAND" ]; then
    echo "  ✗ Wayland session, but Xwayland is not running yet."
    echo "    Open any window on the laptop (a terminal is enough), then re-run."
    exit 1
  fi
  echo "  ok  desktop session: $desk"
  if timeout 8 ssh -o BatchMode=yes "$LAPTOP_USER@$LAPTOP_IP" 'pgrep -x rviz2 >/dev/null' 2>/dev/null; then
    echo "  ok  rviz2 already running — look at the laptop screen"
  elif [ "${2:-}" = "start" ]; then
    disp=$(echo "$desk" | awk '{print $2}'); xa=$(echo "$desk" | awk '{print $3}')
    echo "  launching rviz2 on DISPLAY=$disp"
    timeout 10 ssh -o BatchMode=yes "$LAPTOP_USER@$LAPTOP_IP" \
      "DISPLAY=$disp XAUTHORITY=$xa XDG_RUNTIME_DIR=/run/user/\$(id -u) nohup bash ~/rover_view.sh >/tmp/rviz.log 2>&1 &" >/dev/null 2>&1 || true
    sleep 6
    # pgrep -x, never -f: -f matches our own ssh command line and always "succeeds".
    if timeout 8 ssh -o BatchMode=yes "$LAPTOP_USER@$LAPTOP_IP" 'pgrep -x rviz2 >/dev/null' 2>/dev/null; then
      echo "  ok  rviz2 started"
    else
      echo "  ✗ rviz2 did not stay up. Its error is ON THE LAPTOP:"
      timeout 8 ssh -o BatchMode=yes "$LAPTOP_USER@$LAPTOP_IP" 'tail -6 /tmp/rviz.log' 2>/dev/null | sed 's/^/       /'
    fi
  else
    echo "  --  rviz2 not running.  ./rover.sh view start"
  fi
  echo
  echo "  ⚠ Do NOT enable the camera Image display — it takes the map from 9.4 Hz to 0."
  ;;

# ── stopping ────────────────────────────────────────────────────────────────
stop)
  need_container
  echo "  E-STOP: killing everything that can command motion"
  kill_match "[n]avigation_launch.py"
  kill_match "[s]afety_guard.py"
  # Hold zeros briefly so the ESP32's last received command is a stop, not
  # whatever was in flight when nav2 died.
  rexec "timeout 3 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true"
  echo "  stopped. Perception layers (l1-l4) left running."
  ;;

down)
  docker rm -f "$NAME" >/dev/null 2>&1 && echo "  container removed" || echo "  nothing to remove"
  ;;

*)
  sed -n '2,30p' "$0" | sed 's/^# \?//'
  ;;
esac
