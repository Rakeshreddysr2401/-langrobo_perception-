#!/bin/bash
# Orin nav stack — cuVSLAM + nvblox + nav2 + vision, ONE self-contained container.
# Everything (nodes, configs, YOLO weights, behavior tree) is baked into the image
# at /opt/orin-nav. No workspace mounts.
#
#   ./run_stack.sh up       # camera + base TF + cuVSLAM (FULL SLAM: map->odom live,
#                           #   /slam/save_map + /slam/localize, maps persist in ./maps) + nvblox
#                           #   ABORTS with a clear power-cycle message if the D555 never streams
#   ./run_stack.sh cam      # relaunch ONLY the D555 camera node (use after a power-cycle;
#                           #   cuVSLAM re-locks automatically once images return)
#   ./run_stack.sh nav2     # nav2 (NO blind recoveries BT; map frame from cuVSLAM)
#   ./run_stack.sh vision   # YOLO + pixel_to_goal + SAFETY GUARD
#                           #   chain: controller_server -> /cmd_vel_nav -> velocity_smoother
#                           #     -> /cmd_vel_smoothed -> collision_monitor -> /cmd_vel_shim
#                           #     -> safety_guard -> /cmd_vel -> ESP32   (single path, no bypass)
#   ./run_stack.sh status   # one-shot health — measures RATES (not publisher counts),
#                           #   TF freshness and /odom/health pose trust, then SLAM/nav2/
#                           #   safety/ESP32 link. Optional arg = sample window in seconds.
#   ./run_stack.sh view     # laptop RViz: resolve it, prove someone is logged in, say what
#                           #   to run. 'view start' also launches it remotely.
#   ./run_stack.sh stop     # E-STOP: kill nav/motion nodes + zero /cmd_vel
#   ./run_stack.sh remap    # fresh map/pose: restart cuVSLAM + nvblox (camera untouched)
#   ./run_stack.sh rviz     # RViz nav2 view on the Jetson monitor (:1)
#   ./run_stack.sh logs <realsense|cuvslam|nvblox|nav2|detections_3d|...>
#   ./run_stack.sh down     # hard stop: remove the container (ultimate e-stop)
#
# CAMERA (D555): ethernet/PoE DDS at 192.168.11.55 — NOT USB. It pings even when its
# on-camera DDS server is dead, so ping is NOT a health check. If the realsense log
# shows "No RealSense devices were found" the camera must be PHYSICALLY power-cycled
# (unplug PoE cable ~5s, replug) — no software restart recovers a dead DDS server.
#
# SAFETY (tethered rover): BT has NO Spin/BackUp; speed caps live in nav2.yaml
# (MPPI vx_max 0.30 / wz_max 1.0, velocity_smoother 0.25 / 1.0).
# Rover motion tests: pause YOLO first (CPU overload -> cuVSLAM pose jumps).
set -eo pipefail
IMAGE=${IMAGE:-orin-nav:1.1}
NAME=${NAME:-orin_nav}
NAV=/opt/orin-nav
CAM_NS=/camera/camera0
# cuVSLAM wheel dir FIRST (its libcuvslam.so must beat Isaac ROS's Thor build at
# /opt/ros/jazzy/lib), then the CUDA-12 user-space libs.
CU12='/usr/local/lib/python3.12/dist-packages/cuvslam:/usr/local/lib/python3.12/dist-packages/nvidia/cuda_runtime/lib:/usr/local/lib/python3.12/dist-packages/nvidia/cublas/lib:/usr/local/lib/python3.12/dist-packages/nvidia/cusolver/lib:/usr/local/lib/python3.12/dist-packages/nvidia/cusparse/lib:/usr/local/lib/python3.12/dist-packages/nvidia/nvjitlink/lib:/usr/local/lib/python3.12/dist-packages/nvidia/nvtx/lib'

dexec(){ docker exec -d "$NAME" bash -lc "unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE; export ROS_DOMAIN_ID=0; source /opt/ros/jazzy/setup.bash; $1"; }
rexec(){ docker exec "$NAME" bash -lc "unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE; export ROS_DOMAIN_ID=0; source /opt/ros/jazzy/setup.bash; $1"; }

# Launch the D555 over DDS/ethernet. Shared by `up` and `cam` so the exact
# stereo-IR + emitter + sync config lives in exactly one place.
#
# enable_sync:=false is CRITICAL (2026-07-22): with the camera's cross-stream
# frame-syncer ON, enabling color gates the infra1/infra2 pair behind color
# alignment over DDS and STARVES the IR stereo to <1 Hz -> cuVSLAM stalls ->
# no /odom, no map->odom->base_link TF -> RViz map goes blank and nvblox builds
# nothing. It is NOT a bandwidth issue (even 424x240x6 color starved IR) and NO
# camera power-cycle fixes it. With sync OFF, infra1/infra2 stay hardware-synced
# within the depth module (cuVSLAM stereo is fine) and color+IR coexist:
# IR ~22-23 Hz, /odom ~18 Hz, color ~7 Hz all together. Do NOT set this back to
# true. See memory: d555-color-starves-ir-slam.
#
# emitter_enabled:=0 is CRITICAL (2026-08-09): the IR PROJECTOR is rigidly
# mounted on the camera, so its dot pattern is re-painted from the camera's
# viewpoint every frame -> on a low-texture/reflective floor the dots stay
# ~fixed in the image as the rig TRANSLATES, and cuVSLAM (which tracks infra1/2
# features) sees almost no motion -> it UNDER-REPORTS translation badly. Measured
# emitter ON: 100 cm real push -> 26 cm /odom (~4x under, floor-dependent);
# emitter OFF: 100 cm -> 97 cm (3%). Passive-stereo DEPTH stays metric-correct
# either way (verified 140 cm wall -> 1.40 m), so nvblox still works; emitter-off
# depth is just noisier on textureless surfaces. Localization > perfect depth, so
# emitter stays OFF. Refinement if depth suffers: emitter_on_off:=true
# (alternating) + feed cuVSLAM only the emitter-off frames.
launch_cam(){
  dexec 'printf "{\"context\":{\"dds\":{\"enabled\":true,\"domain\":0}}}" > ~/.realsense-config.json;
         export LD_LIBRARY_PATH=/root/librealsense/install/lib:$LD_LIBRARY_PATH;
         exec ros2 launch realsense2_camera rs_launch.py camera_name:=camera0 \
            enable_infra1:=true enable_infra2:=true depth_module.infra_profile:=896x504x30 \
            depth_module.emitter_enabled:=0 enable_depth:=true enable_color:=true enable_motion:=true enable_sync:=false \
            > /tmp/realsense.log 2>&1'
}

# True once the D555 is actually streaming (infra1 image has a live publisher).
# This is the real health signal — the camera pings even when its DDS is dead.
cam_ready(){
  rexec "n=\$(ros2 topic info $CAM_NS/infra1/image_rect_raw 2>/dev/null | awk '/Publisher count/{print \$3}'); [ \"\${n:-0}\" -ge 1 ]" >/dev/null 2>&1
}

# Poll up to ~40s for the camera to stream. If it never does AND the realsense log
# is reporting no devices, the on-camera DDS server is dead -> the ONLY fix is a
# physical power-cycle. Print exact steps and return non-zero (never proceed blind).
cam_wait(){
  local t=0
  while [ "$t" -lt 40 ]; do
    if cam_ready; then echo "      D555 streaming — OK"; return 0; fi
    t=$((t+3)); sleep 3
  done
  echo ""
  echo "  ✗ D555 is NOT streaming after 40s."
  if docker exec "$NAME" grep -q "No RealSense devices were found" /tmp/realsense.log 2>/dev/null; then
    echo "    The camera's on-DDS server is offline (it may still PING — that does not count)."
    echo "    FIX: physically power-cycle the D555 — unplug its PoE ethernet cable ~5s,"
    echo "         replug, wait ~15s for it to boot, then run:  ./run_stack.sh cam"
  else
    echo "    Check the log:  ./run_stack.sh logs realsense"
  fi
  return 1
}

case "${1:-up}" in
up)
  docker rm -f "$NAME" >/dev/null 2>&1 || true
  mkdir -p "$HOME/orin-nav-stack/maps"
  # repo mounted RO over the baked /opt/orin-nav so code/config edits take
  # effect on restart without an image rebuild; /maps is the rw SLAM-map store.
  docker run -d --name "$NAME" --entrypoint /bin/bash --runtime=nvidia --network=host --privileged \
    -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_DISABLE_REQUIRE=1 -e ROS_DOMAIN_ID=0 \
    -e DISPLAY=:1 -e XAUTHORITY=/root/.Xauthority \
    -v /run/user/1000/gdm/Xauthority:/root/.Xauthority:ro -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$HOME/orin-nav-stack":/opt/orin-nav:ro \
    -v "$HOME/orin-nav-stack/maps":/maps \
    "$IMAGE" -c 'sleep infinity' >/dev/null
  sleep 2
  echo "[1/4] D555 (stereo IR + depth + color + motion, emitter OFF — cuVSLAM scale)"
  launch_cam
  # Gate on real streaming instead of a blind sleep — if the camera never comes
  # up we abort HERE with a power-cycle message rather than starting SLAM blind.
  cam_wait || { echo "  Aborting bring-up. Container left running so 'cam' can retry after power-cycle."; exit 1; }
  # z was 0.20 until 2026-08-10. MEASURED with nodes/floor_probe.py: open floor
  # deprojected to z = +0.037 m in base_link (19k points, 16-84% spread only
  # 0.035-0.041), i.e. the TF overstated the camera height by 3.7 cm. base_link's
  # origin is on the ground, so floor MUST read ~0.000.
  #
  # That 4 cm error is what made nvblox map the floor as an obstacle (the old
  # esdf_slice_min_height of 0.05 sat 1 cm above the apparent floor, inside depth
  # noise) -> inflation plateau, MPPI crawling at 0.082 m/s, "Failed to make
  # progress". It also biased the z of every 3D detection.
  #
  # Re-verify after ANY camera remount:  python3 nodes/floor_probe.py  -> want ~0.000
  echo "[2/4] base_link -> camera0_link static TF (x0.10 z0.163 — floor_probe measured)"
  dexec 'exec ros2 run tf2_ros static_transform_publisher --x 0.10 --y 0.0 --z 0.163 \
            --frame-id base_link --child-frame-id camera0_link > /tmp/basetf.log 2>&1'
  # Rover body for RViz. There is no URDF in this repo, so without this the rover
  # is drawn as bare TF axes and "I cannot see the rover" is the first thing you
  # hit in learn/00-setup.md. Also draws the ~87 deg FOV wedge, which is the
  # honest picture of how much it is blind to.
  dexec "exec python3 $NAV/nodes/rover_marker.py > /tmp/rover_marker.log 2>&1"
  sleep 2
  echo "[3/4] cuVSLAM wrapper node (pyCuVSLAM cu12; publishes /odom too)"
  dexec "export LD_LIBRARY_PATH=$CU12:\$LD_LIBRARY_PATH; exec python3 $NAV/cuvslam_ros_node.py > /tmp/cuvslam.log 2>&1"
  sleep 6
  echo "[4/4] nvblox (depth + cuVSLAM pose -> 3D map/ESDF)"
  dexec "NVB=\$(python3 -c 'from ament_index_python.packages import get_package_share_directory as g; print(g(\"nvblox_examples_bringup\")+\"/config/nvblox/nvblox_base.yaml\")');
         exec ros2 run nvblox_ros nvblox_node --ros-args --params-file \$NVB --params-file $NAV/config/nvblox.yaml \
            -p use_color:=false \
            -r camera_0/depth/image:=/camera/camera0/depth/image_rect_raw \
            -r camera_0/depth/camera_info:=/camera/camera0/depth/camera_info > /tmp/nvblox.log 2>&1"
  echo "perception up. Next: ./run_stack.sh nav2   then   ./run_stack.sh vision"
  ;;
cam)
  # Relaunch ONLY the D555 (e.g. after a mid-session DDS drop + power-cycle).
  # cuVSLAM keeps running and re-locks automatically once images return.
  docker exec "$NAME" bash -lc 'for pid in $(pgrep -f "[r]s_launch"); do kill -9 $pid 2>/dev/null; done; for pid in $(pgrep -f "[r]ealsense2_camera_node"); do kill -9 $pid 2>/dev/null; done; true' || true
  sleep 3
  echo "[cam] relaunching D555 over DDS/ethernet"
  launch_cam
  cam_wait && echo "  camera back — cuVSLAM will re-lock; check ./run_stack.sh status"
  ;;
nav2)
  # map->odom now comes LIVE from cuvslam_ros_node (SLAM correction) — no
  # static publisher here anymore.
  #
  # Kill any existing nav2 FIRST (added 2026-08-10). Re-running this command
  # used to stack a second full nav2 stack on top of the first: duplicate
  # action servers on /navigate_to_pose and /follow_path, which surfaces as
  #   "unknown goal response, ignoring..."
  #   "BtActionNode::Tick: invalid status value"
  # and goals aborting at random. Same guard 'fuse'/'remap' already use.
  docker exec "$NAME" bash -lc '
    for pid in $(pgrep -f "[n]avigation_launch.py"); do kill -9 $pid 2>/dev/null; done
    ps -eo pid,comm | grep -iE "bt_navigat|controller_serv|behavior_ser|velocity_smo|collision_mon|planner_serv|smoother_ser|lifecycle_man|waypoint|docking|route_serv" | awk "{print \$1}" | xargs -r kill -9 2>/dev/null
    true' >/dev/null 2>&1 || true
  sleep 4
  dexec "exec ros2 launch nav2_bringup navigation_launch.py params_file:=$NAV/config/nav2.yaml \
            use_sim_time:=False use_composition:=False autostart:=True use_respawn:=False > /tmp/nav2.log 2>&1"
  echo "nav2 launching (no-blind-recovery BT). Wait for: ./run_stack.sh logs nav2 -> 'Managed nodes are active'"
  ;;
vision)
  dexec "exec python3 $NAV/nodes/detections_3d.py --ros-args -p model:=$NAV/models/yolov8n.pt > /tmp/detections_3d.log 2>&1"
  dexec "exec python3 $NAV/nodes/pixel_to_goal.py > /tmp/pixel_to_goal.log 2>&1"
  # nav2's collision_monitor obstacle source. MUST be running before nav2 drives:
  # with no fresh source the monitor fail-safes to "stop due to invalid source"
  # and holds the rover at zero. See nodes/depth_to_cloud.py.
  dexec "exec python3 $NAV/nodes/depth_to_cloud.py > /tmp/depth_to_cloud.log 2>&1"
  # cmd_vel_deadband.py is DELIBERATELY NOT started (2026-08-10). It was built for
  # the old open-loop L298N firmware and re-floored every command to vx>=0.20 /
  # wz>=0.80, which left MPPI with no fine control authority (bang-bang steering ->
  # the rover weaved and spun instead of tracking the path). Firmware v2's 50 Hz
  # encoder PID + gMinDuty now does the static-friction job properly.
  dexec "exec python3 $NAV/nodes/safety_guard.py > /tmp/safety_guard.log 2>&1"
  # imu_to_base is owned by 'fuse' now (it only feeds the EKF). Run './run_stack.sh
  # fuse' for visual+IMU odometry; plain vision stays visual-odom-only.
  echo "vision + goal + safety guard starting (no deadband shim — firmware PID owns low speed)"
  ;;
fuse)
  # Visual-inertial fusion: cuVSLAM /odom + D555 gyro -> robot_localization EKF,
  # which OWNS the odom->base_link TF. So cuVSLAM is restarted with
  # publish_odom_tf:=false (no TF fight), then imu_to_base + ekf come up.
  # Run after 'up'. Fresh fusion reset = this command (the fusion-mode 'remap').
  # See config/ekf.yaml: two_d_mode + gyro-yaw-only (accel NOT fused — diverges).
  docker exec "$NAME" bash -lc 'for pid in $(pgrep -f "[c]uvslam_ros_node.py"); do kill -9 $pid 2>/dev/null; done; for pid in $(pgrep -f "[i]mu_to_base.py"); do kill -9 $pid 2>/dev/null; done; for pid in $(pgrep -f "[e]kf_node"); do kill -9 $pid 2>/dev/null; done; for pid in $(pgrep -x nvblox_node); do kill -9 $pid 2>/dev/null; done; true' || true
  sleep 3
  echo "[1/4] cuVSLAM (odom->base_link TF OFF — EKF owns it)"
  dexec "export LD_LIBRARY_PATH=$CU12:\$LD_LIBRARY_PATH; exec python3 $NAV/cuvslam_ros_node.py --ros-args -p publish_odom_tf:=false > /tmp/cuvslam.log 2>&1"
  sleep 5
  echo "[2/4] imu_to_base (D555 motion -> /imu/base, gyro in base_link)"
  dexec "exec python3 $NAV/nodes/imu_to_base.py > /tmp/imu_to_base.log 2>&1"
  sleep 1
  echo "[3/4] robot_localization EKF (/odom + /imu/base -> odom->base_link)"
  dexec "exec ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_filter_node --params-file $NAV/config/ekf.yaml > /tmp/ekf.log 2>&1"
  sleep 3
  # Cross-checks cuVSLAM against /cmd_vel and /wheel_odom and publishes
  # /odom/health. cuVSLAM fails SILENTLY on low-texture floors (reports
  # slam=True while under-reporting travel and drifting yaw) — this is what
  # stops the motion behaviours acting on a pose that has gone bad.
  dexec "exec python3 $NAV/nodes/odom_health.py > /tmp/odom_health.log 2>&1"
  echo "[4/4] nvblox (depth + fused pose -> 3D map/ESDF)"
  dexec "NVB=\$(python3 -c 'from ament_index_python.packages import get_package_share_directory as g; print(g(\"nvblox_examples_bringup\")+\"/config/nvblox/nvblox_base.yaml\")');
         exec ros2 run nvblox_ros nvblox_node --ros-args --params-file \$NVB --params-file $NAV/config/nvblox.yaml \
            -p use_color:=false -r camera_0/depth/image:=/camera/camera0/depth/image_rect_raw \
            -r camera_0/depth/camera_info:=/camera/camera0/depth/camera_info > /tmp/nvblox.log 2>&1"
  echo "fusion up: odom->base_link now from the EKF (/odometry/filtered). Verify: ./run_stack.sh status"
  ;;
stop)
  # E-STOP: kill everything that can command motion, then hold zeros briefly.
  docker exec "$NAME" bash -lc '
    for pid in $(pgrep -f "[n]avigation_launch.py"); do kill -9 $pid 2>/dev/null; done
    for pid in $(pgrep -f "[c]md_vel_deadband"); do kill -9 $pid 2>/dev/null; done
    for pid in $(pgrep -f "[s]afety_guard"); do kill -9 $pid 2>/dev/null; done
    for pid in $(pgrep -f "[v]isual_approach"); do kill -9 $pid 2>/dev/null; done
    for pid in $(pgrep -f "[d]rive_test"); do kill -9 $pid 2>/dev/null; done
    ps -eo pid,comm | grep -iE "bt_navigat|controller_serv|behavior_ser|velocity_smo|collision_mon|planner_serv|smoother_ser|lifecycle_man|waypoint|docking" | awk "{print \$1}" | xargs -r kill -9 2>/dev/null
    sleep 1
    source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0
    for i in 1 2 3 4 5 6; do ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1; done
    echo "E-STOP: motion nodes killed, zeros sent (rover watchdog holds stop)"' \
  || { echo "container not running — using hard stop"; docker rm -f "$NAME" >/dev/null 2>&1; }
  ;;
remap)
  # bracket patterns so pgrep -f can't match this wrapper shell itself (a
  # self-match kill -9'd the exec -> exit 137 -> set -e aborted remap silently)
  # also kill any EKF/imu relay so cuVSLAM (restarted with its own odom TF ON)
  # doesn't fight the EKF for odom->base_link. 'remap' = clean visual-only reset;
  # 'fuse' = clean fusion reset.
  # Was fusion running before we tore it down? If so the user is about to be
  # silently DOWNGRADED to visual-only, and that has a real cost (see below).
  WAS_FUSED=$(docker exec "$NAME" bash -lc 'pgrep -f "[e]kf_node" >/dev/null && echo yes || echo no' 2>/dev/null || echo no)
  docker exec "$NAME" bash -lc 'for pid in $(pgrep -f "[c]uvslam_ros_node.py"); do kill -9 $pid 2>/dev/null; done; for pid in $(pgrep -f "[e]kf_node"); do kill -9 $pid 2>/dev/null; done; for pid in $(pgrep -f "[i]mu_to_base.py"); do kill -9 $pid 2>/dev/null; done; for pid in $(pgrep -x nvblox_node); do kill -9 $pid 2>/dev/null; done; true' || true
  sleep 3
  dexec "export LD_LIBRARY_PATH=$CU12:\$LD_LIBRARY_PATH; exec python3 $NAV/cuvslam_ros_node.py > /tmp/cuvslam.log 2>&1"
  sleep 5
  dexec "NVB=\$(python3 -c 'from ament_index_python.packages import get_package_share_directory as g; print(g(\"nvblox_examples_bringup\")+\"/config/nvblox/nvblox_base.yaml\")');
         exec ros2 run nvblox_ros nvblox_node --ros-args --params-file \$NVB --params-file $NAV/config/nvblox.yaml \
            -p use_color:=false -r camera_0/depth/image:=/camera/camera0/depth/image_rect_raw \
            -r camera_0/depth/camera_info:=/camera/camera0/depth/camera_info > /tmp/nvblox.log 2>&1"
  echo "fresh map/pose: cuVSLAM + nvblox restarted (camera untouched)"
  # DIAGNOSED 2026-08-11: a user remapped, drove a whole room, and got a smeared
  # map with thick blobs instead of walls. Cause: visual-only cuVSLAM lets odom z
  # DRIFT (measured -0.27 m here; README/ekf.yaml record ~0.45 m). nvblox slices
  # obstacles in a fixed odom-z band (0.12-0.40), so a sinking pose slides that
  # band through the floor and back, integrating floor as wall the whole way.
  # The EKF's two_d_mode pins z to 0 and stops it dead. Never silently leave
  # someone in visual-only after a remap again.
  echo
  echo "  ⚠ You are now VISUAL-ONLY (no EKF). odom z will DRIFT, and nvblox slices"
  echo "    obstacles in a fixed odom-z band — a drifting z smears the map and turns"
  echo "    floor into fake walls. Fine for a quick pose check; NOT for mapping."
  if [ "$WAS_FUSED" = "yes" ]; then
    echo "    Fusion WAS running before this remap. Get it back:  ./run_stack.sh fuse"
  else
    echo "    Before you drive and map:                           ./run_stack.sh fuse"
  fi
  ;;
rviz)
  DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority xhost +local:root >/dev/null 2>&1 || true
  dexec "export DISPLAY=:1 XAUTHORITY=/root/.Xauthority; exec rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz > /tmp/rviz.log 2>&1"
  echo "RViz nav2 view on the Jetson monitor (:1)"
  ;;
status)
  # One-shot health of every layer.
  #
  # This used to print PUBLISHER COUNTS. That is the wrong instrument: every failure
  # this rig has had was a RATE COLLAPSE with the publisher count still sitting at 1
  # (IR 22->0.97 Hz, cuVSLAM frozen but /odom's publisher still registered, wheel_state
  # at 1 Hz instead of 20). nodes/stack_status.py measures actual rates, TF freshness
  # and pose trust; the lifecycle/link checks that it can't do stay here.
  if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
    echo "container '$NAME' not running — start with ./run_stack.sh up"; exit 1
  fi
  rexec "python3 $NAV/nodes/stack_status.py --window ${2:-4}" || true
  rexec '
    safe=$(timeout 5 ros2 topic echo /safety/state --once 2>/dev/null | grep -o "data: .*" | head -1)
    slam=$(timeout 5 ros2 topic echo /slam/status --once 2>/dev/null | grep -o "\"slam_pose_ok\": [a-z]*" | head -1)
    esp=$(ros2 topic info /cmd_vel 2>/dev/null | awk "/Subscription count/{print \$3}")
    # LIVE nav2 check: bt_navigator lifecycle state (not a stale /tmp/nav2.log grep,
    # which stays "Managed nodes are active" even after nav2 has crashed).
    nav=$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -o "^active")
    printf "  cuVSLAM        %s\n" "${slam:-<no /slam/status>}"
    printf "  safety         %s\n" "${safe:-<no /safety/state>}"
    # >=1 because odom_health also subscribes to /cmd_vel (it cross-checks commanded
    # vs visual travel), so this is 2 with fusion up. 0 = the ESP32 is NOT linked.
    printf "  ESP32 wheels (/cmd_vel subs):   %s\n" "${esp:-0}  (0 = wheels NOT linked)"
    printf "  nav2           %s\n" "${nav:-<not active / not started>}"
    echo
  '
  ;;
view)
  # Locate the laptop RViz view and say plainly why it is not showing.
  #
  # Two traps, both of which have cost a session:
  #   1. The laptop's IP MOVED (.12 -> .10). DHCP handed .12 to the ESP32, so the
  #      documented address ssh-refuses and looks like "the laptop is off".
  #   2. If the laptop sits at the GDM login screen, an rviz launched over ssh runs
  #      INVISIBLY and exits non-zero nowhere. Nothing is broken; nobody is logged in.
  LAPTOP_USER=${LAPTOP_USER:-rakhi24}
  LAPTOP_IP=${LAPTOP_IP:-192.168.1.10}
  echo
  echo "  laptop RViz view — $LAPTOP_USER@$LAPTOP_IP"
  echo "  (override with: LAPTOP_IP=x.x.x.x ./run_stack.sh view)"
  echo
  # Retry the ping. Measured 2026-08-11: the laptop's wifi power-saves, so RTT swings
  # 21 -> 128 ms and the FIRST icmp packet is often dropped while the radio wakes up.
  # A single `ping -c1` therefore reports a healthy, logged-in laptop as "off" — which
  # then aborts the whole view command. 3 tries costs nothing and removes the false alarm.
  if ! ping -c3 -W2 "$LAPTOP_IP" >/dev/null 2>&1; then
    echo "  ✗ $LAPTOP_IP does not respond to ping — laptop off, asleep, or on another network."
    echo "    Find it:  getent hosts rover-esp32.local   # make sure you are not looking at the ESP32"
    exit 1
  fi
  echo "  ok  host is up"
  if ! timeout 8 ssh -o BatchMode=yes -o ConnectTimeout=4 "$LAPTOP_USER@$LAPTOP_IP" true 2>/dev/null; then
    echo "  ✗ ssh refused/failed. If this is 192.168.1.12 you are talking to the ESP32,"
    echo "    not the laptop (DHCP swapped them 2026-08-10). The laptop is 192.168.1.10."
    exit 1
  fi
  echo "  ok  ssh works"
  # A desktop session = a session that is NOT gdm and has a real graphical type.
  # Emits: "<user> <display> <xauthority>".
  #
  # Trap 3 (found 2026-08-11): on a WAYLAND session `loginctl -p Display` is EMPTY,
  # so the old code fell back to a guessed ":0" and passed NO auth cookie. rviz2 then
  # died instantly with "Authorization required, but no authorization protocol
  # specified / could not connect to display :0" — into /tmp/rviz.log on the laptop,
  # where nobody looks. The authoritative source for BOTH the display number and the
  # cookie path is the Xwayland process cmdline (it is started with -auth <path>).
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
        # Wayland session but Xwayland is NOT up yet (GNOME starts it on demand).
        # Do NOT guess — guessing ":0 + ~/.Xauthority" is exactly what produced the
        # silent "could not connect to display :0" failure on 2026-08-11. Say so.
        echo "$n NO_XWAYLAND NO_XWAYLAND"; break
      else
        d=$(loginctl show-session "$s" -p Display --value 2>/dev/null)
        a=$HOME/.Xauthority
      fi
      echo "$n ${d:-:0} ${a:-$HOME/.Xauthority}"; break
    done' 2>/dev/null)
  if [ -z "$desk" ]; then
    echo "  ✗ NOBODY IS LOGGED IN to the laptop desktop (only gdm holds the seat)."
    echo
    echo "    RViz CANNOT display in this state — launching it over ssh runs it invisibly"
    echo "    with no error. This is the #1 cause of 'RViz shows nothing'."
    echo
    echo "    FIX: physically log into the laptop, then on the laptop run:"
    echo "         bash ~/rover_view.sh"
    exit 1
  fi
  if [ "$(echo "$desk" | awk '{print $2}')" = "NO_XWAYLAND" ]; then
    echo "  ✗ Laptop is on a WAYLAND session but Xwayland is not running yet."
    echo "    rviz2 is an X11 app — it has nothing to draw into, and would die with"
    echo "    'could not connect to display :0' in /tmp/rviz.log ON THE LAPTOP."
    echo
    echo "    FIX: open any window on the laptop desktop (a terminal is enough), then"
    echo "         re-run this. GNOME starts Xwayland on demand, so it appears once"
    echo "         something asks for X. Confirm with:  pgrep -a Xwayland"
    exit 1
  fi
  echo "  ok  desktop session: $desk"
  if timeout 8 ssh -o BatchMode=yes "$LAPTOP_USER@$LAPTOP_IP" 'pgrep -x rviz2 >/dev/null' 2>/dev/null; then
    echo "  ok  rviz2 is ALREADY running — look at the laptop screen"
  else
    echo "  --  rviz2 not running. Start it ON THE LAPTOP:   bash ~/rover_view.sh"
    echo "      (or from here:  ./run_stack.sh view start)"
  fi
  if [ "${2:-}" = "start" ]; then
    disp=$(echo "$desk" | awk '{print $2}')
    xa=$(echo "$desk" | awk '{print $3}')
    echo "  launching rviz2 remotely on DISPLAY=$disp (XAUTHORITY=$xa) ..."
    timeout 10 ssh -o BatchMode=yes "$LAPTOP_USER@$LAPTOP_IP" \
      "DISPLAY=$disp XAUTHORITY=$xa XDG_RUNTIME_DIR=/run/user/\$(id -u) nohup bash ~/rover_view.sh >/tmp/rviz.log 2>&1 &" \
      >/dev/null 2>&1 || true
    sleep 6
    if timeout 8 ssh -o BatchMode=yes "$LAPTOP_USER@$LAPTOP_IP" 'pgrep -x rviz2 >/dev/null' 2>/dev/null; then
      echo "  ok  rviz2 started (verified with pgrep -x, not -f — -f matches our own ssh cmdline)"
    else
      echo "  ✗ rviz2 did not stay up. Its error is ON THE LAPTOP, not here:"
      timeout 8 ssh -o BatchMode=yes "$LAPTOP_USER@$LAPTOP_IP" 'tail -6 /tmp/rviz.log' 2>/dev/null | sed 's/^/       /'
    fi
  fi
  echo
  echo "  ⚠ RViz's '2D Goal Pose' button sends a REAL nav goal — the rover moves."
  echo
  ;;
logs) docker exec "$NAME" tail -n 40 "/tmp/${2:-cuvslam}.log";;
down) docker rm -f "$NAME" >/dev/null 2>&1 && echo "removed $NAME (hard stop)";;
*) echo "usage: $0 {up|cam|nav2|vision|fuse|status [secs]|view [start]|stop|remap|rviz|logs <name>|down}";;
esac
