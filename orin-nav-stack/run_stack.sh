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
#   ./run_stack.sh vision   # YOLO + pixel_to_goal + motor shim + SAFETY GUARD + imu_to_base
#                           #   chain: /cmd_vel_nav -> deadband -> /cmd_vel_shim -> guard -> /cmd_vel
#   ./run_stack.sh status   # one-shot health: camera / SLAM / nav2 / safety / ESP32 wheel link
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
# SAFETY (tethered rover): BT has NO Spin/BackUp; deadband caps vx<=0.22 wz<=0.90.
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
  echo "[2/4] base_link -> camera0_link static TF (x0.10 z0.20)"
  dexec 'exec ros2 run tf2_ros static_transform_publisher --x 0.10 --y 0.0 --z 0.20 \
            --frame-id base_link --child-frame-id camera0_link > /tmp/basetf.log 2>&1'
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
  dexec "exec ros2 launch nav2_bringup navigation_launch.py params_file:=$NAV/config/nav2.yaml \
            use_sim_time:=False use_composition:=False autostart:=True use_respawn:=False > /tmp/nav2.log 2>&1"
  echo "nav2 launching (no-blind-recovery BT). Wait for: ./run_stack.sh logs nav2 -> 'Managed nodes are active'"
  ;;
vision)
  dexec "exec python3 $NAV/nodes/detections_3d.py --ros-args -p model:=$NAV/models/yolov8n.pt > /tmp/detections_3d.log 2>&1"
  dexec "exec python3 $NAV/nodes/pixel_to_goal.py > /tmp/pixel_to_goal.log 2>&1"
  dexec "exec python3 $NAV/nodes/cmd_vel_deadband.py > /tmp/cmd_vel_deadband.log 2>&1"
  dexec "exec python3 $NAV/nodes/safety_guard.py > /tmp/safety_guard.log 2>&1"
  # imu_to_base is owned by 'fuse' now (it only feeds the EKF). Run './run_stack.sh
  # fuse' for visual+IMU odometry; plain vision stays visual-odom-only.
  echo "vision + goal + motor shim (vx<=0.22 wz<=0.90) + safety guard starting"
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
  docker exec "$NAME" bash -lc 'for pid in $(pgrep -f "[c]uvslam_ros_node.py"); do kill -9 $pid 2>/dev/null; done; for pid in $(pgrep -f "[e]kf_node"); do kill -9 $pid 2>/dev/null; done; for pid in $(pgrep -f "[i]mu_to_base.py"); do kill -9 $pid 2>/dev/null; done; for pid in $(pgrep -x nvblox_node); do kill -9 $pid 2>/dev/null; done; true' || true
  sleep 3
  dexec "export LD_LIBRARY_PATH=$CU12:\$LD_LIBRARY_PATH; exec python3 $NAV/cuvslam_ros_node.py > /tmp/cuvslam.log 2>&1"
  sleep 5
  dexec "NVB=\$(python3 -c 'from ament_index_python.packages import get_package_share_directory as g; print(g(\"nvblox_examples_bringup\")+\"/config/nvblox/nvblox_base.yaml\")');
         exec ros2 run nvblox_ros nvblox_node --ros-args --params-file \$NVB --params-file $NAV/config/nvblox.yaml \
            -p use_color:=false -r camera_0/depth/image:=/camera/camera0/depth/image_rect_raw \
            -r camera_0/depth/camera_info:=/camera/camera0/depth/camera_info > /tmp/nvblox.log 2>&1"
  echo "fresh map/pose: cuVSLAM + nvblox restarted (camera untouched)"
  ;;
rviz)
  DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority xhost +local:root >/dev/null 2>&1 || true
  dexec "export DISPLAY=:1 XAUTHORITY=/root/.Xauthority; exec rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz > /tmp/rviz.log 2>&1"
  echo "RViz nav2 view on the Jetson monitor (:1)"
  ;;
status)
  # One-shot health of every layer. Empty/0 values flag what's not up yet.
  if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
    echo "container '$NAME' not running — start with ./run_stack.sh up"; exit 1
  fi
  rexec '
    campub=$(ros2 topic info '"$CAM_NS"'/infra1/image_rect_raw 2>/dev/null | awk "/Publisher count/{print \$3}")
    slam=$(ros2 topic echo /slam/status --once 2>/dev/null | grep -o "\"slam_pose_ok\": [a-z]*" | head -1)
    safe=$(ros2 topic echo /safety/state --once 2>/dev/null | grep -o "data: .*" | head -1)
    esp=$(ros2 topic info /cmd_vel 2>/dev/null | awk "/Subscription count/{print \$3}")
    # LIVE nav2 check: bt_navigator lifecycle state (not a stale /tmp/nav2.log grep,
    # which stays "Managed nodes are active" even after nav2 has crashed).
    nav=$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -o "^active")
    printf "  camera D555 (infra1 publisher): %s\n" "${campub:-0}  (>=1 = streaming)"
    printf "  cuVSLAM        %s\n" "${slam:-<no /slam/status>}"
    printf "  safety         %s\n" "${safe:-<no /safety/state>}"
    printf "  ESP32 wheels (/cmd_vel subs):   %s\n" "${esp:-0}  (1 = wheels linked)"
    printf "  nav2           %s\n" "${nav:-<not active / not started>}"
  '
  ;;
logs) docker exec "$NAME" tail -n 40 "/tmp/${2:-cuvslam}.log";;
down) docker rm -f "$NAME" >/dev/null 2>&1 && echo "removed $NAME (hard stop)";;
*) echo "usage: $0 {up|cam|nav2|vision|fuse|status|stop|remap|rviz|logs <name>|down}";;
esac
