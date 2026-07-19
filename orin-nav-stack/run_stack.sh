#!/bin/bash
# Orin nav stack — cuVSLAM + nvblox + nav2 + vision, ONE self-contained container.
# Everything (nodes, configs, YOLO weights, behavior tree) is baked into the image
# at /opt/orin-nav. No workspace mounts.
#
#   ./run_stack.sh up       # camera + base TF + cuVSLAM (FULL SLAM: map->odom live,
#                           #   /slam/save_map + /slam/localize, maps persist in ./maps) + nvblox
#   ./run_stack.sh nav2     # nav2 (NO blind recoveries BT; map frame from cuVSLAM)
#   ./run_stack.sh vision   # YOLO + pixel_to_goal + motor shim + SAFETY GUARD + imu_to_base
#                           #   chain: /cmd_vel_nav -> deadband -> /cmd_vel_shim -> guard -> /cmd_vel
#   ./run_stack.sh stop     # E-STOP: kill nav/motion nodes + zero /cmd_vel
#   ./run_stack.sh remap    # fresh map/pose: restart cuVSLAM + nvblox (camera untouched)
#   ./run_stack.sh rviz     # RViz nav2 view on the Jetson monitor (:1)
#   ./run_stack.sh logs <realsense|cuvslam|nvblox|nav2|detections_3d|...>
#   ./run_stack.sh down     # hard stop: remove the container (ultimate e-stop)
#
# SAFETY (tethered rover): BT has NO Spin/BackUp; deadband caps vx<=0.22 wz<=0.90.
# Rover motion tests: pause YOLO first (CPU overload -> cuVSLAM pose jumps).
set -eo pipefail
IMAGE=${IMAGE:-orin-nav:1.1}
NAME=${NAME:-orin_nav}
NAV=/opt/orin-nav
# cuVSLAM wheel dir FIRST (its libcuvslam.so must beat Isaac ROS's Thor build at
# /opt/ros/jazzy/lib), then the CUDA-12 user-space libs.
CU12='/usr/local/lib/python3.12/dist-packages/cuvslam:/usr/local/lib/python3.12/dist-packages/nvidia/cuda_runtime/lib:/usr/local/lib/python3.12/dist-packages/nvidia/cublas/lib:/usr/local/lib/python3.12/dist-packages/nvidia/cusolver/lib:/usr/local/lib/python3.12/dist-packages/nvidia/cusparse/lib:/usr/local/lib/python3.12/dist-packages/nvidia/nvjitlink/lib:/usr/local/lib/python3.12/dist-packages/nvidia/nvtx/lib'

dexec(){ docker exec -d "$NAME" bash -lc "unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE; export ROS_DOMAIN_ID=0; source /opt/ros/jazzy/setup.bash; $1"; }

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
  echo "[1/4] D555 (stereo IR + depth + color + motion, emitter ON)"
  dexec 'printf "{\"context\":{\"dds\":{\"enabled\":true,\"domain\":0}}}" > ~/.realsense-config.json;
         export LD_LIBRARY_PATH=/root/librealsense/install/lib:$LD_LIBRARY_PATH;
         exec ros2 launch realsense2_camera rs_launch.py camera_name:=camera0 \
            enable_infra1:=true enable_infra2:=true depth_module.infra_profile:=896x504x30 \
            depth_module.emitter_enabled:=1 enable_depth:=true enable_color:=true enable_motion:=true enable_sync:=true \
            > /tmp/realsense.log 2>&1'
  sleep 12
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
  dexec "exec python3 $NAV/nodes/imu_to_base.py > /tmp/imu_to_base.log 2>&1"
  echo "vision + goal + motor shim (vx<=0.22 wz<=0.90) + safety guard starting"
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
  docker exec "$NAME" bash -lc 'for pid in $(pgrep -f "[c]uvslam_ros_node.py"); do kill -9 $pid 2>/dev/null; done; for pid in $(pgrep -x nvblox_node); do kill -9 $pid 2>/dev/null; done; true' || true
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
logs) docker exec "$NAME" tail -n 40 "/tmp/${2:-cuvslam}.log";;
down) docker rm -f "$NAME" >/dev/null 2>&1 && echo "removed $NAME (hard stop)";;
*) echo "usage: $0 {up|nav2|vision|stop|remap|rviz|logs <name>|down}";;
esac
