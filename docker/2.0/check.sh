#!/usr/bin/env bash
# check.sh -- static checks of an orin-nav 2.x image on this Jetson. No camera, no lidar,
# no ROS traffic outside the container (ROS_LOCALHOST_ONLY-style: domain 99, loopback only).
#   ~/rover/docker/2.0/check.sh [image]        (default orin-nav:2.0)
# Every line is PASS / FAIL; exit code = number of FAILs.
set -uo pipefail
IMAGE=${1:-orin-nav:2.0}
echo "== $IMAGE  ($(docker image inspect -f '{{.Id}}' "$IMAGE" 2>/dev/null | cut -c8-19))"

docker run --rm --runtime nvidia --network none --entrypoint bash \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_DISABLE_REQUIRE=1 \
  -e ROS_DOMAIN_ID=99 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST "$IMAGE" -c '
fails=0
ok(){ echo "PASS  $1"; }
no(){ echo "FAIL  $1"; fails=$((fails+1)); }
for f in /opt/ros/*/setup.bash /opt/realsense_ws/install/setup.bash; do [ -f $f ] && source $f; done
echo "ROS_DISTRO=$ROS_DISTRO"

# 1. packages the rover launches
for p in isaac_ros_cuvslam isaac_ros_visual_slam_interfaces nvblox_ros nvblox_msgs \
         nav2_bt_navigator nav2_controller nav2_planner nav2_behaviors nav2_lifecycle_manager nav2_mppi_controller nav2_smac_planner nav2_collision_monitor \
         nav2_velocity_smoother nav2_map_server slam_toolbox realsense2_camera \
         robot_state_publisher cv_bridge image_transport rosbag2_py isaac_ros_gpu_partitioning; do
  ros2 pkg prefix $p >/dev/null 2>&1 && ok "pkg $p" || no "pkg $p"
done

# 2. composable plugins the launch files load
types=$(ros2 component types 2>/dev/null)
for c in nvidia::isaac_ros::visual_slam::VisualSlamNode realsense2_camera::RealSenseNodeFactory; do
  grep -q "$c" <<<"$types" && ok "component $c" || no "component $c"
done

# 3. every shared library resolves (no "not found")
for l in $(ls /opt/ros/*/lib/libisaac_ros_cuvslam_node.so /opt/ros/*/lib/libcuvslam.so \
              /opt/ros/*/lib/libnvblox_ros_lib.so /opt/realsense_ws/install/lib/librealsense2_camera.so \
              /opt/librealsense/lib/librealsense2.so 2>/dev/null); do
  m=$(ldd $l | grep "not found" | awk "{print \$1}" | tr "\n" " ")
  [ -z "$m" ] && ok "ldd $(basename $l)" || no "ldd $(basename $l): missing $m"
done

# 4. GPU code for Orin (sm_87) -- 4.6 nvblox had sm_75 only (4-minute JIT)
for l in /opt/ros/*/lib/libcuvslam.so /opt/ros/*/lib/libnvblox_ros_lib.so; do
  if command -v cuobjdump >/dev/null; then
    cuobjdump --list-elf $l 2>/dev/null | grep -q sm_87 && ok "sm_87 in $(basename $l)" || no "sm_87 in $(basename $l)"
  else
    strings $l | grep -q "sm_87" && ok "sm_87 string in $(basename $l)" || no "sm_87 in $(basename $l) (no cuobjdump)"
  fi
done

# 5. librealsense built with DDS (the D555 is a PoE/DDS camera)
if [ -x /opt/librealsense/bin/rs-enumerate-devices ]; then
  ok "librealsense $(/opt/librealsense/bin/rs-enumerate-devices --version 2>&1 | grep -oE "[0-9]+\.[0-9]+\.[0-9]+" | head -1)"
  ls /opt/librealsense/lib | grep -q realdds && ok "librealsense DDS (librealdds)" || no "librealsense DDS: no librealdds"
else no "rs-enumerate-devices"; fi

# 6. TensorRT in the image, and it sees the Orin GPU (1.2: 10.13 refused SM87)
python3 - <<PY && ok "TensorRT python + GPU builder" || no "TensorRT python + GPU builder"
import tensorrt as trt
lg = trt.Logger(trt.Logger.ERROR)
b = trt.Builder(lg)
assert b is not None and b.platform_has_fast_fp16, "no GPU builder"
print("      tensorrt", trt.__version__)
PY

# 7. python deps the rover nodes import
python3 -c "import numpy, scipy, cv2, lz4.frame, yaml, rclpy, cv_bridge" && ok "python deps" || no "python deps"

echo "== FAILS: $fails"; exit $fails
'
