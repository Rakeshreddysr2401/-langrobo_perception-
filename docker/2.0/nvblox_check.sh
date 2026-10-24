#!/usr/bin/env bash
# nvblox_check.sh -- the rover's nvblox (phase2/launch/nvblox.launch.py) on a synthetic depth
# image: a flat wall 1.5 m in front of the camera. Offline: --network none, ROS domain 99.
#   ~/rover/docker/2.0/nvblox_check.sh [image]     (default orin-nav:2.0)
# Reports: seconds from start to the first map slice (4.6's sm_75-only nvblox JIT-compiled
# for ~4 min here), slice rate, nvblox CPU/RAM, ERROR/WARN lines (renamed parameters show here).
# A fresh CUDA cache every run, so the JIT cost (if any) is measured, not hidden.
set -uo pipefail
IMAGE=${1:-orin-nav:2.0}
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
echo "== nvblox_check $IMAGE"
docker run --rm --runtime nvidia --network none --entrypoint bash \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_DISABLE_REQUIRE=1 \
  -v /opt/nvidia/l4t-gpu-libs/openrm/libnvcuvid.so:/usr/lib/aarch64-linux-gnu/nvidia/libnvcuvid.so.1:ro \
  -e ROS_DOMAIN_ID=99 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -v "$REPO/phase2":/opt/rover2:ro "$IMAGE" -c '
for f in /opt/ros/*/setup.bash; do source $f; done
echo "ROS_DISTRO=$ROS_DISTRO"
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link >/dev/null 2>&1 &
# camera optical frame: z forward, x right, y down (base_link: x forward, z up)
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id camera0_depth_optical_frame \
  --z 0.2 --roll -1.5708 --yaw -1.5708 >/dev/null 2>&1 &
python3 - <<PY > /tmp/fake_depth.log 2>&1 &
import numpy as np, rclpy, time
from sensor_msgs.msg import Image, CameraInfo
rclpy.init(); n = rclpy.create_node("fake_depth")
W, H, F = 448, 252, 215.0
pi = n.create_publisher(Image, "/camera/camera0/depth/image_still", 5)
pc = n.create_publisher(CameraInfo, "/camera/camera0/depth/camera_info", 5)
d = np.full((H, W), 1500, np.uint16)             # 1.5 m wall, millimetres (16UC1, as the D555)
def tick():
    st = n.get_clock().now().to_msg()
    m = Image(); m.header.stamp = st; m.header.frame_id = "camera0_depth_optical_frame"
    m.height, m.width, m.encoding, m.step = H, W, "16UC1", W * 2; m.data = d.tobytes(); pi.publish(m)
    c = CameraInfo(); c.header = m.header; c.width, c.height = W, H; c.distortion_model = "plumb_bob"
    c.d = [0.0] * 5; c.k = [F, 0., W / 2, 0., F, H / 2, 0., 0., 1.]; c.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    c.p = [F, 0., W / 2, 0., 0., F, H / 2, 0., 0., 0., 1., 0.]; pc.publish(c)
n.create_timer(1 / 15, tick); rclpy.spin(n)
PY
t0=$(date +%s.%N)
ros2 launch /opt/rover2/launch/nvblox.launch.py > /tmp/nvblox.log 2>&1 &
python3 - <<PY
import time, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
rclpy.init(); n = rclpy.create_node("probe")
topics = None; first = None; count = 0; t0 = float("$t0")
from nvblox_msgs.msg import DistanceMapSlice
def cb(m):
    global first, count
    count += 1
    if first is None: first = time.time()
q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
subs = [n.create_subscription(DistanceMapSlice, t, cb, q) for t in ("/nvblox_node/static_map_slice", "/nvblox_node/map_slice")]
end = time.time() + 300
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
    if first and time.time() - first > 10: break
if first is None:
    print("FAIL  no map slice in 300 s; topics:", [t for t, _ in n.get_topic_names_and_types() if "nvblox" in t][:12])
else:
    print(f"first slice {first - t0:.1f} s after launch; {count / (time.time() - first):.1f} Hz over the next 10 s")
    print(("PASS" if first - t0 < 60 else "FAIL") + "  nvblox up in under 60 s (4.6 JIT: ~240 s)")
PY
echo "cpu/ram:"; ps -eo pcpu,rss,args | grep "[l]ib/nvblox_ros/nvblox_node" | awk "{printf \"  %5s%%  %6d MB  nvblox_node\n\", \$1, \$2/1024}" | head -1
clean(){ sed -E "s/\x1b\[[0-9;]*m//g" | cut -c1-220; }
echo "--- ERROR/FATAL:"; grep -E "\[ERROR\]|\[FATAL\]" /tmp/nvblox.log | clean | sort | uniq -c | head -8
echo "--- WARN (unique):"; grep -E "\[WARN\]" /tmp/nvblox.log | clean | sed -E "s/\[[0-9]+\.[0-9]+\]//" | sort | uniq -c | sort -rn | head -10
' 2>&1 | grep -v XMLPARSER
