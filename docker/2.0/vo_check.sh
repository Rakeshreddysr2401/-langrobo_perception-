#!/usr/bin/env bash
# vo_check.sh -- cuVSLAM on the synthetic IR pair (phase1/harness/fake_stereo.py), inside an image.
# No camera, no robot: --network none, ROS domain 99.
#   ~/rover/docker/2.0/vo_check.sh [image]      (default orin-nav:2.0; orin-nav:1.2 for the 4.6 baseline)
# Prints the tracked motion vs the truth (0.10 m/s sideways for 20 s = 2.00 m), the VO rate,
# vo_isaac's health, and the tracker's CPU.
set -uo pipefail
IMAGE=${1:-orin-nav:2.0}
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
echo "== vo_check $IMAGE"
docker run --rm --runtime nvidia --network none --entrypoint bash \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_DISABLE_REQUIRE=1 \
  -v /opt/nvidia/l4t-gpu-libs/openrm/libnvcuvid.so:/usr/lib/aarch64-linux-gnu/nvidia/libnvcuvid.so.1:ro \
  -v "$HOME/.cache/rover-cuda/vo_check":/root/.nv/ComputeCache -e CUDA_CACHE_MAXSIZE=4294967296 \
  -e ROS_DOMAIN_ID=99 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -v "$REPO/phase1":/opt/rover:ro "$IMAGE" -c '
for f in /opt/ros/*/setup.bash /opt/realsense_ws/install/setup.bash; do [ -f $f ] && source $f; done
echo "ROS_DISTRO=$ROS_DISTRO"
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id camera0_link > /tmp/tf.log 2>&1 &
ros2 launch /opt/rover/launch/vo_isaac.launch.py > /tmp/vslam.log 2>&1 &
python3 /opt/rover/nodes/vo_isaac.py --ros-args -p slam:=false > /tmp/vo.log 2>&1 &
sleep 12                                   # GPU init
python3 - <<PY &
import json, time, rclpy, threading
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n = rclpy.create_node("vo_probe")
P = []; S = {}
n.create_subscription(Odometry, "/vo/odom", lambda m: P.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)), qos_profile_sensor_data)
R = []
def st(m):
    S.update(json.loads(m.data)); R.append(S.get("rate_hz", 0))
n.create_subscription(String, "/vo/status", st, 1)
t0 = time.time()
while time.time() - t0 < 26: rclpy.spin_once(n, timeout_sec=0.1)
if len(P) < 2:
    print("FAIL  no /vo/odom"); raise SystemExit(1)
import math
span = P[-1][0] - P[0][0]
d = [b - a for a, b in zip(P[0][1:], P[-1][1:])]
dist = math.sqrt(sum(x * x for x in d)); truth = 0.10 * span
rate = (len(P) - 1) / span if span > 0 else 0
err = dist - truth
print(f"odom msgs {len(P)} over {span:.1f} s = {rate:.1f} Hz; moved dx={d[0]:+.3f} dy={d[1]:+.3f} dz={d[2]:+.3f} m")
print(f"|d| = {dist:.3f} m, truth {truth:.3f} m (0.10 m/s x {span:.1f} s): error {err*100:+.1f} cm ({err/truth*100:+.1f} %)")
print("vo_isaac status (peak rate %.1f Hz):" % max(R or [0]), {k: S.get(k) for k in ("vo_state", "landmarks", "tracker_resets", "worst_frame_gap_ms", "raw_read_ok")})
print(("PASS" if abs(err) < 0.10 and rate >= 25 else "FAIL") + "  VO within 10 cm of truth and >= 25 Hz")
PY
probe=$!
python3 /opt/rover/harness/fake_stereo.py --seconds 22 > /tmp/fake.log 2>&1 &
sleep 10
echo "cpu (% of one core):"; ps -eo pcpu,rss,args | grep -E "component_container|vo_isaac|fake_stereo" | grep -v grep | awk "{printf \"  %5s%%  %6d MB  %s\n\", \$1, \$2/1024, \$3\" \"\$4}"
wait $probe; rc=$?
echo "--- vslam.log (errors)"; grep -iE "error|fail|exception" /tmp/vslam.log | head -5
echo "--- vo.log (tail)"; tail -3 /tmp/vo.log
exit $rc
'
