#!/usr/bin/env bash
# vo_replay.sh -- replay a recorded IR bag into an image's cuVSLAM (+ vo_isaac.py), offline:
# --network none, ROS domain 98. Same input for every image = a fair A/B.
#   ~/rover/docker/2.0/vo_replay.sh <image> [bag dir under ~/rover/logs] [still|moving]
# For a bag recorded STANDING STILL, every cm of motion reported is error.
set -uo pipefail
IMAGE=${1:?image}; BAG=${2:-ab_ir}; KIND=${3:-still}
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
echo "== vo_replay $IMAGE  bag=$BAG ($KIND)"
docker run --rm --runtime nvidia --network none --entrypoint bash \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_DISABLE_REQUIRE=1 \
  -v "$HOME/.cache/rover-cuda/vo_check":/root/.nv/ComputeCache -e CUDA_CACHE_MAXSIZE=4294967296 \
  -e ROS_DOMAIN_ID=98 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -v "$REPO/phase1":/opt/rover:ro -v "$REPO/logs/$BAG":/bag:ro "$IMAGE" -c '
for f in /opt/ros/*/setup.bash; do source $f; done
echo "ROS_DISTRO=$ROS_DISTRO"
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id camera0_link >/dev/null 2>&1 &
ros2 launch /opt/rover/launch/vo_isaac.launch.py > /tmp/vslam.log 2>&1 &
python3 /opt/rover/nodes/vo_isaac.py --ros-args -p slam:=false > /tmp/vo.log 2>&1 &
sleep 12
python3 - <<PY &
import time, json, math, rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n = rclpy.create_node("replay_probe")
P = []; S = {}; H = []
n.create_subscription(Odometry, "/vo/odom", lambda m: P.append((m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)), qos_profile_sensor_data)
def st(m):
    S.update(json.loads(m.data)); H.append(S.get("healthy"))
n.create_subscription(String, "/vo/status", st, 1)
t0 = time.time()
while time.time() - t0 < 26: rclpy.spin_once(n, timeout_sec=0.1)
if not P: print("FAIL  no /vo/odom"); raise SystemExit(1)
q = P[0]
d = [math.dist(q, p) for p in P]
print(f"odom msgs {len(P)}; max excursion {max(d)*100:.1f} cm; end {d[-1]*100:.1f} cm; end z {P[-1][2]*100:+.1f} cm")
rs, lm = S.get("tracker_resets"), S.get("landmarks")
print(f"healthy {sum(1 for h in H if h)}/{len(H)} reports; resets {rs}; landmarks {lm}")
if "$KIND" == "still":
    print(("PASS" if max(d) < 0.05 else "FAIL") + "  standing still: max excursion < 5 cm")
PY
probe=$!
ros2 bag play /bag > /tmp/play.log 2>&1
wait $probe; rc=$?
echo "--- vslam: $(grep -c "cuvslam: Reset" /tmp/vslam.log) resets, $(grep -c "PoseGraph optimization failed" /tmp/vslam.log) PoseGraph failures, $(grep -c "above threshold" /tmp/vslam.log) frame-gap warnings"
exit $rc
' 2>&1 | grep -v XMLPARSER
