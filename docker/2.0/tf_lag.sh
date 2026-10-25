#!/usr/bin/env bash
# tf_lag.sh -- how often is a LiDAR scan NEWER than the newest odom -> base_link TF?
# Each such scan is one the collision monitor cannot use ("extrapolation into the future").
#   ~/rover/docker/2.0/tf_lag.sh [seconds, default 40]     TWIN=1 for the twin
set -uo pipefail
SECS=${1:-40}
if [ "${TWIN:-0}" = 1 ]; then D=42; CT=rover_twin; else D=0; CT=rover; fi
E="unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE; export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; export ROS_DOMAIN_ID=$D; for f in /opt/ros/*/setup.bash; do source \$f; done;"
docker exec -i "$CT" bash -lc "$E python3 - $SECS" <<'PY' 2>&1 | grep -v XMLPARSER
import sys, time, statistics as st, rclpy
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
rclpy.init(); n = rclpy.create_node('tf_lag'); tf = []; sc = []
def ontf(m):
    now = time.time()
    for t in m.transforms:
        if t.header.frame_id == 'odom' and t.child_frame_id == 'base_link':
            tf.append((now, t.header.stamp.sec + t.header.stamp.nanosec * 1e-9))
n.create_subscription(TFMessage, '/tf', ontf, 100)
n.create_subscription(LaserScan, '/scan', lambda m: sc.append((time.time(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)),
                      QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
t0 = time.time()
while time.time() - t0 < float(sys.argv[1]): rclpy.spin_once(n, timeout_sec=0.01)
age = [a - s for a, s in tf]; j = 0; ahead = []
gaps = sorted(b[0] - a[0] for a, b in zip(tf, tf[1:]))
for arr, stamp in sc:
    while j + 1 < len(tf) and tf[j + 1][0] <= arr: j += 1
    if tf and tf[j][0] <= arr: ahead.append(stamp - tf[j][1])
bad = [a for a in ahead if a > 0]
print(f'odom TF {len(tf) / float(sys.argv[1]):.1f} Hz, stamp age on arrival median {1000 * st.median(age):.0f} ms; '
      f'scans newer than the newest TF: {len(bad)}/{len(ahead)} = {100 * len(bad) / max(1, len(ahead)):.1f} %')
print(f'TF arrival gaps: median {1000 * st.median(gaps):.0f} ms, p99 {1000 * gaps[int(.99 * len(gaps))]:.0f} ms, max {1000 * gaps[-1]:.0f} ms; gaps > 200 ms (collision monitor transform_tolerance): {sum(g > 0.2 for g in gaps)}')
PY
