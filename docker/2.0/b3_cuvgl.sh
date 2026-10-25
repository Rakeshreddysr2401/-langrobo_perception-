#!/usr/bin/env bash
# b3_cuvgl.sh -- Part B3: NVIDIA's cuVGL (camera-only "where am I?") on the TWIN, scored on the truth.
#   ~/rover/docker/2.0/b3_cuvgl.sh <cuvgl_map dir> [seconds, default 200]
# Runs visual_global_localization (orin-nav:2.1, domain 42) on the live IR pair while the twin
# drives (start ./sim scenario on the laptop right after this). Logs every answer with the
# truth at the same moment, then fits ONE rigid 2D transform map -> world (the map's frame is the
# recording's first camera pose, not the world) and reports the residual per answer: that is
# cuVGL's accuracy. Answers off by > 0.5 m after the fit count as wrong.
# The map: phase2/tools/record_cuvgl.sh + create_map_offline.py (ISAAC_ROS_5_UPGRADE.md B3);
# cuVGL features must be SIFT on this Orin (TensorRT cannot build ALIKED for SM 87, 2026-10-25).
set -uo pipefail
MAP=${1:?cuvgl_map dir}; SECS=${2:-200}
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT=$(mktemp -d)
docker rm -f b3_cuvgl >/dev/null 2>&1
docker run --rm --name b3_cuvgl --runtime nvidia --network host --entrypoint bash \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_DISABLE_REQUIRE=1 \
  -e ROS_DOMAIN_ID=42 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -v "$MAP":/map:ro -v "$REPO/phase2/config":/cfg:ro -v "$REPO/phase2/nodes":/nodes:ro -v "$OUT":/out "${IMAGE:-orin-nav:2.1}" -c "
source /opt/ros/lyrical/setup.bash
python3 /nodes/cuvgl_relay.py > /out/relay.log 2>&1 &
ros2 launch isaac_ros_visual_global_localization visual_global_localization.launch.py \
  container_name:=vgl_container vgl_enabled_stereo_cameras:=front_stereo_camera vgl_map_dir:=/map topic_config_file:=/cfg/cuvgl_live.yaml \
  vgl_do_rectify_images:=False vgl_camera_optical_frames:=vslam_left_optical,vslam_right_optical \
  vgl_enable_continuous_localization:=True vgl_frequency:=1.0 > /out/vgl.log 2>&1 &
ros2 run rclcpp_components component_container --ros-args -r __node:=vgl_container > /out/container.log 2>&1 &
python3 - <<'PY'
import json, time, rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
rclpy.init(); n = rclpy.create_node('b3_probe')
truth = []; ans = []
n.create_subscription(PoseStamped, '/sim/truth/pose', lambda m: truth.append((time.time(), m.pose)), 10)
def on(m):
    p = getattr(m.pose, 'pose', m.pose)
    ans.append((time.time(), p))
topics = []
t0 = time.time()
while time.time() - t0 < $SECS:
    rclpy.spin_once(n, timeout_sec=0.05)
    if not topics and time.time() - t0 > 15:
        for name, types in n.get_topic_names_and_types():
            if name.startswith('/visual_localization/pose') or name == '/visual_localization/pose':
                T = PoseWithCovarianceStamped if 'WithCovariance' in types[0] else PoseStamped
                n.create_subscription(T, name, on, 10); topics.append(name)
import math
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
rows = []
for ta, p in ans:
    tt = min(truth, key=lambda r: abs(r[0] - ta)) if truth else None
    if tt and abs(tt[0] - ta) < 0.2:
        rows.append([p.position.x, p.position.y, yaw(p.orientation), tt[1].position.x, tt[1].position.y, yaw(tt[1].orientation)])
json.dump(dict(topics=topics, rows=rows, n_truth=len(truth)), open('/out/b3.json', 'w'))
print(f'answers {len(ans)} on {topics}, matched with truth {len(rows)}')
PY
" 2>&1 | grep -v XMLPARSER | tail -3
python3 - "$OUT/b3.json" <<'PY'
import json, math, sys
import numpy as np
d = json.load(open(sys.argv[1])); R = np.array(d['rows'])
if len(R) < 3:
    print('FAIL  too few answers to score:', len(R)); sys.exit(1)
A, B = R[:, 0:2], R[:, 3:5]
# robust rigid fit: Umeyama on all, then refit on the 70 % best
def fit(a, b):
    ma, mb = a.mean(0), b.mean(0); U, _, Vt = np.linalg.svd((a - ma).T @ (b - mb))
    Rm = (U @ Vt).T
    if np.linalg.det(Rm) < 0: Vt[-1] *= -1; Rm = (U @ Vt).T
    return Rm, mb - Rm @ ma
Rm, t = fit(A, B)
for _ in range(2):
    e = np.linalg.norm((A @ Rm.T + t) - B, axis=1); keep = e <= np.percentile(e, 70)
    Rm, t = fit(A[keep], B[keep])
e = np.linalg.norm((A @ Rm.T + t) - B, axis=1)
rot = math.atan2(Rm[1, 0], Rm[0, 0])
dyaw = np.degrees(np.abs((R[:, 2] + rot - R[:, 5] + np.pi) % (2 * np.pi) - np.pi))
ok = e < 0.5
print(f'{len(R)} answers; map->world fit: rot {math.degrees(rot):.1f} deg, t ({t[0]:.2f}, {t[1]:.2f})')
print(f'right (< 0.5 m): {ok.sum()}/{len(R)} = {100 * ok.mean():.0f} %;  of those: position median {100 * np.median(e[ok]):.1f} cm, '
      f'p90 {100 * np.percentile(e[ok], 90):.1f} cm; heading median {np.median(dyaw[ok]):.1f} deg')
PY
echo "(logs: $OUT)"
