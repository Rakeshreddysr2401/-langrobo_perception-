#!/usr/bin/env bash
# b2_gridloc.sh -- Part B2: NVIDIA's occupancy-grid localizer vs our relocalize.py, on the SAME
# live /scan and the SAME saved map. Passive: NVIDIA's nodes live under /b2 and publish only
# /b2/localization_result; nothing is fed back to the rover, nothing moves.
#   ~/rover/docker/2.0/b2_gridloc.sh [map name, default last] [runs, default 3]
#   TWIN=1 ...   the same on the twin (domain 42, container rover_twin, ~/.rover_twin/maps), and
#                both answers are scored against the twin's TRUE pose (/sim/truth/pose; its frame
#                `odom` = the world; a SLAM_FRESH map frame starts on it).
# Needs: the rover container up with ./rover lidar (and robot_state_publisher's TF).
set -uo pipefail
MAP=${1:-last}; RUNS=${2:-3}
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TWIN=${TWIN:-0}
if [ "$TWIN" = 1 ]; then DOMAIN=42; CT=rover_twin; MAPS=$HOME/.rover_twin/maps; else DOMAIN=0; CT=rover; MAPS=$REPO/maps; fi
IMAGE=${IMAGE:-orin-nav:2.1}
B2_PARAMS=${B2_PARAMS:-}   # extra localizer params, python dict body, e.g. "'invalid_range_threshold': 0.25"
[ -f "$MAPS/$MAP.yaml" ] || { echo "no $MAPS/$MAP.yaml"; exit 1; }
ENVSET="unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE; export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; export ROS_DOMAIN_ID=$DOMAIN; for f in /opt/ros/*/setup.bash; do source \$f; done;"

echo "== B2 on map $MAP ($RUNS runs each), domain $DOMAIN"
truth(){ # $1 = label
  echo "-- truth $1 (/sim/truth/pose, after 3 s standing still):"
  docker exec $CT bash -lc "$ENVSET python3 - <<'PY'
import math, time, rclpy
from geometry_msgs.msg import PoseStamped
rclpy.init(); n = rclpy.create_node('b2_truth'); got = []
n.create_subscription(PoseStamped, '/sim/truth/pose', got.append, 10)
t0 = time.time()
# wait until the rover has stood still for 3 s (a moving rover makes every comparison void)
still = None; last = None
while time.time() - t0 < 180:
    rclpy.spin_once(n, timeout_sec=0.1)
    if not got: continue
    c = (got[-1].pose.position.x, got[-1].pose.position.y, got[-1].pose.orientation.z)
    if last and max(abs(a - b) for a, b in zip(c, last)) < 0.002:
        still = still or time.time()
        if time.time() - still > 3: break
    else:
        still = None
    last = c
p = got[-1].pose; q = p.orientation
print('   truth: x=%.3f y=%.3f yaw=%.1f deg' % (p.position.x, p.position.y, math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))))
PY" 2>&1 | grep -v XMLPARSER
}
[ "$TWIN" = 1 ] && truth before
echo "-- ours: phase2/tools/relocalize.py"
for i in $(seq "$RUNS"); do
  t0=$(date +%s.%N)
  out=$(docker exec $CT bash -lc "$ENVSET python3 /opt/rover2/tools/relocalize.py /maps/$MAP.yaml 2>/dev/null" | grep '^{' | tail -1)
  echo "   run $i: $(python3 -c 'import json,sys,math; d=json.loads(sys.argv[1]); p=d.get("pose"); print(d["verdict"], ("x=%.3f y=%.3f yaw=%.1f deg" % (p[0], p[1], math.degrees(p[2]))) if p else "", "--", d.get("why",""))' "$out" 2>/dev/null || echo "$out")  ($(python3 -c "print(round($(date +%s.%N)-$t0,1))") s)"
done

echo "-- NVIDIA: isaac_ros_occupancy_grid_localizer ($IMAGE) ${B2_PARAMS:+extra params: $B2_PARAMS}"
docker rm -f b2_gridloc >/dev/null 2>&1
docker run -d --name b2_gridloc --runtime nvidia --network host --entrypoint bash \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_DISABLE_REQUIRE=1 \
  -v "$MAPS":/maps:ro "$IMAGE" -c "$ENVSET cat > /tmp/b2.launch.py <<'PY'
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
def generate_launch_description():
    return LaunchDescription([ComposableNodeContainer(
        name='b2_container', namespace='b2', package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(package='isaac_ros_pointcloud_utils',
                plugin='nvidia::isaac_ros::pointcloud_utils::LaserScantoFlatScanNode',
                name='scan_to_flatscan', namespace='b2', remappings=[('scan', '/scan'), ('flatscan', '/b2/flatscan')]),
            ComposableNode(package='isaac_ros_occupancy_grid_localizer',
                plugin='nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode',
                name='occupancy_grid_localizer', namespace='b2',
                parameters=['/maps/$MAP.yaml', {'loc_result_frame': 'map', 'map_yaml_path': '/maps/$MAP.yaml', $B2_PARAMS}],
                remappings=[('flatscan', '/b2/flatscan'), ('flatscan_localization', '/b2/flatscan_localization'),
                            ('localization_result', '/b2/localization_result')]),
        ], output='screen')])
PY
exec ros2 launch /tmp/b2.launch.py" >/dev/null
sleep 15
docker exec b2_gridloc bash -lc "$ENVSET ros2 service list 2>/dev/null | grep -i trigger" | sed 's/^/   service: /'
for i in $(seq "$RUNS"); do
  docker exec b2_gridloc bash -lc "$ENVSET python3 - <<'PY'
import math, time, rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
rclpy.init(); n = rclpy.create_node('b2_probe'); got = []
n.create_subscription(PoseWithCovarianceStamped, '/b2/localization_result', got.append, 10)
srv = [s for s, _ in n.get_service_names_and_types() if s.endswith('trigger_grid_search_localization')]
t_end = time.time() + 5
while not srv and time.time() < t_end:
    rclpy.spin_once(n, timeout_sec=0.2); srv = [s for s, _ in n.get_service_names_and_types() if s.endswith('trigger_grid_search_localization')]
if not srv: print('   no trigger service'); raise SystemExit(1)
c = n.create_client(Empty, srv[0]); c.wait_for_service(timeout_sec=5)
t0 = time.time(); c.call_async(Empty.Request())
while not got and time.time() - t0 < 30: rclpy.spin_once(n, timeout_sec=0.1)
if not got: print('   run $i: no result in 30 s'); raise SystemExit(1)
p = got[0].pose.pose; q = p.orientation
yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
cov = got[0].pose.covariance
print(f'   run $i: x={p.position.x:.3f} y={p.position.y:.3f} yaw={yaw:.1f} deg  frame {got[0].header.frame_id}  sd_xy {math.sqrt(max(cov[0], 0)):.3f} m  ({time.time() - t0:.1f} s)')
PY" 2>&1 | grep -v XMLPARSER
done
echo "   (its errors/warnings:)"; docker logs b2_gridloc 2>&1 | grep -E "ERROR|WARN" | grep -v "deprecated" | sed -E 's/\x1b\[[0-9;]*m//g' | cut -c1-200 | sort | uniq -c | head -6
docker rm -f b2_gridloc >/dev/null
[ "$TWIN" = 1 ] && truth after
exit 0
