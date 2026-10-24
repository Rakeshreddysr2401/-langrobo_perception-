#!/usr/bin/env bash
# nav2_check.sh -- bring up the rover's real nav2 (phase3/launch/nav2.launch.py + config/nav2.yaml
# + our costmap plugin + BT xml) in an image, offline: --network none, ROS domain 99, static TF,
# no sensors. Catches renamed params/plugins/BT nodes before any twin or robot run.
#   ~/rover/docker/2.0/nav2_check.sh [image]      (default orin-nav:2.0)
# PASS = lifecycle manager reports "Managed nodes are active" with 0 ERROR/FATAL lines.
set -uo pipefail
IMAGE=${1:-orin-nav:2.0}
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
echo "== nav2_check $IMAGE"
docker run --rm --runtime nvidia --network none --entrypoint bash -e ROS_DOMAIN_ID=99 \
  -v "$REPO/phase3":/opt/rover3:ro "$IMAGE" -c '
for f in /opt/ros/*/setup.bash; do source $f; done
P=/opt/rover3/plugins/ws/install_${ROS_DISTRO}/setup.bash
[ -f $P ] && source $P || echo "FAIL  no costmap plugin build for $ROS_DISTRO (phase3/plugins/build.sh)"
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link >/dev/null 2>&1 &
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id laser >/dev/null 2>&1 &
timeout 60 ros2 launch /opt/rover3/launch/nav2.launch.py > /tmp/nav.log 2>&1
clean(){ sed -E "s/\x1b\[[0-9;]*m//g" | cut -c1-230; }
echo "ROS_DISTRO=$ROS_DISTRO"
n=$(grep -cE "\[ERROR\]|\[FATAL\]" /tmp/nav.log)
grep -E "\[ERROR\]|\[FATAL\]" /tmp/nav.log | clean | sort | uniq -c | sort -rn | head -12
echo "--- warnings (unique):"; grep -E "\[WARN\]" /tmp/nav.log | clean | sed -E "s/\[[0-9]+\.[0-9]+\]//" | sort | uniq -c | sort -rn | head -10
if grep -q "Managed nodes are active" /tmp/nav.log && [ "$n" = 0 ]; then echo "PASS  nav2 active, 0 errors"; exit 0
else echo "FAIL  nav2: $n ERROR/FATAL lines, active=$(grep -c "Managed nodes are active" /tmp/nav.log)"; exit 1; fi
' 2>&1 | grep -v XMLPARSER
