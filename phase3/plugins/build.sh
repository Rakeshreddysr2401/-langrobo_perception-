#!/usr/bin/env bash
# Build the rover's nav2 costmap plugins (footprint_clear) for the rover
# container. Same recipe as lidar/build.sh: compiled INSIDE the image, never
# installed into it, with the workspace bind-mounted from the host at the path
# it runs from (/opt/rover3/plugins -- phase3/ is mounted at /opt/rover3), so
# the output survives ./rover stop and needs no container restart.
# nav2_supervise.sh sources ws/install/setup.bash when it exists.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE=${IMAGE:-orin-nav:1.1}
echo "[plugins] colcon build in $IMAGE"
docker run --rm --user "$(id -u):$(id -g)" --entrypoint bash \
  -v "$HERE/..":/opt/rover3 "$IMAGE" -c \
  'source /opt/ros/jazzy/setup.bash && cd /opt/rover3/plugins/ws && \
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -15'
# install/ holds symlinks to /opt/rover3/... (container paths): check build/
[ -f "$HERE/ws/build/rover_costmap_plugins/librover_costmap_plugins.so" ] \
  && echo "[plugins] OK -- now: ./rover nav" \
  || { echo "[plugins] build failed"; exit 1; }
