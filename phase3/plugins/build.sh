#!/usr/bin/env bash
# Build the rover's nav2 costmap plugins (footprint_clear) for the rover
# container. Same recipe as lidar/build.sh: compiled INSIDE the image, never
# installed into it, with the workspace bind-mounted from the host at the path
# it runs from (/opt/rover3/plugins -- phase3/ is mounted at /opt/rover3), so
# the output survives ./rover stop and needs no container restart.
# nav2_supervise.sh sources ws/install/setup.bash when it exists.
set -euo pipefail
# One build per ROS distro: Jazzy (orin-nav:1.x) keeps ws/build + ws/install, others get
# ws/build_<distro> + ws/install_<distro> (orin-nav:2.x: _lyrical), so 1.2 stays a rollback.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE=${IMAGE:-orin-nav:1.2}
echo "[plugins] colcon build in $IMAGE"
docker run --rm --user "$(id -u):$(id -g)" --entrypoint bash \
  -v "$HERE/..":/opt/rover3 "$IMAGE" -c \
  'for f in /opt/ros/*/setup.bash; do source $f; done && cd /opt/rover3/plugins/ws && \
   S=$([ "$ROS_DISTRO" = jazzy ] && echo "" || echo "_$ROS_DISTRO") && \
   colcon --log-base log$S build --symlink-install --build-base build$S --install-base install$S --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -15'
# install/ holds symlinks to /opt/rover3/... (container paths): check build/
SUF=$(docker run --rm --entrypoint bash "$IMAGE" -c 'for f in /opt/ros/*/setup.bash; do source $f; done; [ "$ROS_DISTRO" = jazzy ] || echo "_$ROS_DISTRO"')
[ -f "$HERE/ws/build$SUF/rover_costmap_plugins/librover_costmap_plugins.so" ] \
  && echo "[plugins] OK -- now: ./rover nav" \
  || { echo "[plugins] build failed"; exit 1; }
