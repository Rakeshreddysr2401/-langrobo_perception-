#!/usr/bin/env bash
# Build the RPLidar C1 driver (sllidar_ros2) for the rover container. One-off.
#
# The image has no build recipe and must never be modified, so the driver is
# compiled INSIDE the image but with the workspace bind-mounted from the host:
# the binaries land in lidar/ws/install on the host and survive every
# `./rover stop`. It is mounted at /opt/lidar at runtime — the same path it is
# built at, because colcon's --symlink-install writes absolute paths.
#
# Runs as the calling user, not root, so the output is deletable without sudo.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE=${IMAGE:-orin-nav:1.1}
SLLIDAR_REPO=https://github.com/Slamtec/sllidar_ros2.git
SLLIDAR_REV=34300099fadfc772965962dec837bf436706188f   # verified on the C1, 2026-09-22

mkdir -p "$HERE/ws/src"
if [ ! -d "$HERE/ws/src/sllidar_ros2/.git" ]; then
  echo "[lidar] fetching sllidar_ros2 @ ${SLLIDAR_REV:0:8}"
  git clone -q "$SLLIDAR_REPO" "$HERE/ws/src/sllidar_ros2"
  git -C "$HERE/ws/src/sllidar_ros2" checkout -q "$SLLIDAR_REV"
fi

echo "[lidar] colcon build in $IMAGE (≈30 s)"
docker run --rm --user "$(id -u):$(id -g)" --entrypoint bash \
  -v "$HERE":/opt/lidar "$IMAGE" -c \
  'source /opt/ros/jazzy/setup.bash && cd /opt/lidar/ws && \
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3'

[ -x "$HERE/ws/build/sllidar_ros2/sllidar_node" ] \
  && echo "[lidar] OK — now: ./rover lidar" \
  || { echo "[lidar] build failed"; exit 1; }
