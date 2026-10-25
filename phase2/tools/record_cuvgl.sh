#!/usr/bin/env bash
# record_cuvgl.sh -- record one drive for an Isaac ROS 5.0 visual map (cuVGL + cuVSLAM +
# occupancy), plan B3. Run with ./rover pose up (camera + vo_isaac for the vslam_* frames) and
# drive the room slowly by teleop, IN LIGHT (emitter is off: a dark room gives black IR).
#   ~/rover/phase2/tools/record_cuvgl.sh <name> [seconds, default 300]
# -> ~/rover/logs/bags/<date>_cuvgl_<name>/ (~27 MB/s of IR: 5 min ~ 8 GB)
set -euo pipefail
NAME=${1:?name}; SECS=${2:-300}
OUT=$(date +%Y%m%d-%H%M%S)_cuvgl_$NAME
FREE=$(df --output=avail -B1G / | tail -1 | tr -d ' ')
NEED=$(( SECS * 27 / 1000 + 5 ))
[ "$FREE" -ge "$NEED" ] || { echo "only ${FREE} GB free, need ~${NEED} GB for ${SECS} s"; exit 1; }
echo "[cuvgl] recording $SECS s -> logs/bags/$OUT   (Ctrl-C stops early, bag is kept)"
docker exec rover bash -lc "unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE; export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; \
  export ROS_DOMAIN_ID=0; source /opt/ros/lyrical/setup.bash; mkdir -p /logs/bags; cd /logs/bags; \
  timeout -s INT $SECS ros2 bag record -s mcap -o $OUT --topics \
    /camera/camera0/infra1/image_rect_raw /camera/camera0/infra2/image_rect_raw \
    /camera/camera0/infra1/camera_info /camera/camera0/infra2/camera_info \
    /tf /tf_static /odom /scan /vo/odom" || true
docker exec rover bash -lc "source /opt/ros/lyrical/setup.bash; ros2 bag info /logs/bags/$OUT | grep -E 'Duration|Count:' | head -12; chown -R $(id -u):$(id -g) /logs/bags/$OUT"
