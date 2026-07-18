#!/bin/bash
# RTAB-Map localization (rgbd_odometry -> /odom + odom->base_link TF, rtabmap
# -> map->odom TF + persistent /data/rtabmap.db). Run from the Jetson HOST.
#
# This replaces the cuVSLAM sidecar as the odometry/SLAM source (2026-07-16):
# cuVSLAM 12.6 explodes under real motion on this rig (frame gaps + no IMU +
# soft-surface whiplash; every extrinsics permutation A/B-tested — see
# ISSUES_AND_SOLUTIONS.md). RTAB-Map runs in the SAME Jazzy container as the
# camera driver (no cross-distro DDS), on color+aligned depth from the
# camera's internal depth engine (450+ features vs cuVSLAM's 0-4), with
# Reg/Force3DoF so poses stay planar by construction and
# Odom/ResetCountdown=1 for auto-recovery after tracking loss.
#
# Params mirror launch/perception.launch.py mode:=real (the 2026-07-15
# verified profile) — keep them in sync.
set -eo pipefail
DIR=$(cd "$(dirname "$0")" && pwd)

# Sim and real must NEVER share a SLAM memory: localizing the sim world
# against the real map (or vice versa) corrupts both. Same node, same
# params — only the database file follows config/hardware.
HW=$(tr -d '[:space:]' < "$DIR/../config/hardware" 2>/dev/null || echo real)
DB=/data/rtabmap.db
[ "$HW" = "sim" ] && DB=/data/rtabmap_sim.db

# NOTE: align_depth.enable is accepted but publishes NOTHING on the DDS
# driver (same trap as pointcloud.enable — SDK processing blocks don't exist
# for network cameras). Instead we feed infra1 as the "rgb" image: RealSense
# depth is computed in infra1's viewpoint, so infra1 + raw depth are
# perfectly registered by construction (same frame, same 896x504 grid) and
# rtabmap takes mono8 fine. Emitter must stay OFF (dots would corrupt the
# feature tracking on the IR image).

# rtabmap writes /data/rtabmap.db (SQLite) — SIGKILL mid-write corrupts it
# (malformed-DB crash loop, 2026-07-17). TERM, wait for the DB to close, -9.
docker exec isaac_ros bash -c '
    pkill -TERM rtabmap 2>/dev/null
    for i in $(seq 1 15); do pgrep rtabmap >/dev/null || break; sleep 1; done
    pkill -9 rtabmap 2>/dev/null
    pkill -9 -f "[r]gbd_odometry" 2>/dev/null
    pkill -9 -f "[e]kf_node" 2>/dev/null
    true'
sleep 2

# A corrupt DB makes rtabmap abort at start (FATAL, no map frame, vision AI
# silently dead). Verify it and archive a bad one instead of crash-looping.
docker exec -e DB="$DB" isaac_ros bash -c '
    [ -f "$DB" ] || exit 0
    python3 - <<"PYEOF"
import os, sqlite3, sys
db = os.environ["DB"]
try:
    ok = sqlite3.connect(f"file:{db}?mode=ro", uri=True) \
                .execute("PRAGMA quick_check;").fetchone()[0] == "ok"
except sqlite3.DatabaseError:
    ok = False
sys.exit(0 if ok else 1)
PYEOF
' || {
    stamp=$(date +%Y%m%d-%H%M%S)
    echo "WARNING: $DB failed integrity check — archiving to $DB.corrupt-$stamp, starting a fresh map"
    docker exec isaac_ros mv "$DB" "$DB.corrupt-$stamp"
}

docker exec -d isaac_ros bash -c '
    unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
    export ROS_DOMAIN_ID=0
    source /opt/ros/jazzy/setup.bash
    source /workspaces/isaac_ros-dev/install/setup.bash
    exec ros2 run rtabmap_odom rgbd_odometry --ros-args \
        -p frame_id:=base_link \
        -p odom_frame_id:=odom \
        -p publish_tf:=false \
        -p approx_sync:=true \
        -p approx_sync_max_interval:=0.05 \
        -p qos:=2 \
        -p Reg/Force3DoF:="'"'"true"'"'" \
        -p Odom/ResetCountdown:="'"'"1"'"'" \
        -r rgb/image:=/camera/camera0/infra1/image_rect_raw \
        -r depth/image:=/camera/camera0/depth/image_rect_raw \
        -r rgb/camera_info:=/camera/camera0/infra1/camera_info \
        > /tmp/rgbd_odometry.log 2>&1'

# EKF (robot_localization): fuses visual /odom translation + D555 gyro heading
# and OWNS the odom->base_link TF (rgbd_odometry above now publishes_tf:=false).
# Fixes visual odom's rotation-blindness (48deg read for a real 360). Config +
# rationale in config/ekf.yaml. rtabmap (map->odom) stacks on top unchanged.
docker exec -d isaac_ros bash -c '
    unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
    export ROS_DOMAIN_ID=0
    source /opt/ros/jazzy/setup.bash
    source /workspaces/isaac_ros-dev/install/setup.bash
    exec ros2 run robot_localization ekf_node --ros-args \
        --params-file /workspaces/isaac_ros-dev/src/langrobo_perception/config/ekf.yaml \
        > /tmp/ekf.log 2>&1'

docker exec -d -e DB="$DB" isaac_ros bash -c '
    unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
    export ROS_DOMAIN_ID=0
    source /opt/ros/jazzy/setup.bash
    source /workspaces/isaac_ros-dev/install/setup.bash
    exec ros2 run rtabmap_slam rtabmap --ros-args \
        -p frame_id:=base_link \
        -p map_frame_id:=map \
        -p odom_frame_id:=odom \
        -p subscribe_depth:=true \
        -p approx_sync:=true \
        -p qos_image:=2 \
        -p qos_camera_info:=2 \
        -p database_path:=$DB \
        -p Reg/Force3DoF:="'"'"true"'"'" \
        -p Mem/IncrementalMemory:="'"'"true"'"'" \
        -r rgb/image:=/camera/camera0/infra1/image_rect_raw \
        -r depth/image:=/camera/camera0/depth/image_rect_raw \
        -r rgb/camera_info:=/camera/camera0/infra1/camera_info \
        > /tmp/rtabmap.log 2>&1'

echo "RTAB-Map starting — check: ros2 topic hz /odom ; tf map->odom->base_link"
