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

# NOTE: align_depth.enable is accepted but publishes NOTHING on the DDS
# driver (same trap as pointcloud.enable — SDK processing blocks don't exist
# for network cameras). Instead we feed infra1 as the "rgb" image: RealSense
# depth is computed in infra1's viewpoint, so infra1 + raw depth are
# perfectly registered by construction (same frame, same 896x504 grid) and
# rtabmap takes mono8 fine. Emitter must stay OFF (dots would corrupt the
# feature tracking on the IR image).

docker exec isaac_ros bash -c 'pkill -9 -f "[r]gbd_odometry" 2>/dev/null; pkill -9 -f "[r]tabmap_slam" 2>/dev/null; pkill -9 rtabmap 2>/dev/null; true'
sleep 2

docker exec -d isaac_ros bash -c '
    unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
    export ROS_DOMAIN_ID=0
    source /opt/ros/jazzy/setup.bash
    source /workspaces/isaac_ros-dev/install/setup.bash
    exec ros2 run rtabmap_odom rgbd_odometry --ros-args \
        -p frame_id:=base_link \
        -p odom_frame_id:=odom \
        -p publish_tf:=true \
        -p approx_sync:=true \
        -p approx_sync_max_interval:=0.05 \
        -p qos:=2 \
        -p Reg/Force3DoF:="'"'"true"'"'" \
        -p Odom/ResetCountdown:="'"'"1"'"'" \
        -r rgb/image:=/camera/camera0/infra1/image_rect_raw \
        -r depth/image:=/camera/camera0/depth/image_rect_raw \
        -r rgb/camera_info:=/camera/camera0/infra1/camera_info \
        > /tmp/rgbd_odometry.log 2>&1'

docker exec -d isaac_ros bash -c '
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
        -p database_path:=/data/rtabmap.db \
        -p Reg/Force3DoF:="'"'"true"'"'" \
        -p Mem/IncrementalMemory:="'"'"true"'"'" \
        -r rgb/image:=/camera/camera0/infra1/image_rect_raw \
        -r depth/image:=/camera/camera0/depth/image_rect_raw \
        -r rgb/camera_info:=/camera/camera0/infra1/camera_info \
        > /tmp/rtabmap.log 2>&1'

echo "RTAB-Map starting — check: ros2 topic hz /odom ; tf map->odom->base_link"
