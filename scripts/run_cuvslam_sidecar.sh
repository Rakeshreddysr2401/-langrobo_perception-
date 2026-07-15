#!/bin/bash
# Launch cuVSLAM (Isaac ROS 3.2.6, Orin build) in the Humble sidecar container.
# Run from the Jetson HOST.
#
# Verified working 2026-07-15: cuVSLAM 12.6 tracking at ~24Hz on Orin Nano JP7.2.
#
# Prerequisites:
#   - image cuvslam-sidecar:3.2.6 built (docker/cuvslam-sidecar/)
#   - D555 stereo IR streaming: run_d555_stereo.sh (infra1/infra2, emitter off)
#
# Notes:
#   - UDP-only DDS profile is required: the jazzy isaac_ros container has
#     private IPC, so FastDDS shared memory cannot cross containers.
#   - No ROS_DISCOVERY_SERVER: plain multicast so both containers on the host
#     discover each other without the Pi5.
set -eo pipefail

PROFILE=/home/rakhi24/workspaces/isaac_ros-dev/src/langrobo_perception/docker/cuvslam-sidecar/udp_only.xml

docker rm -f cuvslam 2>/dev/null || true

exec docker run -d --name cuvslam \
    --runtime nvidia \
    --network host \
    -v "$PROFILE":/udp_only.xml:ro \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/udp_only.xml \
    -e ROS_DOMAIN_ID=0 \
    cuvslam-sidecar:3.2.6 \
    ros2 run isaac_ros_visual_slam isaac_ros_visual_slam --ros-args \
        -p num_cameras:=2 \
        -p min_num_images:=2 \
        -p rectified_images:=true \
        -p enable_image_denoising:=false \
        -p enable_localization_n_mapping:=true \
        -p enable_slam_visualization:=true \
        -p enable_observations_view:=true \
        -p enable_landmarks_view:=true \
        -p base_frame:=base_link \
        -p camera_optical_frames:="[camera0_infra1_optical_frame,camera0_infra2_optical_frame]" \
        -r /visual_slam/image_0:=/camera/camera0/infra1/image_rect_raw \
        -r /visual_slam/camera_info_0:=/camera/camera0/infra1/camera_info \
        -r /visual_slam/image_1:=/camera/camera0/infra2/image_rect_raw \
        -r /visual_slam/camera_info_1:=/camera/camera0/infra2/camera_info \
        -p enable_imu_fusion:=false \
        -r /visual_slam/imu:=/camera/camera0/motion/sample \
        "$@"
# IMU fusion OFF: with default noise params the D555 IMU diverges (~850m
# position blow-up while stationary, verified 2026-07-15). Re-enable with
# -p enable_imu_fusion:=true only after calibrating gyro/accel noise densities
# against /camera/camera0/motion/imu_info and verifying the motion-frame
# extrinsics. Visual-only tracking is stable (0.000m drift stationary).
