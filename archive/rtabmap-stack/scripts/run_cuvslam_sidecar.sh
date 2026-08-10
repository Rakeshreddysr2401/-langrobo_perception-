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

docker run -d --name cuvslam \
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
        `# INFRA1 = left, INFRA2 = right (proven by patch-match disparity` \
        `# gradient 2026-07-16: near objects appear 26..115px further LEFT in` \
        `# infra2 = standard RealSense layout). NOTE the DDS driver exports` \
        `# MIRRORED extrinsics (TF infra2 at -0.095 optical-x, camera_info Tx` \
        `# +42.5 instead of -42.5) — do NOT trust driver TF for the rig; the` \
        `# corrected static chain below is the only extrinsics source here.` \
        -p camera_optical_frames:="[camera0_infra1_optical_frame,camera0_infra2_optical_frame]" \
        -r /visual_slam/image_0:=/camera/camera0/infra1/image_rect_raw \
        -r /visual_slam/camera_info_0:=/camera/camera0/infra1/camera_info \
        -r /visual_slam/image_1:=/camera/camera0/infra2/image_rect_raw \
        -r /visual_slam/camera_info_1:=/camera/camera0/infra2/camera_info_fixed \
        -p enable_imu_fusion:=false \
        -r /visual_slam/imu:=/camera/camera0/motion/sample \
        "$@"

# The Jazzy container's /tf_static does NOT deserialize in this Humble
# container ("sequence size exceeds remaining buffer") — cuVSLAM could never
# resolve base_link and ran with NO stereo extrinsics (0-4 junk features,
# drift at rest, pose explosions under motion; found 2026-07-16). Publish the
# full static chain locally so the rig never depends on cross-distro DDS.
# base_link->camera0_link mirrors run_d555_stereo.sh; infra2 at +0.095
# body-y matches the driver's own TF (A/B burst-tested 2026-07-16: the
# -0.095 variant exploded odometry to -1200m in seconds; +0.095 is correct
# in this TF convention even though infra2 is the physically RIGHT camera).
sleep 3
# camera_info Tx-sign relay: the driver exports P[3]=+fx*B on infra2 but the
# ROS stereo convention (and cuVSLAM's rig builder) needs -fx*B on the right
# camera. cuVSLAM subscribes camera_info_fixed (remap above).
docker cp "$(dirname "$0")/../docker/cuvslam-sidecar/info_fix.py" cuvslam:/tmp/info_fix.py
docker exec -d cuvslam bash -c '. /opt/ros/humble/setup.bash && export FASTRTPS_DEFAULT_PROFILES_FILE=/udp_only.xml && exec python3 /tmp/info_fix.py'
docker exec -d cuvslam bash -c '. /opt/ros/humble/setup.bash && exec ros2 run tf2_ros static_transform_publisher --x 0.10 --y 0 --z 0.25 --frame-id base_link --child-frame-id camera0_link'
docker exec -d cuvslam bash -c '. /opt/ros/humble/setup.bash && exec ros2 run tf2_ros static_transform_publisher --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id camera0_link --child-frame-id camera0_infra1_optical_frame'
docker exec -d cuvslam bash -c '. /opt/ros/humble/setup.bash && exec ros2 run tf2_ros static_transform_publisher --y 0.095 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id camera0_link --child-frame-id camera0_infra2_optical_frame'
echo "cuvslam sidecar up (with in-container static TF chain)"
# IMU fusion OFF: with default noise params the D555 IMU diverges (~850m
# position blow-up while stationary, verified 2026-07-15). Re-enable with
# -p enable_imu_fusion:=true only after calibrating gyro/accel noise densities
# against /camera/camera0/motion/imu_info and verifying the motion-frame
# extrinsics. Visual-only tracking is stable (0.000m drift stationary).
