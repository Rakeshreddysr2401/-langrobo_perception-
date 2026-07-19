# Consolidated, self-contained Orin navigation stack — ONE image:
#   cuVSLAM (Orin-native pyCuVSLAM) + nvblox + nav2, JetPack 7.2 / Jazzy.
#
# Built FROM isaac_ros:cuvslam-unified, which already contains:
#   - Isaac ROS Jazzy base + nvblox_ros + realsense2_camera (D555 DDS) + nav2 + cv_bridge
#   - the Orin-native cu12 cuVSLAM wheel + its CUDA-12 user-space libs
#   - the cuVSLAM ROS 2 wrapper node
# Here we bake in OUR configs so nothing external is mounted at runtime — the
# langrobo_perception workspace is NOT required and can be deleted.
#
# NOTE (future): cuvslam-unified is layered on the big langrobo-prod image, so it
# still carries RTABMap/YOLO/etc. To slim down, rebuild FROM an isaac-ros base
# with only nvblox + realsense + cuVSLAM + these configs (see README "Lean image").
FROM isaac_ros:cuvslam-unified

# cuVSLAM ROS 2 wrapper node (subscribes D555 stereo -> publishes odom->base_link + /odom)
COPY cuvslam_ros_node.py /opt/orin-nav/cuvslam_ros_node.py

# Our owned configs (nav2 params w/ baked no-blind-recovery BT, nvblox overrides)
COPY config/ /opt/orin-nav/config/

# Vision/goal/motor nodes (standalone rclpy scripts) + YOLO weights
#   detections_3d, pixel_to_goal, cmd_vel_deadband (speed-capped), imu_to_base,
#   visual_approach, drive_test
COPY nodes/ /opt/orin-nav/nodes/
COPY models/ /opt/orin-nav/models/

LABEL org.orin-nav.stack="cuvslam+nvblox+nav2" \
      org.orin-nav.note="self-contained; no workspace mount required"
