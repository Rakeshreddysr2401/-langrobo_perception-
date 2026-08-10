#!/bin/bash
# Launch RViz2 to visualize cuVSLAM tracking (D555 stereo IR + sidecar).
# Run from the Jetson HOST — opens the GUI on the Jetson display.
#
# Shows: left IR camera view, cuVSLAM odometry arrows, SLAM path line,
# landmark cloud (blue) and live observations (red).
set -eo pipefail

DISPLAY=${DISPLAY:-:1}
export DISPLAY

xhost +local:docker 2>/dev/null || true

RVIZ_CONFIG=/workspaces/isaac_ros-dev/src/langrobo_perception/config/rviz_cuvslam.rviz

docker exec -e DISPLAY="$DISPLAY" isaac_ros bash -c "
  unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
  export ROS_DOMAIN_ID=0
  source /opt/ros/jazzy/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null || true
  export LD_LIBRARY_PATH=/root/librealsense/install/lib:\${LD_LIBRARY_PATH}
  exec rviz2 -d $RVIZ_CONFIG
"
