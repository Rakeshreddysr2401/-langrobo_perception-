#!/bin/bash
# Launch RViz2 for LangRobo real perception stack (D555 + RTAB-Map + nvblox).
# Run from the Jetson HOST (not inside the container) — it opens the GUI on :1.
# Prerequisites: perception stack already running via run_all.sh.
#
# Usage:
#   ./run_rviz_real.sh            # launches on Jetson display :1
#   DISPLAY=:0 ./run_rviz_real.sh # if Jetson uses display :0
set -eo pipefail

DISPLAY=${DISPLAY:-:1}
export DISPLAY

# Allow the container to draw on the host X server.
xhost +local:docker 2>/dev/null || true

RVIZ_CONFIG=/workspaces/isaac_ros-dev/src/langrobo_perception/config/rviz_real.rviz

docker exec -e DISPLAY="$DISPLAY" isaac_ros bash -c "
  unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
  source /opt/ros/jazzy/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null || true
  export LD_LIBRARY_PATH=/root/librealsense/install/lib:\${LD_LIBRARY_PATH}
  exec rviz2 -d $RVIZ_CONFIG
"
