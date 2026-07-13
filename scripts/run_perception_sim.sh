#!/bin/bash
# Launch LangRobo perception (nvblox + Nav2), sim profile, inside the Isaac
# ROS container. Run from the Jetson host:
#   docker exec -it isaac_ros_dev_container /workspaces/isaac_ros-dev/src/langrobo_perception/scripts/run_perception_sim.sh
# Prerequisite: rover_sim Gazebo running on the laptop with its DDS peers
# profile (see rover_sim docs/INTERFACE.md).
set -eo pipefail

export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/isaac_ros-dev/config/fastdds_unicast.xml

# Register with the Pi5 "meeting point" (Fast DDS Discovery Server) so the
# brain — a discovery-server client — can see Nav2 and the cmd_vel stamper.
# Resolve the name to IPv4 explicitly: mDNS prefers IPv6 here and the server
# is UDPv4-only, which breaks registration silently (see Pi5 NETWORKING.md).
PI5_IP=$(getent ahostsv4 rakhi24-desktop.local 2>/dev/null | awk 'NR==1{print $1}')
export ROS_DISCOVERY_SERVER="${PI5_IP:-192.168.1.16}:11811"
echo "discovery server: $ROS_DISCOVERY_SERVER"

source /opt/ros/jazzy/setup.bash
if [ -f /workspaces/isaac_ros-dev/install/setup.bash ]; then
    source /workspaces/isaac_ros-dev/install/setup.bash
else
    echo "workspace not built — run:" >&2
    echo "  cd /workspaces/isaac_ros-dev && colcon build --symlink-install --packages-select langrobo_perception" >&2
    exit 1
fi

exec ros2 launch langrobo_perception perception.launch.py mode:=sim "$@"
