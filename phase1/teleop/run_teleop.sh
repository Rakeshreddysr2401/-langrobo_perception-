#!/usr/bin/env bash
# LangRobo mobile teleop web server (hold-to-move -> /cmd_vel).
# Mirrors the brain's ROS env: domain 0, FastRTPS, plain multicast (NOT the
# discovery server) so it shares the graph with the micro-ROS agent + Jetson.
set -eo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER || true
unset ROS_LOCALHOST_ONLY

exec python3 "$(dirname "$0")/teleop_web.py"
