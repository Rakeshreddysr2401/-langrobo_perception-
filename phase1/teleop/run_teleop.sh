#!/usr/bin/env bash
# LangRobo mobile teleop web server (hold-to-move -> /cmd_vel).
# Mirrors the brain's ROS env: domain 0, FastRTPS, plain multicast (NOT the
# discovery server) so it shares the graph with the micro-ROS agent + Jetson.
# ROS distro: the Pi 5's one switch (~/ros2_ws/scripts/ros_env.sh), Lyrical since
# 2026-10-25; `echo jazzy > ~/.langrobo/ros_distro` rolls back.
set -eo pipefail

D="${KINO_ROS_DISTRO:-$(cat "$HOME/.langrobo/ros_distro" 2>/dev/null || echo lyrical)}"
set +u
source "/opt/ros/$D/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER || true
unset ROS_LOCALHOST_ONLY

exec python3 "$(dirname "$0")/teleop_web.py"
