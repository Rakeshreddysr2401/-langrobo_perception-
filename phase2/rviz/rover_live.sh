#!/usr/bin/env bash
# Start RViz for the rover. Lives on the laptop as ~/rover_live.sh; `./rover
# view` pushes the CONFIG here on every run, so the repo stays the single source.
#
# THE -d IS LOAD-BEARING. `rviz2 ~/rover_live.rviz` looks like it loads the
# config and does not: rviz2 ignores a bare path and starts with its DEFAULTS --
# Fixed Frame "map", which does not exist on this robot, and zero displays. The
# result is a blank window with no error, and the ROS graph shows rviz connected
# while subscribing to nothing. That cost an evening; this script exists so it
# cannot happen again.
export DISPLAY="${DISPLAY:-:0}"
source /opt/ros/jazzy/setup.bash
# must match the rover. `TWIN=1 ./rover view` sets 42 and its own config: the
# The twin's window, beside the real one.
export ROS_DOMAIN_ID="${ROVER_RVIZ_DOMAIN:-0}"
unset ROS_DISCOVERY_SERVER        # a stale one shows an empty graph
unset FASTRTPS_DEFAULT_PROFILES_FILE
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec rviz2 -d "${ROVER_RVIZ_CONFIG:-$HOME/rover_live.rviz}" "$@"
