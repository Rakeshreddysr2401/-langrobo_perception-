#!/usr/bin/env bash
# Start RViz for the rover.
#
# THE -d IS LOAD-BEARING. `rviz2 ~/rover.rviz` looks like it loads the config and
# does not: rviz2 ignores a bare path and starts with its DEFAULTS -- Fixed Frame
# "map", which does not exist on this robot, and zero displays. The result is a
# blank window with no error, and the ROS graph shows rviz connected while
# subscribing to nothing. That cost an evening; this script exists so it cannot
# happen again.
export DISPLAY="${DISPLAY:-:0}"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0            # must match the rover
unset ROS_DISCOVERY_SERVER        # a stale one shows an empty graph
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec rviz2 -d "$HOME/rover.rviz" "$@"
