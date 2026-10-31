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
# Lyrical = the robot's distro (2026-10-25). Jazzy RViz saw the scan and maps but
# never the camera or the global costmap (2026-11-01): big messages do not cross
# Fast DDS 2.14 <-> 3.6. setup_lyrical_laptop.sh installs it; Jazzy is the fallback.
for d in lyrical jazzy; do
  [ -f "/opt/ros/$d/setup.bash" ] && { source "/opt/ros/$d/setup.bash"; break; }
done
[ "$ROS_DISTRO" = lyrical ] || echo "rover_live.sh: on $ROS_DISTRO, not lyrical -- no camera or global costmap (sudo bash ~/setup_lyrical_laptop.sh)" >&2
# must match the rover. `TWIN=1 ./rover view` sets 42 and its own config: the
# The twin's window, beside the real one.
export ROS_DOMAIN_ID="${ROVER_RVIZ_DOMAIN:-0}"
unset ROS_DISCOVERY_SERVER        # a stale one shows an empty graph
unset FASTRTPS_DEFAULT_PROFILES_FILE
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec rviz2 -d "${ROVER_RVIZ_CONFIG:-$HOME/rover_live.rviz}" "$@"
