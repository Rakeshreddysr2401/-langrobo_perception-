#!/bin/bash
# How far can Nav2 actually plan RIGHT NOW?
#
# Sweeps compute_path_to_pose outward in several directions and reports the
# farthest goal the global planner can reach through KNOWN-FREE space. This is
# a planning-only probe — it never sends a NavigateToPose goal, so the robot
# does NOT move.
#
# Use it to answer "is my map built enough to navigate?" A freshly-started or
# stationary robot sees only a small forward patch; after you drive/map an area
# (see MAPPING_WORKFLOW.md) the reachable radius grows and persists in the
# RTAB-Map DB.
#
#   scripts/nav_reachability.sh            # default sweep, goals in map frame
#
# Reads the robot's current map pose and probes RELATIVE to it, so it works
# wherever the robot is.
set -eo pipefail

docker exec isaac_ros bash -c '
  unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
  source /opt/ros/jazzy/setup.bash

  # robot pose in map (fall back to origin)
  # line looks like:  - Translation: [x, y, z]  -> fields 3,4 after stripping punctuation
  read RX RY < <(timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null \
    | grep -m1 Translation | tr -d "[],:" | awk "{print \$3, \$4}")
  RX=${RX:-0.0}; RY=${RY:-0.0}
  echo "robot at map (${RX}, ${RY}) — probing reachability (no motion)"

  reachable() {  # reachable <dx> <dy>  (offset from robot, metres)
    gx=$(python3 -c "print($RX + $1)"); gy=$(python3 -c "print($RY + $2)")
    out=$(timeout 12 ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
      "{goal: {header: {frame_id: map}, pose: {position: {x: $gx, y: $gy}, orientation: {w: 1.0}}}, use_start: false}" 2>/dev/null)
    echo "$out" | grep -q "status: SUCCEEDED"
  }

  # For each heading, walk outward until planning fails; report last success.
  for dir in "forward 1 0" "left 0 1" "right 0 -1" "back -1 0"; do
    set -- $dir; name=$1; ux=$2; uy=$3
    best=0.0
    for r in 0.15 0.25 0.40 0.60 0.80 1.00 1.50 2.00 3.00; do
      dx=$(python3 -c "print($ux * $r)"); dy=$(python3 -c "print($uy * $r)")
      if reachable "$dx" "$dy"; then best=$r; else break; fi
    done
    printf "  %-8s reachable to: %s m\n" "$name" "$best"
  done
  echo "(0.0 m = nothing that way is mapped as free yet — drive/rotate to map it)"
'
