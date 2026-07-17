#!/bin/bash
# Stop the Jetson perception stack cleanly. The camera DRIVER is left running
# by default: restarting it trips the D555's stale-DDS failure mode, which
# only a physical power cycle clears. `stop_all.sh --full` kills it too.
# Kill by comm name — pkill -f patterns match their own docker-exec shell and
# kill the wrong thing (learned the hard way).
EXTRA=""
[ "${1:-}" = "--full" ] && EXTRA="realsense2_c static_transf"
docker exec isaac_ros bash -c '
    # launch parent FIRST — use_respawn:=True resurrects the nodes otherwise
    pkill -9 -f "[n]avigation_launch.py" 2>/dev/null
    sleep 1
    # rtabmap holds an open SQLite DB (/data/rtabmap.db) — SIGKILL mid-write
    # corrupts it (malformed-DB crash loop, 2026-07-17). TERM, wait, then -9.
    pkill -TERM rtabmap 2>/dev/null
    for i in $(seq 1 15); do pgrep rtabmap >/dev/null || break; sleep 1; done
    pkill -9 rtabmap 2>/dev/null
    for n in bt_navig planner_ser controller_s lifecycle_m collision_m \
             velocity_sm behavior_se smoother_se waypoint_f route_serv opennav \
             component_co nvblox_node rgbd_odomet '"$EXTRA"'; do
        pkill -9 "$n" 2>/dev/null
    done
    pkill -9 -f "[d]etections_3d" 2>/dev/null
    pkill -9 -f "[p]ixel_to_goal" 2>/dev/null
    pkill -9 -f "[c]md_vel_deadband" 2>/dev/null
    true' 2>/dev/null
docker stop cuvslam >/dev/null 2>&1
echo "perception stack stopped (containers stay up; camera untouched)"
