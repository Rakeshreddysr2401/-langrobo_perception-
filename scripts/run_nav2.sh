#!/bin/bash
# Launch Nav2 for the LangRobo rover (cuVSLAM + nvblox stack). Run from HOST.
#
# Prerequisites (in order):
#   1. run_d555_stereo.sh      — camera
#   2. run_cuvslam_sidecar.sh  — localization (map->odom->base_link)
#   3. run_nvblox.sh           — costmaps
#
# Send a goal from RViz ("2D Goal Pose" toolbar button) or CLI:
#   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
#     "{pose: {header: {frame_id: map}, pose: {position: {x: 0.5}, orientation: {w: 1.0}}}}"
set -eo pipefail

# Kill the LAUNCH PARENT first: with use_respawn:=True it resurrects every
# node killed below, leaving TWO stacks fighting over /cmd_vel (seen live
# 2026-07-16 — "unknown goal response" in bt_navigator, rover hum, no motion).
# [n] bracket keeps pkill -f from matching this docker-exec shell itself.
docker exec isaac_ros bash -c 'pkill -9 -f "[n]avigation_launch.py"; true'
sleep 1
docker exec isaac_ros bash -c 'pkill -9 bt_navig; pkill -9 planner_ser; pkill -9 controller_s; pkill -9 lifecycle_m; pkill -9 collision_m; pkill -9 velocity_sm; pkill -9 behavior_se; pkill -9 smoother_se; pkill -9 waypoint_f; pkill -9 route_serv; pkill -9 opennav; true'
sleep 3

docker exec -d isaac_ros bash -c '
    unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
    export ROS_DOMAIN_ID=0
    source /opt/ros/jazzy/setup.bash
    source /workspaces/isaac_ros-dev/install/setup.bash
    exec ros2 launch nav2_bringup navigation_launch.py \
        params_file:=/workspaces/isaac_ros-dev/src/langrobo_perception/config/nav2_real.yaml \
        use_sim_time:=False use_composition:=False \
        autostart:=True use_respawn:=True \
        > /tmp/nav2.log 2>&1'

echo "Nav2 starting — wait ~30s, then check:"
echo "  docker exec isaac_ros grep -a 'Managed nodes are active' /tmp/nav2.log"
