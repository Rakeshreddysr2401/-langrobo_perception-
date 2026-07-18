#!/bin/bash
# LangRobo — ONE command to stop and start everything, properly.
#
# The single lifecycle entry point for the Jetson stack. Wraps the staged
# bring-up / safe teardown / health scripts AND the hard-won recovery steps so
# you don't have to remember the gotchas.
#
#   robot.sh start           Staged bring-up with health gates (run_all.sh):
#                            robot-TF -> camera -> localization -> nvblox ->
#                            Nav2 -> vision AI. Leaves a healthy camera alone.
#   robot.sh stop            Safe teardown; camera LEFT running (stop_all.sh).
#                            rtabmap gets SIGTERM first so its SQLite map DB
#                            is not corrupted mid-write.
#   robot.sh down            Full teardown INCLUDING the camera + robot TF.
#   robot.sh restart         Clean full cycle: down -> clear DDS -> start.
#                            Use this after a crash or config change.
#   robot.sh status          Health table, camera->wheels->Pi5->VLM.
#   robot.sh recover-camera  Fix a stuck D555 without a power cycle (see below).
#
# WHY 'restart' clears DDS and why 'recover-camera' exists (learned 2026-07-18,
# full write-up in REQUIREMENTS.md): the D555 is an Ethernet/DDS camera. If its
# driver is killed ungracefully, a STALE HOST-SIDE DDS participant strands the
# device's session and every relaunch logs "No RealSense devices were found!"
# even though the camera still pings. A physical power cycle does NOT clear it;
# a clean host DDS teardown (stop all nodes + `ros2 daemon stop` + remove
# /dev/shm/fastrtps*) does. Both paths below do that before touching the camera.
set -eo pipefail
DIR=$(cd "$(dirname "$0")" && pwd)

in_container() {  # run a bash snippet inside the isaac_ros container
    docker exec isaac_ros bash -c "
        unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
        export ROS_DOMAIN_ID=0
        source /opt/ros/jazzy/setup.bash
        $*"
}

clear_dds() {  # drop every stale DDS participant on the host side
    echo "  clearing host DDS (daemon + shared memory)..."
    in_container 'ros2 daemon stop >/dev/null 2>&1 || true'
    docker exec isaac_ros bash -c \
        'rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps* \
               /dev/shm/fastdds*  /dev/shm/sem.fastdds* 2>/dev/null; true'
}

wait_camera() {  # true if infra1 is streaming within <deadline_s>
    local deadline=$1 start=$SECONDS
    while (( SECONDS - start < deadline )); do
        in_container "timeout 4 ros2 topic echo /camera/camera0/infra1/image_rect_raw --once >/dev/null 2>&1" \
            && return 0
    done
    return 1
}

cmd=${1:-}
case "$cmd" in

    start)
        exec "$DIR/run_all.sh"
        ;;

    stop)
        exec "$DIR/stop_all.sh"
        ;;

    down)
        exec "$DIR/stop_all.sh" --full
        ;;

    status)
        exec "$DIR/status_all.sh"
        ;;

    restart)
        echo "[1/3] full teardown (safe: rtabmap DB protected)"
        "$DIR/stop_all.sh" --full >/dev/null 2>&1 || true
        sleep 2
        echo "[2/3] clear DDS so the D555 is discoverable"
        clear_dds
        sleep 2
        echo "[3/3] staged bring-up"
        exec "$DIR/run_all.sh"
        ;;

    recover-camera)
        echo "Recovering the D555 (stale-DDS, no power cycle needed)."
        echo "[1/3] tearing down all nodes so nothing holds the device"
        "$DIR/stop_all.sh" --full >/dev/null 2>&1 || true
        sleep 2
        echo "[2/3] clearing host DDS"
        clear_dds
        sleep 2
        echo "[3/3] launching the camera alone (+ robot TF, emitter off)"
        "$DIR/run_d555_stereo.sh" >/dev/null
        if wait_camera 45; then
            echo "  [ok] D555 streaming again — now 'robot.sh start' for the full stack"
        else
            echo "  [FAIL] still no camera. LAST RESORT: unplug the D555 POWER for" >&2
            echo "         20s, replug, wait 45s, then rerun 'robot.sh recover-camera'." >&2
            echo "         (Do NOT run anything else while it boots.)" >&2
            exit 1
        fi
        ;;

    *)
        # Print the header comment block (from line 2 to the first non-# line).
        awk 'NR>=2 && /^#/ {sub(/^# ?/,""); print; next} NR>=2 {exit}' "$0"
        exit 1
        ;;
esac
