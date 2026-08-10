#!/bin/bash
# Prints the odometry topic of the active localization backend (used by
# run_all.sh / status_all.sh health gates).
DIR=$(cd "$(dirname "$0")" && pwd)
backend=$(tr -d '[:space:]' < "$DIR/../config/localization" 2>/dev/null)
case "${backend:-rtabmap}" in
    cuvslam) echo /visual_slam/tracking/odometry ;;
    *)       echo /odom ;;
esac
