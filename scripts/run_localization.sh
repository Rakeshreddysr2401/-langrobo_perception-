#!/bin/bash
# Localization backend dispatcher — THE single switch point for "where am I".
# Edit config/localization to change backend (one word):
#
#   rtabmap  (default) — RTAB-Map color+depth SLAM in the same Jazzy container
#                        as the camera driver. ~12Hz /odom, 450+ features,
#                        Force3DoF planar poses, auto-recovery. Verified.
#   cuvslam            — GPU stereo-IR SLAM in the Humble sidecar
#                        (run_cuvslam_sidecar.sh). PARKED 2026-07-16: explodes
#                        under motion — cross-distro DDS starves it of frames
#                        and TF; needs NVIDIA Orin/JP7 builds (moves it into
#                        the main container), IMU calibration and a rigid
#                        mount before re-adoption. See ISSUES_AND_SOLUTIONS.md.
#
# Both backends publish the same contract consumed by nvblox + Nav2 + the Pi5:
#   map->odom->base_link TF and an odometry topic (see localization_odom_topic).
set -eo pipefail
DIR=$(cd "$(dirname "$0")" && pwd)

backend=$(tr -d '[:space:]' < "$DIR/../config/localization" 2>/dev/null)
backend=${backend:-rtabmap}

case "$backend" in
    rtabmap) exec "$DIR/run_rtabmap.sh" ;;
    cuvslam) exec "$DIR/run_cuvslam_sidecar.sh" ;;
    *) echo "unknown localization backend '$backend' (config/localization)" >&2; exit 1 ;;
esac
