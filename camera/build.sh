#!/usr/bin/env bash
# Build the D555's camera stack -- librealsense (with DDS) + realsense-ros -- at the versions made
# for the camera's firmware, for the rover container.
#
# Why (2026-11-15): the image (docker/2.0/Dockerfile) has librealsense 2.58.2 + realsense-ros 4.58.2.
# The D555 got firmware 7.58.46386 (for the MTU work). With it, 4.58.2 refused the camera
# ("Unsupported device! Product ID: 0xDDS": it matched only the name "Intel RealSense D555", the
# firmware now says "RealSense D555"), and 2.58.2, once past that, could not open depth
# ("more than one profile found for stream 'Aligned_Depth_To_Color'": the firmware's new streams and
# duplicate profiles). librealsense 2.59.2 knows them; realsense-ros 4.59.2 builds for Lyrical as is
# and has no product-ID whitelist.
#
# Like lidar/build.sh: compiled INSIDE the image with this folder bind-mounted at /opt/camera, so the
# image is not changed (a new layer needed ~16 GB the disk did not have). Output, all under camera/ws/:
#   librealsense/            the library + tools (rs-enumerate-devices, rs-dds-config, ...)
#   install_<distro>/        realsense2_camera + realsense2_camera_msgs (an overlay)
# `./rover camera` uses them when they exist (library first on LD_LIBRARY_PATH, overlay sourced after
# /opt/realsense_ws). Back to the image's 2.58.2/4.58.2:  rm -rf camera/ws
#
#   ./camera/build.sh          both (~45 min, the library is most of it; stack down: it needs the cores)
#   ./camera/build.sh ros      only realsense-ros (a few minutes)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE=${IMAGE:-orin-nav:2.0}
LRS_REV=v2.59.2
ROS_REV=4.59.2
WS="$HERE/ws"
mkdir -p "$WS/src"

fetch(){  # fetch <url> <tag> <dir>
  [ -d "$3/.git" ] && [ "$(git -C "$3" describe --tags 2>/dev/null)" = "$2" ] && return
  rm -rf "$3"; echo "[camera] fetching $(basename "$1") $2"
  git clone -q --depth 1 -b "$2" "$1" "$3"
}
run(){ docker run --rm --network host --user "$(id -u):$(id -g)" -e HOME=/tmp --entrypoint bash \
         -v "$HERE":/opt/camera "$IMAGE" -c "for f in /opt/ros/*/setup.bash; do source \$f; done; $1"; }

if [ "${1:-all}" != ros ]; then
  fetch https://github.com/IntelRealSense/librealsense "$LRS_REV" "$WS/src/librealsense"
  echo "[camera] librealsense $LRS_REV with DDS (the image's flags, docker/2.0/Dockerfile step 3)"
  run "cmake -S /opt/camera/ws/src/librealsense -B /opt/camera/ws/build_librealsense \
         -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/camera/ws/librealsense \
         -DBUILD_WITH_DDS=ON -DFORCE_RSUSB_BACKEND=ON -DBUILD_WITH_CUDA=OFF \
         -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
         -DBUILD_TOOLS=ON -DBUILD_UNIT_TESTS=OFF > /opt/camera/ws/cmake_librealsense.log 2>&1 \
       && cmake --build /opt/camera/ws/build_librealsense -j4 2>&1 | tail -2 \
       && cmake --install /opt/camera/ws/build_librealsense > /dev/null"
  rm -rf "$WS/build_librealsense"         # ~2 GB of objects; the install is what runs
fi
[ -f "$WS/librealsense/lib/librealsense2.so" ] || { echo "[camera] librealsense build failed (ws/cmake_librealsense.log)"; exit 1; }

fetch https://github.com/IntelRealSense/realsense-ros "$ROS_REV" "$WS/src/realsense-ros"
touch "$WS/src/realsense-ros/realsense2_description/COLCON_IGNORE"    # the rover has its own description
# Our patches on top (camera/patches, each says why). Idempotent: one that applies in REVERSE is in.
for p in "$HERE"/patches/realsense-ros-$ROS_REV-*.patch; do
  [ -e "$p" ] || continue
  if git -C "$WS/src/realsense-ros" apply --reverse --check "$p" 2>/dev/null; then echo "[camera] patch already in: $(basename "$p")"
  else git -C "$WS/src/realsense-ros" apply "$p" && echo "[camera] applied: $(basename "$p")"; fi
done
rm -rf "$WS"/build_[a-z]*[!e]    # realsense-ros build dirs (not build_librealsense): configure fresh after a patch
echo "[camera] realsense-ros $ROS_REV against it"
run "cd /opt/camera/ws && S=_\$ROS_DISTRO && \
     CMAKE_PREFIX_PATH=/opt/camera/ws/librealsense:\$CMAKE_PREFIX_PATH \
     colcon --log-base log\$S build --merge-install --base-paths src/realsense-ros --build-base build\$S --install-base install\$S \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF 2>&1 | tail -3"
SUF=$(run 'echo "_$ROS_DISTRO"')
[ -f "$WS/install$SUF/lib/librealsense2_camera.so" ] \
  && echo "[camera] OK -- librealsense $LRS_REV + realsense-ros $ROS_REV; ./rover camera uses them" \
  || { echo "[camera] realsense-ros build failed"; exit 1; }
