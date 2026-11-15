# camera/ -- the D555's camera stack, matched to its firmware

`./camera/build.sh` builds librealsense 2.59.2 (with DDS) and realsense-ros 4.59.2 inside the rover
image into `camera/ws/` (not in git; ~45 min). `./rover camera` uses them when they are there: the
library first on `LD_LIBRARY_PATH`, the ROS overlay sourced after the image's `/opt/realsense_ws`.

Why (2026-11-15): the D555 was moved to firmware 7.58.46386 (MTU work). The image's 4.58.2 driver
refused it ("Unsupported device! Product ID: 0xDDS" -- it matched only "Intel RealSense D555"), and its
2.58.2 library could not open depth ("more than one profile found for stream 'Aligned_Depth_To_Color'").
2.59.2 / 4.59.2 are the releases for that firmware.

Back to the image's 2.58.2 / 4.58.2 (only for the camera's old firmware): `rm -rf camera/ws`.
When the image is rebuilt, move docker/2.0/Dockerfile to these versions and delete this folder.

Patches on realsense-ros (`camera/patches/`, applied by build.sh):
- `realsense-ros-4.59.2-lyrical-no-fastdds-preload.patch`: 4.59 force-loads ROS's libfastdds (3.x on
  Lyrical) before librealsense, whose realdds has Fast DDS 2.10 built in; the node segfaulted at start.
