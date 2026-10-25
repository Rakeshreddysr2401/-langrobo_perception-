# docker/ — the rover's image

**The image is `orin-nav:2.0`** (Isaac ROS 5.0, ROS 2 Lyrical), built from the Dockerfile in
[`2.0/`](2.0/README.md), which also holds its checks (`check.sh`, `vo_check.sh`, `nvblox_check.sh`,
`nav2_check.sh`, audits). `orin-nav:2.1` = 2.0 + the Part B features (`2.0/Dockerfile.features`).

The older `docker commit` images 1.1 (pyCuVSLAM, Isaac ROS 4.4) and 1.2 (Isaac ROS 4.6, Jazzy)
were deleted on 2026-10-24/25; their recipe and notes are in git history
(`git log -- docker/Dockerfile ISAAC_ROS_4_6_UPGRADE.md`).
