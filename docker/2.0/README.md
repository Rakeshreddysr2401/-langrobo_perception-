# docker/2.0 — the rover's images on Isaac ROS 5.0 / ROS 2 Lyrical

Real recipes, built on the Jetson (Orin Nano, JetPack 7.2). Plan and test log:
`../../ISAAC_ROS_5_UPGRADE.md`.

## Two layers

```
nvcr.io/nvidia/base/ubuntu:noble-20251013
└── orin-nav:2.0   Dockerfile          core: pose, map, nav, camera, detection
    └── orin-nav:2.1   Dockerfile.features + localization (cuVGL, grid localizer), Grounding DINO,
        │                                  people segmentation, ESS/FoundationStereo (plan Part B)
        └── orin-arm:2.1   Dockerfile.arm  + the arm: MoveIt, cuMotion, SAM2, FoundationPose, pick-and-place
```
One container runs at a time (the top image you need); each image contains the ones below it.
Models (engines, ONNX, cuVGL maps) are host files in `~/rover/models/isaac/`, mounted read-only.

`./rover` runs `orin-nav:2.0`. The arm image adds packages only, never changes the nav
layer, so a broken arm package can't break driving. Each upper layer is written and built only when
its plan step starts, after the layer below passes its tests.

## What is in orin-nav:2.0

| job | package (5.0 / Lyrical) | source |
|---|---|---|
| visual odometry + SLAM | `isaac_ros_cuvslam` (was `isaac_ros_visual_slam`) | Isaac apt `release-5.0 noble-jetpack` |
| 3-D map / costmap layer | `nvblox_ros` | Isaac apt |
| GPU sharing (YOLO next to cuVSLAM) | `isaac_ros_gpu_partitioning` (CUDA MPS) | Isaac apt |
| navigation | nav2 1.5.0, slam_toolbox 2.10.0 | NVIDIA's Lyrical buildfarm (`isaac-ros/ubuntu/main noble`) |
| D555 camera (PoE / DDS) | librealsense v2.58.2 `BUILD_WITH_DDS=ON` → `/opt/librealsense`; realsense-ros 4.58.2 + NVIDIA's Lyrical patch → `/opt/realsense_ws` | source, in the Dockerfile |
| YOLO | our TensorRT runner (`phase4/nodes/yolo_backend.py`) on TensorRT 10.16 | Jetson / CUDA apt |
| python | numpy, scipy, OpenCV, lz4, yaml | Ubuntu apt |

Not in the image (host-mounted workspaces, built with the image): `sllidar_ros2`
(`lidar/build.sh`) and the rover's costmap plugin (`phase3/plugins/build.sh`).
Not in the image at all: torch/ultralytics (only the `.pt` fallback used them).

## Apt sources and why

| source | gives |
|---|---|
| `isaac.download.nvidia.com/isaac-ros/release-5.0 noble-jetpack` | Isaac ROS 5.0 for Jetson, `isaac-ros-cli` (its install writes the official CUDA 13.2 / TensorRT 10.16 pins) |
| `isaac.download.nvidia.com/isaac-ros/ubuntu/main noble` | ROS 2 Lyrical for Ubuntu 24.04 — packages.ros.org has none |
| `packages.ros.org` (`ros2-apt-source` 1.1.0) | colcon, rosdep |
| CUDA `ubuntu2404/sbsa`, Jetson `common r39.2` | CUDA 13.2, TensorRT, VPI |

## Build

```
cd ~/rover/docker/2.0
docker build -t orin-nav:2.0 .                      # ~1 h, mostly librealsense
docker build -f Dockerfile.arm -t orin-arm:2.0 .    # later, for the arm
```
Then rebuild the host-mounted workspaces against it:
`IMAGE=orin-nav:2.0 ~/rover/lidar/build.sh` and `IMAGE=orin-nav:2.0 ~/rover/phase3/plugins/build.sh`.
