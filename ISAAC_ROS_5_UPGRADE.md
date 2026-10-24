# Isaac ROS 4.6 (Jazzy) → 5.0 (Lyrical): the plan

Started 2026-10-24 on branch `isaac-ros-5`. Owner's call: move now, while the project
is early, because the next big goals are a robot arm and pick-and-place.

## Why

| benefit | why it matters for us |
|---|---|
| FoundationPose library, up to 5.5× faster (NVIDIA's claim) | object 6-D pose = what an arm needs before grasping |
| FoundationStereo fine-tuning skill | better depth from the D555 IR pair for small objects |
| `isaac_ros_gpu_partitioning` (CUDA MPS) | YOLO overloading the GPU already broke cuVSLAM pose once (2026-07-19) |
| ROS 2 Lyrical (newer long-term release) | build the arm work on it once instead of migrating later |
| NITROS replaced by the standard ROS 2 GPU buffer | plainer interop with MoveIt / arm drivers |
| fixes only land in 5.x | 4.6 on this board: nvblox sm_75-only, image TensorRT 10.13 refuses SM87 |

Navigation itself is not expected to improve. Sources: the 5.0.0 release notes
(https://nvidia-isaac-ros.github.io/releases/index.html) and getting started page.

## Facts checked (2026-10-24)

- **Jetson OK:** 5.0 supports Jetson Orin + Thor on **JetPack 7.2** (R39.2). This board: R39.2.1. 
- **Packages exist for Jetson** (`release-5.0 noble-jetpack`, 320 packages): `isaac-ros-cuvslam`,
  `isaac-ros-nvblox`, `isaac-ros-cumotion(-moveit)`, `isaac-ros-foundationpose`,
  `isaac-ros-realsense`, `isaac-ros-yolov8`, `isaac-ros-tensor-rt`.
- **ROS Lyrical for Ubuntu 24.04 comes from NVIDIA's buildfarm**, not packages.ros.org
  (`https://isaac.download.nvidia.com/isaac-ros/ubuntu/main noble`, 2673 arm64 packages):
  nav2 1.5.0 (mppi, smac, collision monitor, velocity smoother, map server), slam_toolbox 2.10.0,
  MoveIt 2.15.0, ros2_control 6.9.0, cv_bridge, image_transport, rosbag2_py, rviz2.
  **Missing there:** `sllidar_ros2` (we build it from source already: `lidar/build.sh`),
  `micro_ros_agent`, `nvblox_rviz_plugin` (laptop side).
- **Base image** (`isaac-ros-cli` 2.6.0, `Dockerfile.isaac_ros`): plain
  `nvcr.io/nvidia/base/ubuntu:noble-20251013` + Isaac apt + Jetson `r39.2` repo + ROS buildfarm +
  `ros-lyrical-ros-base`. Small, so 2.0 can be a **real Dockerfile**, not `docker commit` layers.
- **Renamed:** `isaac_ros_visual_slam` → `isaac_ros_cuvslam` (interfaces package keeps its name).
- **Removed:** all NITROS packages. Our own code never calls NITROS APIs (checked: no hits).
- **Known 5.0 issues that touch us:** RealSense supported in Docker mode only (we use Docker);
  `isaac_ros_stereo_image_proc` VPI error on AGX Orin JETSON backend (use CUDA backend);
  YOLOv8 spurious component-load errors; **cuMotion MoveIt examples may fail to launch
  on Lyrical** (vendor packages not certified) — the arm's weakest point.
- **Disk:** 49 GB free, `orin-nav:1.2` = 73 GB. 2.0 must stay lean; 1.2 is deleted only at the end.

## What the rover uses today → where it comes from in 2.0

| today (orin-nav:1.2, Jazzy) | 2.0 (Lyrical) | work |
|---|---|---|
| `isaac_ros_visual_slam` 4.6 (VO) | `isaac_ros_cuvslam` 5.0 | rename in `phase1/launch/vo_isaac.launch.py`; check sm_87 + 0 SVE |
| `nvblox_ros` 4.6 (sm_75 only → 4-min JIT) | `isaac_ros_nvblox` 5.0 | check for sm_87 code |
| librealsense **2.58.2 source build, `BUILD_WITH_DDS=ON`** (D555 is PoE/DDS) | same, built in the Dockerfile | the buildfarm's librealsense2 2.57.7 is not our DDS build |
| realsense-ros 4.58.x | build from source against Lyrical | apt has 4.57.7 only |
| nav2 + slam_toolbox (Jazzy) | buildfarm nav2 1.5.0, slam_toolbox 2.10.0 | rebuild `phase3/plugins` (costmap plugin) against nav2 1.5 |
| sllidar_ros2 (source, `lidar/build.sh`) | same, rebuilt | |
| YOLO: own TensorRT runner + `/hosttrt` mount (image TRT 10.13 refuses SM87) | TensorRT 10.16 from the Jetson repo | drop the `/hosttrt` mount if 10.16 loads the engine in-image |
| torch 2.12 + ultralytics (`.pt` fallback only) | optional | leave out unless needed (saves GBs) |
| python: numpy, scipy, cv2, lz4, yaml | apt in the Dockerfile | |
| `/opt/ros/jazzy` hard-coded | `/opt/ros/$ROS_DISTRO` | 7 files: `rover`, `phase1/teleop/run_teleop.sh`, `phase2/rviz/rover_live.sh`, `phase3/{config/nav2.yaml,plugins/build.sh,launch/nav2.launch.py}`, `lidar/build.sh`, `ops/rover_watchdog.py` |
| `libnvcuvid` bind-mount, CUDA cache mount, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` | keep | re-check each is still needed |

## Steps (each one gated; nothing moves the robot without the owner's OK)

1. **Build `orin-nav:2.0`** from `docker/2.0/Dockerfile` on the Jetson. No hardware, no ROS domain 0.
2. **Binary checks**, as for 4.6: SVE instruction count = 0 and sm_87 GPU code in
   `libcuvslam.so` and nvblox; TensorRT 10.16 builds and loads `yolov8n_640x384` on SM87.
3. **Synthetic tests** in 2.0 (fake stereo → cuVSLAM at 30 Hz, nvblox first-run time).
4. **Port the code** on this branch: package rename, `ROS_DISTRO` paths, `./rover`
   `IMAGE=orin-nav:2.0`, plugin + sllidar rebuilds. Old image keeps working via `IMAGE=orin-nav:1.2`.
5. **Laptop twin, step by step (owner, needs sudo).** One change at a time, `./sim test contract`
   after each:
   - a. confirm which Isaac Sim release bundles Lyrical (6.2 bundles Jazzy) and its driver need;
   - b. driver 580 → the version that Isaac Sim needs (595+ if anything Isaac ROS 5 runs there);
   - c. Isaac Sim upgrade, ROS bridge = Lyrical;
   - d. laptop RViz / tools on Lyrical.
   Until (c), test whether the Jazzy twin and the Lyrical Jetson talk on domain 42
   (`/sim/hardware`, `./sim parity compare`). If they don't, step 6 waits for (c).
   **The Pi 5 is the same question on the real robot:** brain, teleop and the micro-ROS
   agent (ESP32) run Jazzy there and talk to the Jetson over DDS. Test Jazzy↔Lyrical on the
   real graph standing still; if it fails, the Pi 5 moves to Lyrical too (NVIDIA's buildfarm
   has arm64 noble packages; `micro_ros_agent` must be built from source).
6. **Twin run on 2.0:** `TWIN=1 IMAGE=orin-nav:2.0 ./rover up`, the same scenarios as 1.2
   (`./sim scenario`, `./sim agents`), numbers side by side.
7. **Arm in the twin:** cuMotion + MoveIt on Lyrical (the known-issue risk), then FoundationPose.
8. **Real robot on 2.0:** standing-still pose stack (VO, fusion, nvblox, YOLO), then a driven
   test only with the owner's OK. Rules in OPERATIONS.md and the safety notes still apply.
9. **Clean up:** delete `orin-nav:1.2` only after 8 passes and the owner agrees; update
   ARCHITECTURE / OPERATIONS / docker/README; merge `isaac-ros-5`.

## Rollback

Until step 9: `IMAGE=orin-nav:1.2 ./rover up` on the old branch. The host is not changed
by this plan (JetPack 7.2.1 stays; the host `isaac-ros-cli` 2.5 stays dormant).

## Log

- 2026-10-24: binary check of the 5.0 debs: `libcuvslam.so`, the cuvslam node and
  `libnvblox_ros_lib.so` have **0 SVE**; GPU code **sm_87** in both (nvblox 4.6 was sm_75-only,
  so its 4-minute first-run JIT should be gone). Plugin class unchanged
  (`nvidia::isaac_ros::visual_slam::VisualSlamNode`, now in `libisaac_ros_cuvslam_node.so`).
- 2026-10-24: code made image-agnostic (works on 1.2 and 2.0): `ENVSET` and the watchdog,
  build scripts and nav2 launch source `/opt/ros/*/setup.bash` (+ `/opt/realsense_ws`);
  `vo_isaac.launch.py` picks `isaac_ros_cuvslam` or `isaac_ros_visual_slam`; camera
  `LD_LIBRARY_PATH` has `/opt/librealsense/lib` (2.0) before `/root/librealsense/install/lib` (1.2).
  Pi 5 (`phase1/teleop/run_teleop.sh`) and laptop (`phase2/rviz/rover_live.sh`, `./rover view`
  probe) stay Jazzy until those machines move.
- 2026-10-24: plan written; branch `isaac-ros-5` from `detector-trt` (efd3c28).
  `orin-nav:1.1` deleted by the owner earlier the same day; `lidar/build.sh` and
  `phase3/plugins/build.sh` defaulted to it → now 1.2 (a6f45f0). Laptop was off (no route to .9).
