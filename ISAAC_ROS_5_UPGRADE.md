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

## Part B: the new 5.0 features we adopt (owner, 2026-10-24: "use all the useful ones")

Starts after step 6 (2.0 works on the twin). Same rules: one feature at a time, twin first,
measured against the twin's TRUE pose / objects, then the real robot standing still, then
driven only with the owner's OK. Each feature is a switch in `./rover` (off = today's behaviour),
so nothing replaces working code until its numbers beat it.

### Images

```
orin-nav:2.0   core: cuVSLAM, nvblox, nav2, D555, YOLO-TRT, GPU partitioning   (steps 1-9)
orin-nav:2.1   + Part B packages (one Dockerfile layer per feature group)
orin-arm:2.1   + arm (MoveIt, cuMotion, FoundationPose, SAM2, pick-and-place)  (B7-B8)
```
Model files (TensorRT engines, ONNX, cuVGL maps) live on the host in `~/rover/models/isaac/`,
mounted read-only — never baked into an image, so rebuilds don't re-download GBs.

### The budget rule (Orin Nano: 6 CPU cores, 7.4 GB RAM shared with the GPU)

Not everything runs at once. The rover works in **modes**; heavy models run on demand:

| mode | always on | on demand |
|---|---|---|
| **start-up** | camera, lidar | cuVGL / occupancy-grid localizer → once, until the pose is found, then off |
| **drive** | cuVSLAM, nvblox (+ people), nav2, YOLO-TRT at 5 Hz, tracker | Grounding DINO: once per "find X", then YOLO + our lock-and-track follows it |
| **arm** (rover parked, nav2 paused) | cuVSLAM, camera | SAM2 → FoundationPose → cuMotion; FoundationStereo/ESS depth for the grasp |

`isaac_ros_gpu_partitioning` gives cuVSLAM a fixed GPU share in every mode (the 2026-07-19
lesson: YOLO starving the tracker blew up the pose). Every feature's test records CPU, RAM,
GPU and cuVSLAM rate before/after; a feature that drops cuVSLAM below 25 Hz doesn't ship.

### Features, in order

**Priority (owner, 2026-10-24): navigation first.** B1–B5 (GPU sharing, localization, cuVGL,
nav2 improvements, people) are the focus until nav works well on the real robot. B6 (Grounding
DINO) follows. B7–B8 (arm) stay planned, not started, until the owner starts the arm; step 7
(arm in the twin) moves after Part B nav.

| # | feature | package(s) | replaces / improves | pass test (twin, then real) |
|---|---|---|---|---|
| B1 | GPU partitioning | `isaac_ros_gpu_partitioning` (needs `nvidia-cuda-mps-control` ≥ 13010; check MPS works on Orin) | protects cuVSLAM from YOLO/DNN load | YOLO + Grounding DINO under load: cuVSLAM ≥ 25 Hz, 0 pose jumps |
| B2 | startup localization (lidar) | `isaac_ros_occupancy_grid_localizer` | manual initial pose; `SLAM_FRESH=1` on the twin | from 10 random start poses in a saved map: found pose within 0.15 m / 5° |
| B3 | visual mapping + global localization | `isaac_ros_visual_mapping`, `isaac_mapping_ros`, `isaac_ros_visual_global_localization` (cuVGL) | the "saved room locks onto the wrong place" problem | map from one recorded drive; relocalize from 10 starts + 1 "kidnap": within 0.15 m; cuVSLAM resumes in the map |
| B4 | nvblox → nav2 official path | `nvblox_nav2` | part of our `fused_obstacles` | side by side on the same bag/scenario: same or fewer missed obstacles, no new ghost walls |
| B5 | people in the map | nvblox people segmentation (`isaac_ros_peoplesemseg_models_install`, Segformer/U-Net) | people leave ghost obstacles | twin with walking people: no permanent ghost cells; budget held |
| B6 | open-vocabulary "find X" | `isaac_ros_grounding_dino` | YOLO's fixed 80 classes for the brain | twin objects by name (incl. non-COCO): found ≥ 9/10, latency measured; hands off to target_lock |
| B7 | DNN depth for grasping | `isaac_ros_ess` (light) / `isaac_ros_foundationstereo` (heavy) | D555 depth on shiny/dark/small objects | depth error vs twin truth on test objects; FoundationStereo only if Orin rate is usable |
| B8 | pick-and-place | `isaac_ros_segment_anything2`, `isaac_ros_foundationpose`, cuMotion + MoveIt, `isaac_ros_manipulation_pick_and_place` | (new) | twin: pose error, grasp success ≥ 8/10 on the test objects; then the real arm |
| B9 | watch only | COMPASS / X-Mobility (`isaac_ros_deploy_*`): learned nav policy, beta, gated models | — | a twin experiment later; never replaces nav2 without the owner's call |

Not adopted: VDA5050 / mission client (warehouse fleets).

### Notes for B2 / B3 (from NVIDIA's release-5.0 docs, 2026-10-24)

- **cuVGL node** (`isaac_ros_visual_global_localization`): in = grayscale stereo
  `visual_localization/image_{0,1}` + `camera_info_{0,1}` (our D555 IR pair, as cuVSLAM gets);
  trigger = topic `visual_localization/trigger_localization` or the Trigger service of the same name;
  out = `visual_localization/pose` (PoseWithCovarianceStamped), optional `vmap → base_link` TF
  (`publish_map_to_base_tf` — **off for us**: fusion2 / vo_isaac own the TF tree; we feed the pose in).
  Needs `map_dir`, `config_dir` (`keypoint_creation_config.pb.txt`, `localizer_config.pb.txt`).
- **Map creation is offline** from a rosbag (`create_map_offline.py`, Isaac Perceptor tutorial):
  makes the cuVGL map, a cuVSLAM map and an occupancy grid in one go. NVIDIA's converter steps are
  written for x86 → plan for the laptop (RTX 4050) to build maps from bags the Jetson records.
  Unknown until tried: whether the Jetson can do it, how long a room takes.
- **Occupancy grid localizer**: lidar scan + a known occupancy map → pose for nav2. 5.0 assets exist on
  NGC; exact topics/params to be read from the installed package (docs page moved). A 2025 forum
  report says results were slow → measure time-to-pose.
- Both run **once at start-up / on request**, then stop (budget rule).

### Risks to check early

- Grounding DINO, FoundationStereo and FoundationPose are big models; their Orin **Nano**
  rate is unknown (NVIDIA's numbers are mostly AGX Orin / Thor). B6–B8 start with a 10-minute
  rate + RAM check before any integration work.
- cuVGL forum reports (2025) of trouble building maps from rosbags — may be fixed in 5.0; B3 starts small.
- cuMotion MoveIt on Lyrical: 5.0 known issue (vendor packages not certified).

## Rollback

Until step 9: `IMAGE=orin-nav:1.2 ./rover up` on the old branch. The host is not changed
by this plan (JetPack 7.2.1 stays; the host `isaac-ros-cli` 2.5 stays dormant).

## Log

- **2026-10-25 05:xx — step 9 done + Part B started.** `orin-nav:1.2` deleted (owner) → 75 GB free;
  all 4.6/Jazzy compatibility code removed (one Lyrical path; checks re-run green, see commit
  920041f). `./rover engine` re-done for 2.0 (ultralytics/torch/onnx in a 5.7 GB host cache; tested).
  **`orin-nav:2.1`** = `fa4d12545a6c` (56.3 GB; +13 GB is torch 2.12.1+cu132 from an Isaac pip shim)
  from `docker/2.0/Dockerfile.features`: grid localizer, cuVGL, visual mapping, isaac_mapping_ros,
  nvblox_nav2, segformer + PeopleSemSeg. check.sh 0 FAIL, nav2_check PASS, 83/83 imports.
  **B2 first result (`docker/2.0/b2_gridloc.sh`, live scan, maps/last = 3.5 m room, robot in a corner):**
  ours `relocalize.py` locked, 99.4 % of beams on walls, 1.6-9 s; NVIDIA 0.3-0.5 s but wrong and
  unstable (10-27 % of beams on walls, either frame reading; also with invalid_range_threshold 0.25).
  → keep ours; NVIDIA's gets the twin test with truth (10 starts) before any further thought.
  **B3 ready to record:** no existing bag has BOTH IR cameras (floor_clear: infra1 only). The whole
  mapping toolchain (create_map_offline.py, cuSFM, cuVGL map) is installed for aarch64 in 2.1, so
  maps can be built ON the Jetson. `phase2/tools/record_cuvgl.sh <name>` records the drive
  (IR pair + infos + tf + odom + scan, ~27 MB/s); `phase2/config/cuvgl_d555.yaml` maps our topics
  (vslam_*_optical frames). Needs: daylight, motor power, owner teleop.
  B1 needs host sudo (MPS). B4/B5 next (B4: replay floor_clear depth into nvblox_nav2 vs our costmap).

- **2026-10-25 04:00 — step 8 (real robot, STANDING STILL, motor supply removed by the owner) on 2.0:**
  camera IR 30.0 / depth 30.1 Hz (DDS librealsense 2.58.2 source build) · lidar 10.0 Hz, 720 beams ·
  gyro 200 Hz · lidar odom 10.1 Hz · fused /odom 20.0 Hz, `pose ok — LiDAR-anchored, sd 0.83 cm` ·
  slam_toolbox 0.5 Hz · nvblox grid 4.9 Hz, depth-to-map 28.6 Hz (no JIT wait) · nav2 active,
  0 errors, global/local costmap 0.7/1.7 Hz · `plan_check.py` (planner only): 3/3 reachable goals
  planned, 0 body overlaps, median 451 ms · YOLO live on the image's TensorRT 10.16 · ESP32
  /wheel_state 20.1 Hz, /cmd_vel 2 subscribers · Pi 5 (Jazzy) reads Jetson (Lyrical) /odom at 20.0 Hz;
  Pi's Jazzy NavigateToPose client finds the Lyrical server (goal part identical; no goal sent).
  RAM 5.5/7.4 GB, CPU ~60 %.
  **cuVSLAM "lost"** — NOT the upgrade: 4 AM, dark room, emitter off by design → IR frames mean 18/255,
  std 1, 0 corners. A 20 s recording replayed into BOTH images (`docker/2.0/vo_replay.sh`) gave the
  same wander, reset and PoseGraph failure on 4.6 and 5.0. Re-check VO in light.
  Lyrical fixes found live: `get_logger().warn()` removed in Lyrical rclpy → `.warning()` (5 files;
  vo_isaac.py crashed in its z-recovery handler); `ros2 bag record` needs `--topics` (our harness
  already does). Pi brain default nav backend = `reach` (topic); its direct-nav2 fallback is the only
  cross-distro action use.

- **2026-10-24 night — steps 1-4 DONE on the Jetson (no hardware, no domain 0).**
  `orin-nav:2.0` = `21e7b7e2b28c`, 43.4 GB unpacked (1.2: 73.4 GB), from `docker/2.0/Dockerfile`.
  Build fixes on the way: colcon/rosdep need packages.ros.org; realsense-ros 4.58.2 needs only
  `lyrical` added to its distro switch (NVIDIA's 4.56.3 patch doesn't apply); Isaac pip-shim
  packages need `/etc/isaac-ros-cli/environment.conf` = docker-activated.

  | check (script) | orin-nav:1.2 (4.6, Jazzy) | orin-nav:2.0 (5.0, Lyrical) |
  |---|---|---|
  | image checks (`check.sh`) | — | **0 FAIL** (packages, plugins, ldd, sm_87, DDS librealsense 2.58.2, TensorRT 10.16 in-image) |
  | cuVSLAM, fake stereo (`vo_check.sh`) | 30.0 Hz, −2.4 %, 21 % core, 583 MB | **30.0 Hz, +2.4 %**, 21 % core, 580 MB, 479 landmarks |
  | nvblox, fake depth (`nvblox_check.sh`) | first slice after ~240 s (sm_75 JIT) | **first slice 1.9 s**, 4.9 Hz, 17 % core, 958 MB, 0 errors |
  | our nav2 offline (`nav2_check.sh`) | PASS | **PASS** after 2 fixes (below) |
  | YOLO TensorRT engine | host TRT via /hosttrt, 11.6 ms | **image's own TRT 10.16, 12.5 ms**, 724 MB; no /hosttrt |
  | sllidar_ros2 / costmap plugin build | OK | **OK** (per-distro `build_lyrical/`, `install_lyrical/`) |
  | all 94 rover python modules import | — | **94/94** |
  | Jazzy ↔ Lyrical topics (13 msg types, both ways, UDPv4) | — | **13/13 both ways** |

  Code changes for nav2 1.5 (both images keep working): MPPI motion model is a plugin
  (`DiffDrive.plugin: mppi::DiffDriveMotionModel` in nav2.yaml); BT `IsPathValid` → `ValidatePath`
  (Lyrical copy written by nav2.launch.py); costmap plugin uses `declare_or_get_parameter` on 1.5.
  **nav2 action definitions differ between Jazzy and Lyrical** (NavigateToPose gained error codes,
  feedback fields): fine while every action caller runs on the Jetson (reach_node, navto,
  plan_check — they do; goals cross machines as the `/reach/goal` topic). To check on the Pi 5:
  that its brain never calls nav2 actions directly; RViz's nav2 panel (actions) would need Lyrical.
  Found: `phase2/tools/relocalize.py` already does lidar global localization (1-3 s, ~2 cm) —
  **B2 must beat it**; cuVGL (B3) could break its "ambiguous" verdict in symmetric rooms.
  B1 note: MPS daemon runs on the HOST with sudo (`nvidia-cuda-mps-control -d`, pipe /tmp/nvidia-mps).
  **Disk: 6.5 GB free** after the build (2.0 + 1.2 + 56 GB of bags in logs/bags) — owner decides.
  Next: step 5/6 (laptop is off), then step 8 (real robot, owner).

- 2026-10-24: **test tools** in the repo (were scratch files in 4.6): `docker/2.0/check.sh [image]`
  (packages, plugins, ldd, sm_87, DDS librealsense, TensorRT, python deps; PASS/FAIL),
  `docker/2.0/vo_check.sh [image]` (cuVSLAM on `phase1/harness/fake_stereo.py`, domain 99,
  `--network none`). **Baseline orin-nav:1.2 (4.6):** 30.0 Hz, 2.047 m tracked vs 2.097 m
  truth (−2.4 %), tracker 21 % of a core / 583 MB, 0 resets, 233 landmarks. PASS.

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
