# Issues Faced & Solved — cuVSLAM/nvblox/Nav2 on Jetson Orin Nano (JetPack 7.2)

**Date:** 2026-07-15 · Companion to `CUVSLAM_ORIN_GUIDE.md` (architecture + daily operation)

This file is the honest engineering log: every problem we hit, why it happened, and how it
was solved — then what you now have and how to use it.

---

## Part 1 — The Big One: cuVSLAM crashed on Orin (SIGILL)

**Symptom:** `isaac_ros_visual_slam` from NVIDIA's JetPack 7 apt repo died instantly with
*illegal instruction* on every attempt (releases 4.3, 4.4, fresh installs — all the same).

**Investigation:** Disassembled `libcuvslam.so` with `objdump`. Found ~5000 **SVE/SVE2
instructions** (`whilelo`, z-register loads) compiled into ordinary code paths, no CPU
fallback, literal "ARMv9" string in the binary. Checked NVIDIA's official support matrix:
**Isaac ROS 4.x supports only Jetson Thor on JetPack 7.** Orin Nano's Cortex-A78AE is ARMv8.2
— it physically cannot execute SVE. Checked the brand-new 4.5.0 (July 6, 2026): byte-identical
cuVSLAM binary. Read the Isaac ROS CLI source: no Orin/Thor branching — every Jetson gets the
Thor binary.

**Conclusion:** Not a configuration error, not an installation error. NVIDIA doesn't ship an
Orin cuVSLAM for JetPack 7. **No settings change could ever fix it.**

**Solution — the Humble sidecar:** Isaac ROS **3.2.6** (last Orin-supported release,
ROS 2 Humble/Ubuntu 22.04) ships cuVSLAM 12.6 compiled for Orin (zero SVE — verified).
We proved it loads and runs on the JP7.2 host *before* building anything (dlopen test +
CUDA 12 compatibility test), then built a dedicated container `cuvslam-sidecar:3.2.6` that
runs just cuVSLAM and talks to the main Jazzy container over DDS. **Host OS/JetPack untouched.**

### Sub-issues while building the sidecar

| Issue | Cause | Fix |
|---|---|---|
| CUDA 12 needed, host has CUDA 13 driver | 3.2.6 links cublas12/cusolver11 | pip `nvidia-*-cu12` libs — drivers are backward compatible (verified: `cudaGetDeviceCount` + `cublasCreate` OK) |
| `libgxf_serialization.so` not found | GXF plugins live under `share/isaac_ros_gxf/gxf/lib/*`, not on the loader path | entrypoint adds all GXF dirs to `LD_LIBRARY_PATH` |
| `libnvToolsExt.so.1` not found | pip nvtx wheel ships only `libnvtx3interop` | `cuda-nvtx-12-6` from the JetPack 6 apt repo + symlink |
| `libnvvpi.so.3` not found | VPI3 is a JetPack 6 component | `libnvvpi3` from `repo.download.nvidia.com/jetson/common r36.4` |
| `libEGL.so.1` not found | VPI needs EGL, plain Ubuntu base has none | `apt install libegl1 libgles2` |
| `libcuda.so.1` not found at runtime | plain Ubuntu images don't trigger the GPU mount | `ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all` |

## Part 2 — Cross-container DDS: discovery worked, data didn't

**Symptom:** the sidecar saw all camera topics (`ros2 topic list` fine) but received **zero
messages**. Everything looked connected; nothing flowed.

**Cause:** FastDDS uses **shared memory** transport between participants on the same host.
The two containers have private IPC namespaces → SHM segments are invisible across the
boundary → data silently vanishes. (Joining the IPC namespace failed too: the main container
was created non-shareable.)

**Fix:** UDP-only FastDDS XML profile (`udp_only.xml`) in the sidecar. The publisher
negotiates down to UDP automatically — only the subscriber side needed the profile.
Both sides must leave `ROS_DISCOVERY_SERVER` unset (plain multicast).

## Part 3 — RealSense D555 quirks (it is NOT a normal RealSense)

| Issue | What we learned |
|---|---|
| Not detected at all | Needs **DDS-enabled librealsense built from source** + `realsense2_camera` 4.58.2 rebuilt against it; apt versions have zero DDS support |
| Worked, then randomly stopped | `setup.bash` prepends the apt librealsense — `LD_LIBRARY_PATH` must be set **AFTER** sourcing |
| DDS config silently ignored | `~/.realsense-config.json` needs the `{"context":{...}}` wrapper |
| Topics "missing" | 4.58.2 moved everything to `/camera/camera0/…` |
| `848x480x30 is invalid` | D555 native profile is **896x504x30** — different sensor than D435/D455 |
| No gyro/accel topics | D555 publishes ONE combined IMU stream: `enable_motion:=true` → `/camera/camera0/motion/sample` (200Hz). `enable_gyro`/`unite_imu_method` don't apply |
| No pointcloud topic ever appears | `pointcloud.enable` **is not declared** on the DDS driver — SDK processing filters don't exist for network cameras. Solved with nvblox's GPU back-projected depth instead |
| Camera vanishes after many restarts | The D555's onboard DDS stack goes stale — ping works, discovery doesn't. **Physical power cycle** (unplug 5s) is the only fix |
| IR emitter vs tracking | Emitter dot-pattern corrupts cuVSLAM features → emitter OFF (`depth_module.emitter_enabled:=0`); depth gets noisier — acceptable trade |

## Part 4 — Making the stack correct

| Issue | Cause | Fix |
|---|---|---|
| **IMU fusion exploded** (~850m position drift while stationary) | default cuVSLAM noise params don't match the D555 IMU | fusion **disabled**; visual-only shows 0.000m stationary drift. Revisit after calibrating against `motion/imu_info` |
| **TF tree broken** (`base_link` orphaned, nvblox couldn't clear map) | `camera0_link` had two parents: cuVSLAM's odom and the static mount TF | cuVSLAM `base_frame:=base_link` → clean chain `map→odom→base_link→camera0_link→optical` |
| **Nav2 goal ABORTED**: "Start Coordinates outside bounds" | visual odometry jumps when the camera stream restarts → robot pose ends up 50-800m from the costmap | **restart rule**: camera restart ⇒ restart cuVSLAM + nvblox. One command resets everything to origin |
| **`/cmd_vel` blocked** | collision monitor watched a topic that no longer exists (`/camera0/…` old namespace + no pointcloud on D555) | source switched to `/nvblox_node/back_projected_depth/…` (5Hz, free — GPU already computes it) |
| depth_image_proc pointcloud attempt failed | camera_info QoS incompatibility, overrides not honored | abandoned — nvblox source was better anyway |
| **RViz: no 3D mesh visible** | (a) mesh topic is custom type `nvblox_msgs/Mesh` needing the `nvblox_rviz_plugin/NvbloxMesh` display, (b) color was off = gray-on-gray, (c) camera too close (<0.5m = no valid depth) | correct plugin class + `use_color:=true` + RGB stream on + "sweep the room slowly at 2-4m" |
| Nav2 wouldn't die for restart | `pkill -f nav2` matches its own shell → kills itself (exit 137) | kill by process name (`pkill -9 bt_navig` …) — baked into `run_nav2.sh` |

---

## Part 5 — What You Have Now

A complete, verified autonomous-navigation perception stack, all on the Orin Nano 8GB:

```
RealSense D555 (Ethernet) ──► cuVSLAM 12.6 GPU localization (~11-20Hz odometry)
                          ──► nvblox GPU color 3D mesh + costmaps (~8-9Hz)
                          ──► Nav2 path planning + MPPI control ──► /cmd_vel (14Hz)
                          ──► RViz: camera views, trajectory, 3D world, plans, footprint
```

- **cuVSLAM on Orin Nano under JetPack 7.2** — which NVIDIA does not officially support —
  via the 3.2.6 Humble sidecar. Host stays clean Ubuntu 24/JP7.2.
- Measured on-target: stationary drift 0.000m, full stack ~6GB RAM / 57% GPU — it fits.
- Everything scripted and restart-safe, every config file in the repo, image rebuildable
  from `docker/cuvslam-sidecar/Dockerfile`.

## Part 6 — How To Use It

**Start (5 commands, in order, from the host):**

```bash
cd ~/workspaces/isaac_ros-dev/src/langrobo_perception/scripts
./run_d555_stereo.sh          # camera
./run_cuvslam_sidecar.sh      # localization      (wait ~20s)
./run_nvblox.sh               # 3D map + costmaps
./run_nav2.sh                 # navigation        (wait ~30s)
./run_rviz_cuvslam.sh         # view on the Jetson display
```

**Drive it:**
1. Sweep the camera slowly around the room → colored 3D world builds in RViz.
2. Click **2D Goal Pose** in the RViz toolbar, click a floor point → green path appears,
   `/cmd_vel` starts commanding toward it.
3. Yellow footprint = your vehicle. Orange line = where it has been. Green = where Nav2
   wants it to go.

**When something looks wrong:** pose far from origin / goal aborted →
`./run_cuvslam_sidecar.sh && sleep 20 && ./run_nvblox.sh` (fresh origin, clean map).
Camera not detected → power-cycle the D555. Full health-check commands are in
`CUVSLAM_ORIN_GUIDE.md` §3.

**To make the rover physically move:** connect the ESP32 base and subscribe it to `/cmd_vel`
(plain `geometry_msgs/Twist`, capped at 0.30 m/s in `nav2_real.yaml`). Everything upstream
is already producing it.

**AI / "go near the chair" (added same day):** `./run_vision_ai.sh` starts the Pi5-facing
layer — YOLO map-frame detections on `/vision/detections_3d`, the VLM look() JPEG feed at
2Hz, and the new `pixel_to_goal` node: the VLM points at a pixel on `/vision/pixel_query`
and gets back a ready-to-send Nav2 goal on `/vision/pixel_goal` (depth-deprojected, map
frame, 0.6m approach offset, facing the object). Verified end-to-end on-device. Full
contract: guide §6. Also fixed on the way: `detections_3d` still used the pre-4.58.2 topic
namespace (`/camera0/…`) — it would have silently seen no camera.

Remaining work list: guide §7.
