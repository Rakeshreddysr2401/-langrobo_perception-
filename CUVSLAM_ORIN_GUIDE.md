# cuVSLAM + nvblox + Nav2 on Jetson Orin Nano (JetPack 7.2) — Complete Guide

**Date:** 2026-07-15
**Hardware:** Jetson Orin Nano 8GB · RealSense D555 (Ethernet/DDS) · ESP32 rover
**Host:** Ubuntu 24.04 · JetPack 7.2 (L4T r39.2) · ROS 2 Jazzy — *host untouched throughout*

**Result:** D555 → cuVSLAM 12.6 (GPU, ~20Hz odometry) → nvblox (color 3D mesh ~9Hz + costmaps)
→ Nav2 (path planning + `/cmd_vel` at 14Hz) — the full autonomy stack, all on the Orin Nano.

---

## 1. The Problem: cuVSLAM SIGILL on Orin

Installing `isaac_ros_visual_slam` from NVIDIA's JetPack 7 apt repo
(`isaac.download.nvidia.com/isaac-ros/release-4.4 noble-jetpack`) crashes instantly with
**SIGILL (illegal instruction)** on Orin Nano.

### Root cause (verified by disassembly)

- Isaac ROS 4.x officially supports **only Jetson Thor** on JetPack 7 (see the
  [support matrix](https://nvidia-isaac-ros.github.io/getting_started/index.html)).
- Every 4.x `libcuvslam.so` (4.3, 4.4, 4.5.0) is compiled `-march=armv9` with **SVE/SVE2
  instructions** (`whilelo`, z-register loads) auto-vectorized into ordinary code — no CPU
  fallback. `objdump -d libcuvslam.so | grep whilelo` → ~5000 hits.
- Orin Nano's Cortex-A78AE is **ARMv8.2 — no SVE**. Any code path SIGILLs.
- The Isaac ROS CLI has no Orin/Thor distinction — every Jetson gets the Thor binary.
  **No configuration can fix this.** It is not an installation error.

### The insight that solved it

**Isaac ROS 3.2.6** (ROS 2 Humble, Ubuntu 22.04 — the last Orin-supported release) ships an
Orin/NEON build of cuVSLAM 12.6: zero SVE instructions. It needs CUDA 12 user-space, and the
JetPack 7.2 driver (CUDA 13 era) runs CUDA 12 binaries fine (drivers are backward compatible).
So: run cuVSLAM 3.2.6 in a **Humble sidecar container**, everything else stays in the Jazzy
container. Verified before building: `dlopen` + `CUVSLAM_GetVersion` → 12.6, `cudaGetDeviceCount`
and `cublasCreate` (CUDA 12 pip libs) → success.

---

## 2. Architecture

```
┌───────────────────────────── Jetson Orin Nano (JP 7.2 host) ─────────────────────────────┐
│                                                                                          │
│  ┌──────── isaac_ros container (Ubuntu 24.04, Jazzy) ────────┐  ┌─ cuvslam sidecar ────┐ │
│  │                                                           │  │ (Ubuntu 22.04,Humble)│ │
│  │  realsense2_camera ──► infra1/infra2 ──────────────────── UDP ──► isaac_ros_        │ │
│  │   (DDS librealsense)   depth, color, motion(IMU)          │  │    visual_slam 3.2.6 │ │
│  │                                                           │  │    (cuVSLAM 12.6,    │ │
│  │  nvblox_node ◄── depth+color+TF ◄──────────────────────── UDP ──  Orin NEON build)  │ │
│  │   (GPU mesh, ESDF costmap slice, back-projected depth)    │  │    map→odom→base_link│ │
│  │                                                           │  └──────────────────────┘ │
│  │  Nav2 (planner+MPPI+collision monitor) ──► /cmd_vel ──► ESP32 rover                  │ │
│  │  rviz2 (GUI on Jetson display :1)                         │                           │ │
│  └───────────────────────────────────────────────────────────┘                           │
└──────────────────────────────────────────────────────────────────────────────────────────┘
        ▲ Ethernet (MTU 9000, 192.168.11.x)
   RealSense D555 (DDS network camera, factory IP 192.168.11.55)
```

TF tree: `map → odom` (cuVSLAM SLAM) `→ base_link` (cuVSLAM VO) `→ camera0_link` (static
mount, x=0.10 z=0.25) `→ camera0_*_optical_frame` (realsense driver).

---

## 3. Daily Operation (the only section you need day-to-day)

All scripts in `src/langrobo_perception/scripts/`, run from the Jetson **host**, in order:

```bash
./run_d555_stereo.sh        # 1. camera: stereo IR + depth + color + IMU, emitter off
./run_cuvslam_sidecar.sh    # 2. cuVSLAM localization       (wait ~20s)
./run_nvblox.sh             # 3. 3D reconstruction + costmaps
./run_nav2.sh               # 4. navigation                 (wait ~30s)
./run_vision_ai.sh          # 5. YOLO detections + VLM pixel grounding (Pi5-facing)
./run_rviz_cuvslam.sh       # 6. visualization on the Jetson display (skip on missions)
```

**Send a goal:** click **"2D Goal Pose"** in the RViz toolbar and click a point on the map, or:

```bash
docker exec isaac_ros bash -c 'unset ROS_DISCOVERY_SERVER; source /opt/ros/jazzy/setup.bash; \
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.5}, orientation: {w: 1.0}}}}"'
```

**What you see in RViz:** IR camera views · green odometry arrows + orange SLAM path (your
"vehicle line") · colored 3D mesh growing as the camera moves · costmap on the floor ·
green Nav2 plan line · yellow robot footprint.

**IMPORTANT — restart rules:**
- If the **camera** restarts, restart cuVSLAM *and* nvblox after it (stream gaps make visual
  odometry jump; nvblox keeps the corrupted map otherwise).
- If cuVSLAM restarts, restart nvblox (fresh origin, stale map).
- Robot pose far from origin + "Robot is out of bounds of the costmap" in nav2 log
  → this exact situation: restart cuVSLAM + nvblox.
- If the D555 stops being discovered (`rs-enumerate-devices` → "No device detected" while
  ping works): **physically power-cycle the camera** (unplug 5s). Its DDS stack goes stale
  after many rapid reconnects.

Health checks:

```bash
# odometry flowing? (~20Hz)
docker exec cuvslam bash -c 'source /opt/ros/humble/setup.bash; ros2 topic hz /visual_slam/tracking/odometry'
# robot near origin?
docker exec isaac_ros bash -c 'source /opt/ros/jazzy/setup.bash; ros2 run tf2_ros tf2_echo map base_link'
# nav2 up?
docker exec isaac_ros grep -a "Managed nodes are active" /tmp/nav2.log
# logs: /tmp/rs_infra.log  /tmp/nvblox.log  /tmp/nav2.log (in isaac_ros); docker logs cuvslam
```

---

## 4. How Each Piece Was Built (reference)

### 4.1 D555 camera (Jazzy container) — the D555 is NOT like USB RealSense cameras

| Quirk | Detail |
|---|---|
| Connection | Ethernet DDS, not USB. Needs DDS-enabled librealsense **built from source** (`/root/librealsense`, v2.58.2) + `realsense2_camera` 4.58.2 rebuilt against it |
| Library order | `LD_LIBRARY_PATH=/root/librealsense/install/lib:$LD_LIBRARY_PATH` must be set **AFTER** `source setup.bash` (setup.bash prepends the apt lib) |
| DDS config | `~/.realsense-config.json` = `{"context":{"dds":{"enabled":true,"domain":0}}}` — the `"context"` wrapper is required |
| Topics | `/camera/camera0/…` (namespace default changed in 4.58.2) |
| Native profile | **896x504x30** — not 848x480 like D435/D455 |
| IMU | Single combined stream: `enable_motion:=true` → `/camera/camera0/motion/sample` (200Hz `sensor_msgs/Imu`). No `enable_gyro`/`enable_accel`, no `unite_imu_method` |
| Pointcloud | `pointcloud.enable` **does not exist** on the DDS driver — generate externally (we use nvblox's back-projected depth) |
| Emitter | `depth_module.emitter_enabled:=0` — IR dot pattern corrupts cuVSLAM feature tracking. Depth is noisier without it (acceptable); emitter on/off toggling is the future refinement |
| Network | Jetson `enP8p1s0` at 192.168.11.70/24, **MTU 9000** both ends |

### 4.2 cuVSLAM sidecar (`docker/cuvslam-sidecar/`, image `cuvslam-sidecar:3.2.6`)

Rebuild if ever needed: `docker build -t cuvslam-sidecar:3.2.6 docker/cuvslam-sidecar/`

The Dockerfile encodes every lesson:
- Ubuntu 22.04 + ROS Humble + `ros-humble-isaac-ros-visual-slam` from
  `deb https://isaac.download.nvidia.com/isaac-ros/release-3 jammy release-3.0`
  (the `release-3.0` *component* carries the 3.2.6 debs)
- CUDA 12 user-space via pip (`nvidia-cublas-cu12` etc.) — works on the JP7.2 driver
- `libnvvpi3` + `cuda-nvtx-12-6` from the JetPack 6 repo (`repo.download.nvidia.com/jetson/common r36.4`)
- `libegl1 libgles2` (VPI needs EGL) · GXF plugin dirs added to `LD_LIBRARY_PATH` (entrypoint)
- `NVIDIA_VISIBLE_DEVICES=all` env — plain Ubuntu bases don't trigger the GPU mount without it

**Cross-container DDS gotcha (cost an hour):** discovery worked but no data flowed.
FastDDS uses shared memory between same-host participants; the containers have **private IPC
namespaces**, so SHM silently drops everything. Fix: the sidecar runs a **UDP-only FastDDS
profile** (`udp_only.xml`); the publisher side negotiates down to UDP automatically.
Both sides must leave `ROS_DISCOVERY_SERVER` **unset** (plain multicast).

**IMU fusion is OFF:** with default noise parameters the D555 IMU makes cuVSLAM's position
explode (~850m drift while stationary — verified). Visual-only is rock solid (0.000m drift).
To revisit: calibrate against `/camera/camera0/motion/imu_info` and verify motion-frame extrinsics.

### 4.3 nvblox (Jazzy container — the 4.4 GPU debs run fine on Orin; only cuVSLAM was Thor-built)

- Config: `nvblox_base.yaml` (from `nvblox_examples_bringup`) + our `config/nvblox_real.yaml`
  (`global_frame: odom`, 2D ESDF slice for costmaps) + `use_color:=true` for the colored mesh
- `/nvblox_node/mesh` (~9Hz, only publishes **when subscribed**) — RViz display class is
  `nvblox_rviz_plugin/NvbloxMesh`, NOT a MarkerArray
- `/nvblox_node/static_map_slice` feeds Nav2's costmap layers;
  `/nvblox_node/back_projected_depth/...` (5Hz) feeds the collision monitor

### 4.4 Nav2 (Jazzy container)

- `nav2_bringup navigation_launch.py` + `config/nav2_real.yaml`
- Costmaps: `nvblox::nav2::NvbloxCostmapLayer` on `/nvblox_node/static_map_slice` + inflation
- `bt_navigator` in `map` frame (Pi5 goal contract); costmaps in `odom` (NVIDIA reference wiring)
- MPPI controller, differential drive, speeds capped to the ESP32 envelope (0.30 m/s)
- Collision monitor gates `cmd_vel_smoothed → /cmd_vel`; source = nvblox back-projected depth
  (fixed from the old `/camera0/…` topic that no longer exists)
- Chain verified: goal → `/plan` → `/cmd_vel` 14Hz, 0.08 m/s toward goal

---

## 5. Verified Performance & What Costs What (Orin Nano 8GB, all simultaneous)

| Component | Rate |
|---|---|
| D555 stereo IR + depth + color | 30 / 28 / 24 Hz |
| cuVSLAM odometry | ~20 Hz light load, ~11 Hz with full stack+RViz (stationary drift 0.000m) |
| nvblox color mesh / ESDF slice | ~9 Hz / ~9 Hz |
| Nav2 `/cmd_vel` | ~14 Hz |
| YOLO detections / look() JPEG | 5 Hz / 2 Hz |
| RAM / GPU (everything running) | ~6.0 / 7.4 GB · GPU ~57% |

**Nothing on the host changed** — still Ubuntu 24.04 / JetPack 7.2 / CUDA-13 driver. The
sidecar's Ubuntu 22.04 is container userspace only. cuVSLAM (GPU, CUDA-12-on-13 — no
penalty) and nvblox (GPU, native CUDA 13) both use the iGPU. NITROS runs *inside* nodes;
camera→cuVSLAM crosses containers as plain DDS (the realsense driver isn't NITROS-enabled
even in NVIDIA's reference).

**Overhead ranking (what to trim when you need headroom):**
1. **RViz on the Jetson** — biggest hit; don't run it on missions.
2. **UDP loopback between containers** — causes the frame-delta warnings and the 20→11Hz
   odometry dip. Future fix: recreate `isaac_ros` with shareable IPC (`--ipc shareable`) and
   run the sidecar with `--ipc container:isaac_ros` → FastDDS shared memory, near-zero copy.
3. **Color stream + mesh coloring** — demo candy; disable for navigation-only.
4. Emitter-off depth — quality cost only, not speed.

Nav2 needs ~10Hz localization/costmaps for a 0.3 m/s rover — the stack meets that even
fully loaded. When adding heavier AI, apply #1 and #3 to free ~1-1.5GB RAM + GPU headroom.

## 6. Pi5 / VLM Integration — "go near the chair"

**Network topology (verified 2026-07-16):**

```
D555 camera ──ethernet 192.168.11.x──► Jetson (enP8p1s0: 192.168.11.70)
                                       Jetson (wlP1p1s0: 192.168.1.15)
Pi5 (wlan0: 192.168.1.16) ◄──WiFi 192.168.1.x, same SSID──┘
ESP32 rover ──micro-ROS──► Pi5 agent (~/microros_ws) ──► subscribes /cmd_vel
```

Jetson↔Pi5 talk over WiFi with `ROS_DOMAIN_ID=0` and default multicast discovery — no
discovery server, nothing to configure on either side (both must leave
`ROS_DISCOVERY_SERVER` unset). Verified end-to-end over this link: 2Hz JPEG feed, YOLO
detections, pixel grounding round-trip, NavigateToPose accept/cancel from the Pi5 with
`/cmd_vel` flowing at ~15Hz on the Jetson.

**The Pi5 client (`pi5/langrobo_client.py`)** is the single integration point — deploy it
with `scripts/deploy_pi5.sh` (lands at `~/langrobo/` on the Pi5, repo copy is the source of
truth). Your VLM script imports it:

```python
from langrobo_client import LangRoboClient
bot = LangRoboClient()
frame = bot.look("/tmp/frame.jpg")      # newest camera frame → give to the VLM
res = bot.ground_pixel(u, v)            # VLM's pixel → {"ok":True,"goal":{x,y,yaw}}
bot.go_to(**res["goal"])                # Nav2 action, blocks until arrival
# or one call:  bot.go_near_pixel(u, v)
# YOLO shortcut: bot.go_near_object("chair")   # no VLM needed for known objects
```

CLI for testing: `./langrobo_client.py status | look | objects | ground u v | go x y [yaw]
| go-pixel u v | go-object label | cancel`.

`run_vision_ai.sh` on the Jetson provides the two complementary paths behind it:

**Path A — YOLO (common objects, no VLM round-trip):** `detections_3d` runs YOLOv8n at 5Hz
and publishes map-frame object positions:

```
/vision/detections_3d   std_msgs/String  {"frame":"map","objects":[{"label":"chair","x":2.31,"y":-0.42,...}]}
```

The Pi5 brain matches the label, subtracts an approach offset, sends NavigateToPose. This
already worked with the old contract — same topic, same JSON.

**Path B — VLM pixel grounding (arbitrary language: "the red mug next to the laptop"):**

```
1. Pi5 grabs a frame:      /camera/color/image_raw/compressed   (2Hz JPEG)
2. VLM returns pixel (u,v) of the object in that image
3. Pi5 publishes:          /vision/pixel_query   geometry_msgs/PointStamped
                             point.x=u, point.y=v  (header.frame_id = request id)
4. pixel_to_goal answers:  /vision/pixel_result  std_msgs/String — ALWAYS, success or not:
                             {"id":"<req>","ok":true,"depth_m":3.1,
                              "object":{x,y,z},"goal":{x,y,yaw}}
                             {"id":"<req>","ok":false,"reason":"no_depth_at_pixel"}
                           /vision/pixel_goal    geometry_msgs/PoseStamped (map frame,
                             success only) — pulled back 0.6m, facing the object,
                             directly usable as a Nav2 goal
5. Pi5 forwards it:        bot.go_to(**res["goal"])  (NavigateToPose action)
```

Every query gets an answer matched by request id — `ok:false` carries the reason
(`no_depth_at_pixel`, `depth_out_of_range`, `tf_not_ready`, `camera_not_ready`, …). No
reply at all within ~2s means the query never arrived (transport, not grounding). The
client guards the classic trap here: a fresh ROS node must wait for DDS subscriber
matching before publishing, or the first message vanishes silently. Tunables on the
node: `approach_offset_m` (0.6), `max_range_m` (8.0), `depth_patch_px` (9).

**Depth on the Pi5?** Deliberately not streamed — raw depth is ~7MB/s even compressed and
WiFi would choke. The Pi5 gets metric 3D the light way: pixel queries answered with real
depth by the Jetson (Path B), or ready-made map-frame object positions (Path A). If a
future VLM genuinely needs dense depth, subscribe to
`/camera/camera0/depth/image_rect_raw/compressedDepth` sparingly — it works over the same
link, it's just heavy.

Image resolution for the VLM: 896x504. Remind the VLM to return pixel coordinates in that
space (or scale them back if you downscale the image before prompting).

## 7. Remaining Work

1. **ESP32 rover bring-up** — the last unverified link. Start the micro-ROS agent on the
   Pi5 (`~/microros_ws`), flash the ESP32 to subscribe `/cmd_vel` (plain
   `geometry_msgs/Twist`, capped 0.30 m/s); everything upstream already produces it.
   Wheels + servo GPIO 18/19 pending.
2. **VLM on the Pi5 (or Mac mini)** — wire the VLM's pixel output into
   `bot.go_near_pixel(u, v)`; the whole transport/grounding/navigation chain below it is
   verified. A Mac mini VLM can either talk to the Pi5 over HTTP or join ROS directly.
3. **Camera extrinsics** — measure real mount (`cam_x/y/z/pitch`, defaults in use)
4. **IMU calibration** — then retry `enable_imu_fusion`
5. **Emitter on/off toggling** — better depth without hurting tracking
6. **Map persistence** — cuVSLAM save/load for goals that survive reboots
7. **Shared-memory IPC between containers** — removes the UDP overhead (§5)
8. **Watch NVIDIA releases** — if Isaac ROS ships an Orin JetPack-7 build, the sidecar retires
