# How LangRobo Navigates — full pipeline

*From a depth-camera frame to a motor command, and how a "go there" request
becomes motion without driving into anything. Written 2026-07-18 against the
live, verified real stack (D555 only, no rover attached yet).*

---

## 0. TL;DR — can it "go to a place" if I ask?

**The software can.** Nav2 is live: give it a goal (a coordinate, an object,
or a Telegram phrase) and it localizes, builds a 3D map, plans a collision-free
path, and emits motor commands on `/cmd_vel`. This was verified with a live
test goal — the controller produced real velocity commands and rotated to avoid
the obstacle in front instead of driving into it.

**What is NOT yet true:** nothing physically moves, because the ESP32 rover is
not connected. `/cmd_vel` currently has **0 subscribers** — the commands are
computed and published, but there is no motor listening. To actually drive:

1. Power the ESP32 rover (it runs micro-ROS and subscribes `/cmd_vel`).
2. Confirm firmware flash state (`ping rover-esp32.local`).
3. Supervised low-speed first mission.

So: **"capable" = yes at the planning/command level, verified; "will move" =
only once the rover is powered.**

---

## 1. The pipeline at a glance

```
 D555 depth camera
   │  infra1 (gray)         color (RGB)        depth (mm)        IMU
   ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │ LOCALIZATION  — RTAB-Map (CPU, same container as the driver)      │
 │   rgbd_odometry: infra1+depth → /odom  (+ odom→base_link TF)      │
 │   rtabmap:       infra1+depth → map→odom TF, /map, persistent DB  │
 └─────────────────────────────────────────────────────────────────┘
   │  "where am I" = map→odom→base_link  +  persistent map (/data/rtabmap.db)
   ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │ MAPPING  — nvblox (GPU)                                           │
 │   depth+color+pose → 3D mesh, ESDF, 2D costmap slice,             │
 │                      back-projected obstacle point cloud          │
 └─────────────────────────────────────────────────────────────────┘
   │  "what is around me" = /nvblox_node/static_map_slice (+ mesh)
   ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │ NAVIGATION  — Nav2                                                │
 │   goal + pose + costmap →  global plan (NavFn)                    │
 │                         →  local control (MPPI)                   │
 │                         →  velocity smoother                      │
 │                         →  collision monitor (SlowZone/SafeZone)  │
 │                         →  dead-zone compensation                 │
 └─────────────────────────────────────────────────────────────────┘
   │  /cmd_vel  (geometry_msgs/Twist)
   ▼
 ESP32 rover (micro-ROS) → firmware PWM map → L298N → 2 gearmotors
```

Everything runs on the Jetson Orin Nano inside the `isaac_ros` container on
ROS 2 Jazzy, plain multicast DDS, `ROS_DOMAIN_ID=0`, wall-clock time (no
`use_sim_time`). The Pi5 "brain" and the Mac-mini VLM sit on the same DDS
network for the natural-language layer (§7).

---

## 2. What the depth camera actually provides

The RealSense **D555** is an Ethernet/DDS camera at 192.168.11.55 on the
jumbo-frame (`mtu 9000`) link. Started by `scripts/run_d555_stereo.sh`. Native
896×504. Two settings are enforced at runtime **every** start because the DDS
driver silently drops them as launch args:

- **IR emitter OFF** — the projected dot pattern would corrupt feature tracking
  on the infra image.
- **per-stream global-time OFF** — its per-stream drift model desynchronises
  the stamps; we want raw device stamps that track the host clock.

| Stream | Topic | Format | Rate | Used by |
|---|---|---|---|---|
| Infra1 (left IR, grayscale) | `/camera/camera0/infra1/image_rect_raw` | mono8, 896×504 | ~30 Hz | **RTAB-Map** (as the "rgb"/tracking image) |
| Depth | `/camera/camera0/depth/image_rect_raw` | 16UC1, mm, 0=invalid | 13–30 Hz* | RTAB-Map, **nvblox** |
| Color | `/camera/camera0/color/image_raw` | RGB8, 896×504 | ~30 Hz | **nvblox** (mesh color), **YOLO**, look() feed |
| IMU | `/camera/camera0/motion/sample` | sensor_msgs/Imu | ~200 Hz | (Phase 2: fuse into odometry) |

\* depth rate is CPU-load sensitive; it clears the 10 Hz floor with headroom
on a headless run (see REQUIREMENTS.md "depth clears the floor").

**Why infra1 and not color for tracking:** the D555 computes depth in the
infra1 viewpoint, so infra1 + raw depth are pixel-registered by construction
(same frame, same 896×504 grid). RTAB-Map takes the mono8 image fine, and this
avoids the color↔depth reprojection. (`align_depth` and `pointcloud` are
accepted params but publish nothing on this network driver.)

**Depth is the key metric input.** Each depth pixel is a distance in mm. That
is what becomes 3D obstacles: nvblox back-projects every depth pixel through the
camera intrinsics into a 3D point, fuses it into a truncated-signed-distance
volume, and slices it into the 2D costmap that Nav2 plans on.

**TF** (`scripts/run_robot_tf.sh` + the driver): `base_link → camera0_link`
(static, currently placeholder x=0.10 z=0.25 — **must be measured** on the real
chassis) then `camera0_link → {infra1,depth,color,motion}_optical_frame` from
the driver.

---

## 3. Localization — "where am I" (RTAB-Map)

`scripts/run_localization.sh` (backend switch: `config/localization`). Two nodes
in the camera's own Jazzy container (no cross-distro DDS):

- **`rgbd_odometry`** — infra1 + depth → `/odom` and the `odom → base_link` TF.
  Visual odometry at ~3–8 Hz, quality 250–310 tracked features per frame.
  `Reg/Force3DoF=true` keeps poses planar; `Odom/ResetCountdown=1` auto-recovers
  after a tracking loss.
- **`rtabmap`** — infra1 + depth → the `map → odom` TF, the `/map` occupancy
  grid, and a **persistent SQLite map** at `/data/rtabmap.db`. Loop closure
  relocalises the robot against the saved map, so **named/known locations
  survive reboots** — this is what makes "go back to the kitchen" possible.

Output contract for everything downstream: the full `map → odom → base_link`
transform chain (so any pose can be expressed in the fixed `map` frame) plus the
persistent map. The DB is integrity-checked on every start and a corrupt one is
archived rather than crash-looping.

> **Why RTAB-Map and not cuVSLAM:** NVIDIA ships cuVSLAM only as Thor-class
> binaries for JetPack 7 — it SIGILLs instantly on the Orin Nano's Cortex-A78AE.
> RTAB-Map is the verified-under-motion tracker on this hardware. The backend is
> pluggable for the day an Orin-compatible cuVSLAM exists. See
> `CUVSLAM_ORIN_GUIDE.md`.

---

## 4. Mapping — "what's around me" (nvblox)

`scripts/run_nvblox.sh`. GPU 3D reconstruction fed by depth + color + the
RTAB-Map pose. It produces, all in the `map` frame:

| Output | Topic | Consumer |
|---|---|---|
| 3D mesh (visual) | `/nvblox_node/mesh` | RViz |
| **2D costmap slice** | `/nvblox_node/static_map_slice` | **Nav2 costmaps** |
| Back-projected obstacle cloud | `/nvblox_node/back_projected_depth/...` | **collision monitor** |
| ESDF / distance field | `/nvblox_node/*_esdf_pointcloud` | (internal) |

nvblox integrates depth into a Euclidean Signed Distance Field on the GPU and
slices it at the robot's height into a 2D occupancy grid. That slice is the
obstacle picture Nav2 plans against — walls, furniture, anything the depth
camera has seen becomes a lethal/inflated cell.

---

## 5. Navigation — goal → path → motor command (Nav2)

`scripts/run_nav2.sh`. A behavior-tree-driven stack; the pieces, in order:

1. **Behavior Tree** (`bt_navigator`) — orchestrates the mission. Uses a
   **custom tree** `config/bt_navigate_to_pose_no_blind_backup.xml`: the default
   recovery `BackUp` (blind reverse) is **removed** — the forward-only D555
   cannot see behind, so blind reversing was a crash source. Recovery is
   spin + wait only.
2. **Global planner** (`NavfnPlanner`) — computes a full path over the global
   costmap. `allow_unknown: false` + `track_unknown_space: True`: it will **only
   route through space the camera has actually observed as free** — it refuses
   to plan a path into the unknown. This is the single biggest anti-crash rule.
3. **Local controller** (`MPPIController`, 20 Hz) — samples many candidate
   trajectories each tick and picks the best one that follows the plan and
   avoids costmap obstacles (`consider_footprint: True`). Differential-drive
   limits: `vx_max 0.30`, `vx_min -0.10` (slow, sighted reverse only),
   `wz_max 1.0`. Goal tolerance 0.10 m.
4. **Costmaps** (local + global) — footprint `[±0.15, ±0.13] m` (placeholder,
   measure the real chassis), 0.05 m resolution, layers = `nvblox_layer`
   (obstacles from `static_map_slice`) + `inflation_layer` (0.35 m buffer that
   pushes paths away from walls).
5. **Velocity smoother** — rate-limits accel/jerk so commands are physically
   achievable (`max_velocity [0.25, 0, 1.2]`). → `/cmd_vel_smoothed`.
6. **Collision monitor** — the independent last-resort safety layer (§6).
   → `/cmd_vel_nav`.
7. **Dead-zone compensation** (`scripts/cmd_vel_deadband.py`) — → `/cmd_vel`
   (§6).

### The command chain (verified live)

```
MPPI controller ─► velocity_smoother ─► /cmd_vel_smoothed
                                          │
                                          ▼
                         collision_monitor (SlowZone / SafeZone)
                                          │
                                          ▼  /cmd_vel_nav
                                  cmd_vel_deadband.py
                                          │
                                          ▼  /cmd_vel   ← ESP32 rover subscribes
                                             (currently 0 subscribers: no rover)
```

---

## 6. How it navigates "without a block" — the anti-crash design

Five independent guards, so a failure of any one still leaves the rover safe.
These fix the real wall-hitting incidents documented in REQUIREMENTS.md.

1. **Plan only through known-free space** — `allow_unknown: false` +
   `track_unknown_space: True`. The forward-only camera means everything behind
   and beside it is "unknown"; the planner treats unknown as *not* free, so it
   never plans a path into un-seen space.
2. **Inflated costmap** — a 0.35 m inflation buffer around every obstacle keeps
   normal paths well clear of walls; MPPI considers the actual footprint.
3. **Collision monitor, two rings** — an independent node watching the nvblox
   obstacle cloud, downstream of the planner:
   - **SlowZone** (0.45 m circle) → scale velocity to 40 %.
   - **SafeZone** (0.25 m circle) → hard stop.
   It runs even if the planner/controller misbehaves. Its `source_timeout` was
   raised to **2.5 s** on 2026-07-18 after a bug where a 1.0 s timeout (tighter
   than the ~1.5 s worst-case gap of its ~3 Hz source) tripped "stop due to
   invalid source" and **froze the robot** — fixed and verified.
4. **No blind reverse** — `BackUp` removed from the recovery tree;
   `vx_min -0.10` allows only slow, deliberate reverse under the controller,
   never a blind recovery lunge.
5. **Firmware watchdog** — if `/cmd_vel` stops arriving for 500 ms the ESP32
   stops the motors, so a stalled/crashed upstream node cannot leave the rover
   driving.

### Dead-zone compensation (why it exists)

The L298N + gearmotors need ~60 % PWM to overcome static friction; MPPI's slow
approach speeds (0.07–0.15 m/s) only hummed and stalled a few cm short of goals.
`cmd_vel_deadband.py` rescales every non-zero command onto the effective range
(linear → [0.20, 0.30] m/s, angular → [0.80, 1.40] rad/s), exact zeros pass
through as stop. Nav2 keeps planning in ideal velocities; visual odometry closes
the loop on the faster real motion. **This is a stopgap** — the correct fix is
the same remap inside `rover_firmware.ino`, after which this node is deleted and
the collision monitor's output goes straight back to `/cmd_vel`.

---

## 7. The three ways to say "go there"

All three end in a `nav2_msgs/action/NavigateToPose` goal in the `map` frame.

1. **A coordinate** — directly, e.g. from RViz "2D Goal Pose" or:
   ```
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
     "{pose: {header: {frame_id: map}, pose: {position: {x: 0.5}, orientation: {w: 1.0}}}}"
   ```
2. **An object** — the vision layer (`detections_3d`, `scripts/run_vision_ai.sh`)
   runs YOLOv8n on the color stream, samples depth at each detection, deprojects
   to a 3D point and transforms it to `map`, publishing
   `/vision/detections_3d`. The brain turns "the chair" into that object's
   map-frame coordinate and sends it as a nav goal. A separate visual-servoing
   path (`/vision/target` → `/vision/target_result`) hunts a visible object
   without needing the map. **YOLO runs on-demand** (2026-07-18) to save CPU —
   full rate only when a target is set or someone subscribes to the detections.
3. **A Telegram phrase** — the Pi5 "brain" (LangGraph) takes natural language
   ("go to the bottle"), uses the Mac-mini VLM + the `look()` camera feed
   (`/camera/color/image_raw/compressed`) to ground it, resolves the target to a
   map-frame goal via the vision layer, and issues the NavigateToPose action.
   The brain treats silence on the vision feeds as a dead camera and halts —
   a safety property, not a bug.

---

## 8. Current status & what's left

| Layer | Status |
|---|---|
| D555 camera (infra1/depth/color/IMU) | ✅ live, verified |
| RTAB-Map localization + persistent map | ✅ live, tracking |
| nvblox mapping + costmaps | ✅ live |
| Nav2 planning + control + anti-crash | ✅ live, goal test produced real commands |
| Vision (YOLO on-demand, look feed) | ✅ live |
| **Physical motion** | ⛔ **needs the ESP32 rover powered** (`/cmd_vel` has no consumer yet) |
| Camera mount TF (`base_link→camera0_link`) | ⚠️ placeholder — measure the real chassis |
| Chassis footprint in costmaps | ⚠️ placeholder — measure |
| D555 IMU fusion into odometry | ⏳ Phase 2 |

**To take it for a real drive:** power the ESP32, verify/flash firmware, then a
supervised low-speed mission. The whole perception + planning + safety pipeline
above is already running and was verified end-to-end in camera-only mode.
