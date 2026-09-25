# Architecture — the system as built

What runs where, what talks to what, and **why each decision went the way it
did**. For the measured numbers see [docs/archive/PHASE1.md](docs/archive/PHASE1.md); for how to run it
see [OPERATIONS.md](OPERATIONS.md); for open faults see [docs/archive/TODO.md](docs/archive/TODO.md).

---

## 0. The stack, and why each piece

| layer | technology | why this one |
|---|---|---|
| OS / middleware | **ROS 2 Jazzy**, Fast-DDS | the version Isaac ROS ships against on Orin |
| camera | **RealSense D555** — PoE **ethernet DDS**, not USB | it is what the rover has. The ethernet transport is the source of several traps: it pings while dead, and subscribing to raw camera topics can take it offline |
| visual odometry | **cuVSLAM 16.0.0** via the standalone `pyCuVSLAM` wheel | the packaged `isaac_ros_visual_slam` is a **Thor** build and will not run on this Orin. The wheel is the only path |
| heading | the D555's own **gyro**, complementary-filtered | measured −0.2% to −0.9% error over four 360° turns — better than the wheels can do through scrub |
| distance | **wheel encoders**, calibrated against a tape | excellent in a straight line, useless mid-turn (§13) |
| fusion | **hand-written complementary filter**, `phase1/nodes/fusion.py` | *not* an EKF — see §5 for why a 3-sensor planar problem did not need one |
| LiDAR | **RPLidar C1** — 360°, 10 Hz, USB, driver `sllidar_ros2` pinned + one patch (`lidar/`) | the walls are the one reference that does not drift. Added 2026-09-22 for the pose, not for nav2 |
| drift correction | **slam_toolbox**, online async mapping, owns `map → odom` | scan-matches every scan against the walls it has seen, so the correction is continuous, where cuVSLAM's loop closure only fires on a recognised place (15–27 closures a session after §23, docs/archive/READINESS.md) — and the two cannot both own the frame, so cuVSLAM's is switched off (`SLAM=false`). No map is kept across power-off, by choice |
| mapping | **nvblox** — TSDF → ESDF → 2D slice, GPU | the Orin has the GPU for it, and the ESDF slice is what nav2's costmap layer consumes directly |
| planning | **nav2** — NavFn planner, Regulated Pure Pursuit controller | RPP steers by *arcs*, which is what a skid-steer rover can execute. DWB samples rotations it cannot |
| wheels | **ESP32 + micro-ROS** over WiFi UDP, BTS7960 drivers, PID per side | the board sits on the rover; the link to it must be the thing that fails visibly |
| teleop | a **web page on the Pi 5**, hold-to-move | any phone on the WiFi is a controller and a stop button, with no app to install |
| visualisation | **RViz2 on a laptop** | the Jetson needs its GPU for nvblox |

**Everything runs in one container** (`rover`) on the Jetson, brought up in
layers by `./rover`, because a failure in one layer must name itself rather than
appear as a wall of log with no owner.

---

## 1. Four machines

```
   ┌─────────────────────────────────────────────────────────────┐
   │  D555 depth camera            192.168.11.55  (own subnet)   │
   │  stereo IR · depth · IMU      Ethernet + PoE, NOT USB       │
   └───────────────┬─────────────────────────────────────────────┘
                   │  DDS over Ethernet
   ┌───────────────▼─────────────────────────────────────────────┐
   │  Jetson Orin Nano             192.168.1.15                  │
   │  everything in orin-nav:1.1                                 │
   │    vo_node      stereo IR → cuVSLAM → /vo/odom              │
   │    gyro_node    IMU → base_link → /gyro/base                │
   │    lidar_odom   /scan + gyro → /lidar/odom (every scan)     │
   │    fusion2      gyro·LiDAR·VO·wheels → /odom + TF           │
   │    harness      ./rover record / grade vs LiDAR truth       │
   └───────────────▲─────────────────────────────────────────────┘
                   │  DDS over WiFi
   ┌───────────────┴─────────────────────────────────────────────┐
   │  Pi 5                         192.168.1.16                  │
   │    micro-ROS agent   ESP32 ↔ ROS, UDP :8888                 │
   │    teleop web        hold-to-move → /cmd_vel, :8091         │
   └───────────────▲─────────────────────────────────────────────┘
                   │  micro-ROS over WiFi UDP
   ┌───────────────┴─────────────────────────────────────────────┐
   │  ESP32                        192.168.1.3                   │
   │    50 Hz PID control task (own FreeRTOS core)               │
   │    4× quadrature encoders · 2× BTS7960                      │
   └─────────────────────────────────────────────────────────────┘
```

**Why the split.** The ESP32 runs the control loop on its own FreeRTOS core so
that a network stall cannot stop the wheels being controlled. The Pi 5 owns the
micro-ROS bridge because it sits on the rover and the link to the board must be
short. The Jetson owns everything needing a GPU.

---

## 2. Topics

### From the ESP32

| topic | type | rate | contents |
|---|---|---|---|
| `/wheel_state` | `Vector3` | 20 Hz | `x`=velL, `y`=velR, `z`=commanded vx |
| `/wheel_ticks` | `Quaternion` | 20 Hz | **cumulative** counts: LF, LR, RF, RR |
| `/wheel_odom` | `Vector3` | 20 Hz | x, y (m), θ (rad), integrated on-board at 50 Hz |
| `/rover_diag` | `Vector3` | 1 Hz | `loop()` Hz, free heap KB, agent state |
| `/cmd_vel` | `Twist` | in | target body vx, wz |
| `/pid_gains` | `Vector3` | in | live tuning: Kp, Ki, minDuty |
| `/reset_odom` | `Vector3` | in | any message zeroes the on-board pose |

**Why `Quaternion` for four wheel counts.** It is four doubles, 32 bytes, and
needs no custom message package on either end — which matters when the container
that builds the Jetson side has no rebuild recipe.

**Why `/wheel_odom` is not `nav_msgs/Odometry`.** That message carries two 6×6
covariance blocks, ~700 bytes, over a 512-byte micro-ROS MTU. The Jetson wraps
three numbers where bandwidth is free.

**Why `/rover_diag` exists.** The 1 Hz telemetry fault took a day to find purely
because nothing reported how fast `loop()` was running. It does now — the next
occurrence is a glance rather than an investigation.

### From the camera

| topic | rate | used by |
|---|---|---|
| `/camera/camera0/infra1,2/image_rect_raw` | 30 Hz | cuVSLAM |
| `/camera/camera0/infra1,2/camera_info` | 30 Hz | intrinsics + baseline |
| `/camera/camera0/depth/image_rect_raw` | 30 Hz | **nothing yet** — Phase 2 |
| `/camera/camera0/motion/sample` | 200 Hz | `gyro_node` |

### On the Jetson

| topic | type | rate | published by |
|---|---|---|---|
| `/vo/odom` | `Odometry` | 30 Hz | `vo_node` — **raw**, teleports |
| `/vo/status` | `String` | 1 Hz | `vo_node` — landmarks, health |
| `/gyro/base` | `Imu` | 200 Hz | `gyro_node` — re-framed into `base_link` |
| `/lidar/odom` | `Odometry` | 10 Hz | `lidar_odom_node` — the LiDAR's pose, de-skewed scan-to-submap, covariance from each fit |
| **`/odom`** | `Odometry` | 20 Hz | **`fusion2_node` — what nav2 consumes**, with covariance |
| `/fusion/path` | `Path` | 1 Hz | `fusion2_node` — the track driven, **latched** |
| `/fusion/status` | `String` | 1 Hz | `fusion2_node` — `origin_epoch` (the Pi 5 brain uses it), per-source alive / accepted / rejected, LiDAR and cuVSLAM health, slip / stuck, sd |
| `/scan` | `LaserScan` | 10 Hz | `sllidar_node` — 720 beams, BEST_EFFORT, stamps shifted +82 ms to the measurement (LOCALIZATION.md §2.3) |
| `/map` | `OccupancyGrid` | ~0.5 Hz | `slam_toolbox` — the LiDAR's room, 5 cm cells, TRANSIENT_LOCAL |
| `/robot_description` | `String` | latched | `robot_state_publisher` — the URDF from `description/params.yaml`; RViz draws it (replaced `rover_marker`, 2026-09-24) |

### Mapping and navigation

| topic | type | rate | note |
|---|---|---|---|
| `/nvblox_node/static_occupancy_grid` | `OccupancyGrid` | 5 Hz | **the map.** VOLATILE |
| `/nvblox_node/static_map_slice` | `DistanceMapSlice` | 5 Hz | **not** an OccupancyGrid — what `NvbloxCostmapLayer` consumes |
| `/global_costmap/costmap` | `OccupancyGrid` | 1 Hz | TRANSIENT_LOCAL |
| `/local_costmap/costmap` | `OccupancyGrid` | 2 Hz | TRANSIENT_LOCAL |
| `/plan` | `Path` | on request | nav2's computed route |
| `/goal_pose` | `PoseStamped` | in | set from RViz |

**The durabilities are opposite and both matter.** nvblox publishes VOLATILE;
nav2's costmaps publish TRANSIENT_LOCAL. Subscribe with the wrong one and you
receive nothing while the topic looks perfectly alive — RViz renders that
identically to a dead publisher.

---

## 3. Frames

```
  map ──(slam_toolbox)──► odom ──(fusion2, 20 Hz)──► base_link ─┬─(static)──► camera0_link
                                                                    └─(static)──► laser
```

| transform | owner | why |
|---|---|---|
| `map → odom` | **`slam_toolbox`** (since 2026-09-22) | the LiDAR correction. cuVSLAM also publishes it when `slam:=true`, so `./rover pose` runs with `SLAM=false` and `./rover slam` refuses a second owner |
| `odom → base_link` | **`fusion2_node`** (since 2026-09-26) | fusion2: gyro + LiDAR odometry + VO + wheels, each weighted by its own reliability (LOCALIZATION.md §11). The Phase 1 `fusion_node` it replaced was retired the same day (git history) |
| `base_link → camera0_link` | robot_state_publisher (`description/`) | x 0.1623, y +0.0419 (the LEFT imager; fitted against LiDAR truth), z 0.175, roll −1.40°, yaw +0.96° (camera vs LiDAR). All calibrated 2026-09-25/26 — `description/params.yaml`; vo_node reads the whole mount from TF |
| `base_link → laser` | robot_state_publisher (`description/`) | x 0.1342, z 0.2498 measured 2026-09-24; **yaw +88.60° measured by driving** (`./rover lidar --calibrate`) |

`map → base_link` is the corrected pose; `./rover drive` steers on it. nav2
and the Pi 5 brain still plan in `odom` — moving them is a separate, driven
change (phase2/config/SLAM.md).

**`vo_node` runs with `publish_tf:=false`.** Only one thing may publish a
transform. The raw cuVSLAM pose is the one that teleports — 12 in a single
drive, one of them 651 cm in a frame — and nav2 consuming that would react
violently to a position the rover was never in. That is a **safety** issue, not
an accuracy one.

Two publishers of the same transform make TF non-deterministic, and the symptom
is a robot jittering between two poses with nothing in any log.

### Optical vs ROS axes

cuVSLAM reports the *camera* in *optical* axes (x right, y down, z forward). ROS
wants `base_link` in REP-103 (x forward, y left, z up). Both the axes and the
origin move, so the transform applies on **both sides**:

```
odom_from_base(t) = B · world_from_rig(t) · B⁻¹      B = base_link ← left_optical
```

A single multiply instead of a conjugation makes the rover appear to swing around
a point 17 cm in front of itself.

---

## 4. The nodes

| file | job |
|---|---|
| `phase1/nodes/vo_node.py` | stereo IR → cuVSLAM → `/vo/odom`, `/vo/status`. Owns the frame conjugation |
| `phase1/nodes/gyro_node.py` | D555 IMU → re-framed into `base_link` → `/gyro/base` |
| **`phase1/nodes/lidar_odom.py`** | **LiDAR odometry, no ROS in it** (graded offline and run live) |
| `phase1/nodes/lidar_odom_node.py` | wraps it → `/lidar/odom` |
| **`phase1/nodes/fusion2.py`** | **the estimator (EKF), no ROS in it** |
| `phase1/nodes/fusion2_node.py` | wraps it → `/odom`, TF, `/fusion/status`, `/fusion/path` |
| `phase1/nodes/health.py` | the pose / cuvslam / wheels honesty rows, from `/fusion/status` |
| `phase1/nodes/floor_calib.py`, `cam_lidar_calib.py` | camera tilt off the floor; camera vs LiDAR (`./rover camera --floor` / `--lidar`) |
| `phase1/harness/` | `./rover record` / `./rover grade`: every estimate against LiDAR truth |
| `description/` | the rover's shape: `params.yaml` → URDF, CAD, drift check |
| `phase2/launch/nvblox.launch.py` | nvblox: depth + `/odom` → TSDF, mesh, 2D grid |
| `phase2/rviz/rover_live.rviz` | the RViz view, standard message types only |
| `phase2/rviz/rover_live.sh` | launches RViz on the laptop with the `-d` that is load-bearing |
| `phase3/config/nav2.yaml` | nav2, with every NO-PIVOT adaptation marked |
| `phase3/bt/*.xml` | behaviour trees with Spin removed |
| `phase1/nodes/values.py` | one-shot readout of every sensor |
| `phase1/firmware/rover_firmware_v2.ino` | ESP32: PID, encoders, telemetry |
| `phase1/firmware/HARDWARE.md` | the drivetrain itself — wiring, motor/encoder specs, direction flags, bench results |
| `phase1/firmware/FLASHING.md` | how to flash it, and why the last flash was needed |

### Why the estimators have no ROS inside

`lidar_odom.py` and `fusion2.py` are plain Python classes. The harness runs
them offline over recorded bags and grades them; the nodes run the SAME class
live. What was measured is what ships (the rule the Phase 1 fusion.py set, and
kept).

---

## 5. The estimator

**fusion2** (SENSOR_FUSION_PLAN.md M3; results LOCALIZATION.md §10-11): one EKF
over [x, y, θ, vx, vy, gyro bias]. The gyro predicts at 200 Hz; every other
source corrects **by its own evidence**, through a statistical gate:

| source | enters as | weight comes from |
|---|---|---|
| **LiDAR odometry** | absolute pose (re-anchored offset); late fixes carried forward by motion only | each scan fit's residual and degeneracy; a healthy fit that fails the gate widens OUR uncertainty instead of being dropped |
| **cuVSLAM** | velocity while the LiDAR is healthy, absolute pose when it is not | landmark count; jumps fail the gate |
| **wheels** | forward speed, no sideways speed | turn rate (scrub) and slip evidence (front vs rear, wheels vs gyro) |
| **stillness** | zero velocity; re-measures the gyro bias | all encoders unchanged and nothing commanded |

Over 19 recorded runs: worst 0.8 cm / 0.45° (the Phase 1 fusion: 20.5 cm /
13°); live on the owner's hand-moved return test with skidding wheels:
0.1 cm / 0.32°. The Phase 1 estimator's hard-won rules (refuse a bad sensor,
not just a bad message; a sensor's own confidence cannot catch it being
confidently wrong; do not calibrate during the manoeuvre that breaks the
reference) are why fusion2 gates every source and scales the wheels by slip.

---

## 6. Design decisions, and what forced them

| decision | why |
|---|---|
| **Sources shown separately, not just fused** | a single number cannot tell you which sensor is lying. cuVSLAM once under-read a 100 cm push as 26 cm — a fused value would have looked plausible; two rows disagreeing would not |
| **Layered bring-up, one command per layer** | each layer checks the one beneath it, so a failure names its own layer instead of hiding in a wall of log |
| **Emitter set at runtime and read back** | `emitter_enabled:=0` is a no-op for a Boolean. The projector is bolted to the camera, so its dots move *with* the rig and a 60 cm push read 1.1 cm |
| **Loop closure OFF in Phase 1** | we are measuring raw drift; loop closure would mask exactly that |
| **Depth unused in Phase 1** | it is derived from the same stereo pair cuVSLAM consumes — not an independent witness |
| **Accelerometer never integrated for position** | error grows as t², 180 m in a minute. Its signal is gravity |
| **Distance from cumulative ticks, not velocity** | velocity must be integrated, so a dropped message loses that travel permanently |
| **Covariance from the gates** | nav2 weights poses by covariance; a confident wrong pose is worse than an honest uncertain one |
| **Twist from sensors, not by differencing the pose** | differencing a filtered pose feeds the filter's own lag back into control |
| **Health logged only on change** | a line every second is noise nobody reads; a line the moment a sensor drops out is the one worth finding |

---

## 7. Environment

**The container `orin-nav:1.1` has no build recipe.** It was made by
`docker commit`, not from a Dockerfile, and neither it nor its 57.8 GB base can
be reproduced. **Never install into it; never delete it.** `phase1/` is mounted
read-only, so nodes are edited on the host and the layer restarted — no rebuild.

`ROS_DISCOVERY_SERVER` is unset on every command. A stale one makes nodes
silently invisible to each other, which looks exactly like a crashed node.

The cuVSLAM wheel ships its own `libcuvslam.so` and CUDA-12 userspace libs, and
they must precede Isaac ROS's own build in `/opt/ros/jazzy` — that one is
compiled for Thor and will not run on this Orin.

---

## 8. Constraints that propagate to nav2

- **No speed cap is justified by our evidence.** This said `vx_max ≤ 25 cm/s`
  for a long time; two room loops on 2026-08-22 at 84 and 89 cm/s produced zero
  teleports and zero dropped frames. Teleports track TEXTURE, not speed. The
  honest constraint is to watch `landmarks` in `/fusion/status` — see TODO §3
- **Prefer turn-then-drive over spinning or reversing.** Not for the speed
  reason once given here: reversing is the measured problem (cuVSLAM under-reads
  it by ~6%, TODO §4), and a pivot on this chassis burns roughly two-thirds of
  its torque in scrub (TODO §13)
- **The D555 drops out** in three distinct ways — see [docs/archive/TODO.md](docs/archive/TODO.md) §7. The
  fusion survives it; a robot needing a human to reseat a cable does not
  autonomously
- **The rover cannot turn in place** ([docs/archive/TODO.md](docs/archive/TODO.md) §14). nav2 assumes
  rotation is free; every workaround is marked `NO-PIVOT` in
  `phase3/config/nav2.yaml` so they can be reverted together once it is fixed
