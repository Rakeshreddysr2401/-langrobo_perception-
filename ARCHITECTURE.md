# Architecture — the system as built

What runs where, what talks to what, and **why each decision went the way it
did**. For the measured numbers see [PHASE1.md](PHASE1.md); for how to run it
see [OPERATIONS.md](OPERATIONS.md); for open faults see [TODO.md](TODO.md).

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
   │    fusion_node  → /odom + TF odom→base_link                 │
   │    compare.py   the measurement instrument                  │
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
| **`/odom`** | `Odometry` | 20 Hz | **`fusion_node` — what nav2 consumes** |
| `/fusion/path` | `Path` | 2 Hz | `fusion_node` — the track driven, **latched** |
| `/fusion/status` | `String` | 1 Hz | `fusion_node` — who is covering for whom |

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
  odom ──(fusion_node, 20 Hz)──► base_link ──(static)──► camera0_link
```

| transform | owner | why |
|---|---|---|
| `odom → base_link` | **`fusion_node`** | the guarded estimate, not the raw one |
| `base_link → camera0_link` | static publisher | measured: x 0.170, z 0.163, yaw 2.06° |

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
| **`phase1/nodes/fusion.py`** | **the estimator, with no ROS in it** |
| `phase1/nodes/fusion_node.py` | wraps `fusion.py` → `/odom`, TF, `/fusion/status` |
| `phase1/nodes/compare.py` | the measurement instrument: side-by-side rows, gates, CSV |
| `phase2/launch/nvblox.launch.py` | nvblox: depth + `/odom` → TSDF, mesh, 2D grid |
| `phase2/rviz/rover.rviz` | the RViz view, standard message types only |
| `phase3/config/nav2.yaml` | nav2, with every NO-PIVOT adaptation marked |
| `phase3/bt/*.xml` | behaviour trees with Spin removed |
| `phase1/nodes/values.py` | one-shot readout of every sensor |
| `phase1/firmware/rover_firmware_v2.ino` | ESP32: PID, encoders, telemetry |

### Why `fusion.py` is separate from `fusion_node.py`

The estimator was validated over two days of tape-measured runs **inside
`compare.py`**. If the shipping node re-implemented it, that validation would
apply to nothing.

So the algorithm lives in one plain module with no ROS imports. `compare.py`
grades it; `fusion_node.py` publishes it. They cannot drift into two
implementations where only one was ever measured.

It was extracted **programmatically** — the method bodies are the same source
text, not retyped — and then verified by running both at once and driving:
0.6 cm and 0.47° apart.

---

## 5. The estimator

Fusion here is **assignment plus fallback**, not averaging. No sensor is good at
everything, and one that is bad at a job does not get that job.

| job | primary | measured justification |
|---|---|---|
| **heading** | gyro | −0.2% to −0.9% across five turns; cuVSLAM was +11% on one |
| **distance** | encoder ticks | texture- and direction-independent |
| **direction** | cuVSLAM | does not drift; re-measured against the world each frame |
| **tilt** | accelerometer (gravity) | absolute, never drifts |

### Every sensor is covered

| when this fails | this carries it |
|---|---|
| cuVSLAM blind (<30 landmarks) | wheels + gyro |
| cuVSLAM teleports | wheels + gyro |
| cuVSLAM silent | wheels + gyro, via the independent 20 Hz pulse |
| wheels slip in a turn | gyro heading |
| wheels stale | cuVSLAM distance, last good scale |
| gyro stale | cuVSLAM heading |
| gyro drifts long-term | cuVSLAM, straight-line only |

**FUSED has its own heartbeat.** Until that was added, both the fusion and its
fallback ran from cuVSLAM's callback — so cuVSLAM was still the heartbeat, and
if it went silent the pose froze while the other two sensors were healthy.

### Two rules that cost the most to learn

**Refusing bad messages is not enough — you must refuse a bad sensor.**
Rejecting individual teleports left cuVSLAM publishing at a confident 30 Hz
between them with an equally corrupted *direction*. With 4 landmarks, FUSED took
its heading from it and finished 42.4 cm from the start while the wheels *alone*
managed 17.4 cm. Fusion did worse than its own worst input.

**Do not calibrate during the manoeuvre that breaks your reference.** The
encoder/cuVSLAM scale ratio freezes while turning, because the wheels scrub 1.60×
front-to-rear in a pivot and would drag a good calibration off with distance the
rover never went.

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

- **`vx_max` ≤ 25 cm/s** — above it cuVSLAM teleports
- **Pivots cost tracking** — 2.0 rad/s swings the camera at 34 cm/s, past its
  limit. Prefer plans that turn and drive forward over spinning in place or
  reversing (cuVSLAM under-reads reverse by ~6%)
- **The D555 drops out** in three distinct ways — see [TODO.md](TODO.md) §7. The
  fusion survives it; a robot needing a human to reseat a cable does not
  autonomously
- **The rover cannot turn in place** ([TODO.md](TODO.md) §14). nav2 assumes
  rotation is free; every workaround is marked `NO-PIVOT` in
  `phase3/config/nav2.yaml` so they can be reverted together once it is fixed
