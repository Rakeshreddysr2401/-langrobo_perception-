# ARCHITECTURE — how the pieces connect

Read this once before issue 00, then come back to it whenever an issue mentions
something you don't recognise.

---

## 1. The machines

Four computers. Only the **Jetson** matters for issues 00–08.

| Machine | Address | Job in this plan |
|---|---|---|
| **Jetson Orin Nano 8 GB** | this machine | Everything: camera, cuVSLAM, nvblox, nav2. All nine issues run here. |
| **Your laptop** | `192.168.1.10` | Runs RViz. This is your window into the robot's mind. |
| **Pi 5** | `192.168.1.16` | Phone teleop web page (`:8091`), and relays wheel encoder data |
| **ESP32** | `rover-esp32.local` (`192.168.1.12`) | Drives the motors, counts the wheel encoders |

> ⚠️ The laptop used to be `.12`. DHCP gave `.12` to the **ESP32**. If you ssh to
> `.12` expecting a laptop, you are talking to a microcontroller and it will look
> like "the laptop is off". The laptop is **`.10`**.

They all share one ROS 2 network (`ROS_DOMAIN_ID=0`), so a topic published on the
Jetson is visible on the laptop with no extra setup.

Full cross-machine detail — every topic and who consumes it — lives in
[`../orin-nav-stack/SYSTEM_INTEGRATION.md`](../orin-nav-stack/SYSTEM_INTEGRATION.md).

---

## 2. Frames: the single most important idea

Almost every confusing thing this robot does is a **frame** problem or a **rate**
problem. Frames first.

A *frame* is a coordinate system attached to something. "2 metres forward" is
meaningless until you say *forward from what*. ROS calls the tree of these
relationships **TF**.

```
  map ──────► odom ──────► base_link ──────► camera0_link
   │            │              │                   │
   │            │              │                   └── the camera lens
   │            │              └── the robot itself, origin ON THE FLOOR
   │            │                  between the two wheels
   │            └── "where I have driven since boot", smooth but slowly WRONG
   └── "where I am in the room", jumps when SLAM recognises a place
```

Read it as a chain of corrections:

- **`base_link`** is the robot. Its origin sits on the ground, dead centre
  between the wheels. Everything physical is measured from here — the camera is
  `0.163 m` above it.
- **`odom → base_link`** is *odometry*: smooth, continuous, never jumps. It is
  computed by adding up small motions, so its error grows forever. Over 10 m it
  might be off by 20 cm. Good for "am I moving smoothly right now".
- **`map → odom`** is the *SLAM correction*. When cuVSLAM recognises a place it
  has seen before, it doesn't yank `base_link` — it adjusts this correction
  instead. That is why `map` **jumps** and `odom` never does.
- **`base_link → camera0_link`** is fixed geometry: where the camera is bolted.
  It never changes while driving. Getting it wrong by 4 cm made the robot think
  the floor was an obstacle — see §5.

**Why two frames instead of one:** motion controllers need smoothness (use
`odom`), and goals need to stay put in the room (use `map`). One frame cannot be
both.

---

## 3. The data flow

### Seeing where you are

```
  D555 camera
    ├── IR left + IR right (stereo, ~26 Hz)
    │      └──► cuVSLAM ──► /odom  (x, y, yaw — absolute, ~21 Hz)
    │                          │
    ├── gyro (~74 Hz) ──► imu_to_base ──► /imu/base (yaw rate)
    │                          │          │
    └── depth (~24 Hz)         │          │
                               ▼          ▼
   ESP32 encoders ──► Pi5 relay ──► robot_localization EKF (20 Hz)
      /wheel_state       /wheel_odom          │
      (forward speed)                         ├──► odom→base_link TF
                                              └──► /odometry/filtered
```

Three sensors, three different strengths, which is exactly why they are fused:

| Sensor | Gives | Fails when |
|---|---|---|
| **cuVSLAM** (cameras) | absolute x, y, yaw | fast pivots, blank walls, low light |
| **Gyro** | yaw *rate*, very fast | drifts if you integrate it alone |
| **Wheel encoders** | forward speed | wheels slip |

The EKF (`config/ekf.yaml`) blends them. When you pivot fast and cuVSLAM
momentarily loses tracking, the gyro and wheels carry the pose through the gap.
That is the whole point of issue 03.

> The EKF deliberately **ignores the accelerometer**. A MEMS accel fused naively
> drifts hundreds of metres because gravity and bias swamp the tiny real signal.
> A slow ground robot gains nothing from it and inherits all its drift.

### Seeing the world

```
  D555 depth ──► nvblox ──► 3D voxel map (the mesh you see in RViz)
                     │
                     └──► ESDF 2D slice (~9 Hz) ──► nav2 costmap
```

nvblox builds a 3D model out of depth images. nav2 cannot use 3D directly, so
nvblox flattens a horizontal *slice* of it (currently everything between
`0.12 m` and `0.40 m` above the floor) into a 2D "where can I not go" grid.

### Deciding and moving

```
  goal (you click in RViz)
    ↓
  nav2 planner  ── computes a path around obstacles ──► /plan  (green line)
    ↓
  MPPI controller ── picks wheel speeds to follow it ──► /cmd_vel_nav
    ↓
  velocity_smoother      (no jerky commands)
    ↓  /cmd_vel_smoothed
  collision_monitor      (emergency stop on depth points)
    ↓  /cmd_vel_shim
  safety_guard           (3 forward bumper layers)
    ↓  /cmd_vel
  ESP32 → 50 Hz PID → motors
```

**Every command passes through this single chain.** Nothing is allowed to jump
in halfway — a node publishing straight to `/cmd_vel_nav` once created a feedback
loop *and* bypassed the collision monitor at the same time.

> ⚠️ The **phone teleop is the one exception** — it publishes straight to
> `/cmd_vel`, bypassing `safety_guard`. When you drive by phone you *are* the
> safety system. Drive by sight.

---

## 4. Where things live

```
orin-nav-stack/
  run_stack.sh              ← the one script. up / status / view / vision / nav2 / stop
  cuvslam_ros_node.py       ← cuVSLAM wrapper (SLAM, save/load map)
  config/
    ekf.yaml                ← sensor fusion (issue 03)
    nvblox.yaml             ← 3D mapping + the ESDF slice heights (issues 04, 06)
    nav2.yaml               ← planner, MPPI, speed caps (issues 07, 08)
    laptop_view.rviz        ← the RViz layout you open on the laptop
    perception_view.rviz    ← perception-only layout, for manual mapping runs
  nodes/
    stack_status.py         ← what `status` uses. Measures RATES.
    odom_health.py          ← publishes /odom/health — "is my pose lying to me?"
    odom_ruler.py           ← tape-measure tool (issue 02)
    floor_probe.py          ← measures where the floor sits (issue 04)
    imu_to_base.py          ← rotates the gyro into base_link
    safety_guard.py         ← forward bumper
    depth_to_cloud.py       ← depth → point cloud for collision_monitor
    detections_3d.py        ← YOLO + depth → 3D object positions (later)
  firmware/
    rover_firmware_v2.ino   ← ESP32: 50 Hz encoder PID (issue 03)
  maps/current/             ← saved maps live here
```

The repo is mounted **read-only** into the container over the baked image, so
**editing a config or node takes effect on the next restart — no rebuild.** That
is the loop you will use for all nine issues.

---

## 5. Measurements — things we know because we measured them

This table is the expensive part of this repo. Every row cost at least one
debugging session. **Do not re-litigate these without new measurements.**

| Date | Finding | Evidence |
|---|---|---|
| 2026-08-09 | **IR emitter ON breaks cuVSLAM scale.** A tape-measured 100 cm push read as **26 cm**. Emitter OFF, the same push read 97 cm. | The projected dot pattern is painted on the *world*, so the dots don't move with the camera — cuVSLAM concludes you barely moved. Emitter is now forced OFF in `run_stack.sh`. |
| 2026-08-09 | **Colour stream starves the IR stream.** With `enable_sync:=true` the map came up blank. | Fixed with `enable_sync:=false`. |
| 2026-08-10 | **Camera height was wrong by 4 cm.** TF said 0.200 m; the floor deprojected to **+0.037 m** instead of 0.000. | `floor_probe.py`, 19.5k points. Corrected to **0.163 m** → floor now reads **+0.000 m** (spread −0.002…+0.004). This made nvblox map the floor as an obstacle and biased every 3D detection. |
| 2026-08-10 | A **+1.07° nose-up pitch** remains after the height fix. | Below the ~1.5° threshold where a z-only correction stops working. Partly real mount tilt, partly far-range depth bias. Not worth blocking on. |
| 2026-08-10 | **The floor-as-obstacle workaround is still in place.** `esdf_slice_min_height` was raised 0.05 → **0.12** to dodge the 4 cm error. | The error is fixed now, so this can come back down toward ~0.06 — but that changes what nav2 calls an obstacle, so it must be validated while driving. **This is issue 06.** |
| 2026-08-10 | **cuVSLAM freezes silently when the camera is starved.** Orin load > ~8 drops IR to 1 Hz; cuVSLAM stops and **never recovers**, while the stack keeps navigating on a frozen pose. | Nothing logs an error. This is why rule 3 exists. |
| 2026-08-10 | **The laptop is `192.168.1.10`.** `.12` is the ESP32. | Cost one full "why can't I see RViz" session. |
| 2026-08-10 | RViz launched over ssh renders **invisibly** if the laptop is sitting at the login screen. | You must be *physically logged in* at the laptop. |
| 2026-08-10 | **`cmd_vel_deadband.py` was removed.** It re-floored every command to vx ≥ 0.20 / wz ≥ 0.80, leaving MPPI no fine control — the rover weaved and spun instead of tracking the path. | It was built for the old open-loop L298N motors. Firmware v2's 50 Hz encoder PID does the static-friction job properly. File deleted 2026-08-10. |
| **2026-08-10** | **ESP32 `/wheel_state` runs at exactly 10.0 Hz.** Firmware asks for 50 ms (20 Hz) at `rover_firmware_v2.ino:391` and delivers half that. | Earlier notes said "1 Hz, still on old firmware" — **that is now wrong**, a reflash has happened. The remaining half-rate is a real bug. **This is issue 03.** |

### Known-good baseline

Measured 2026-08-10 with `./run_stack.sh up` (perception only, nav2 not started):

```
  camera IR left     26.0 Hz     cuVSLAM odom    20.8 Hz
  camera depth       23.5 Hz     nvblox slice     9.2 Hz
  EKF odom           20.0 Hz     IMU (gyro)      74.2 Hz
  ESP32 encoders     10.0 Hz  ← should be 20 (issue 03)
  TF map->odom       0.06 s old  pose trust      True
  Orin load1         2.5         (> 8 starves the camera)
```

If your numbers look roughly like this, the stack is healthy.

---

## 6. Vocabulary

| Term | Plain English |
|---|---|
| **Odometry** | The robot's running estimate of how far it has moved |
| **VO / visual odometry** | Odometry computed by watching the world slide past the camera |
| **SLAM** | Building a map *and* locating yourself in it at the same time |
| **Loop closure** | "I've been here before" — lets SLAM cancel accumulated drift |
| **TF** | The tree of coordinate frames and how they relate |
| **Voxel** | A 3D pixel — a small cube of space nvblox marks full or empty |
| **ESDF** | A grid where each cell stores *distance to the nearest obstacle*. Cheap for a planner to ask "how much clearance here?" |
| **Costmap** | 2D grid of "how bad is it to be here". Lethal near walls, free in the open |
| **Planner** | Draws the path from A to B |
| **Controller (MPPI)** | Turns that path into actual wheel speeds, moment to moment |
| **Lifecycle node** | A ROS 2 node that must be explicitly *activated*. nav2 nodes are all lifecycle nodes — "running but not active" is a real and confusing state |
| **QoS** | Delivery rules for a topic. RELIABLE = resend until it arrives; BEST_EFFORT = drop it if late. A mismatch means two healthy nodes silently never talk |
