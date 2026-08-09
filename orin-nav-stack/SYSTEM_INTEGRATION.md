# LangRobo — System Integration & Gap Roadmap

**One place that describes the *whole* robot as it actually runs today** (all four
machines), the exact ROS 2 contract between them, and the prioritized gaps toward the
product goal: *a household robot that can **see, understand, move in sync with its
environment**, and do tasks* — starting with the flagship command **"go near the chair."**

> **See also [`SESSION_2026-08-10.md`](SESSION_2026-08-10.md)** — the debugging session that
> fixed the go-to-object path end to end (cmd_vel chain, floor-as-obstacle, pose trust),
> with the measurements behind each change and the current blocked state.

> This doc is the cross-machine source of truth. The Jetson perception/nav details live in
> [`README.md`](README.md); the Pi5 brain details live in that repo's `ARCHITECTURE.md` /
> `HOW_IT_WORKS.md`. Where those disagree with reality, see **§7 Doc drift** — fix there.

Last synced: 2026-08-08 (encoder-odom fusion session).

---

## 1. The four machines (REAL "rover" mode)

| Machine | Role today | Key fact |
|---|---|---|
| **Jetson Orin Nano 8 GB** (`orin-nav-stack`, this repo) | Perception + navigation: **cuVSLAM** (standalone pyCuVSLAM cu12 wheel — *not* RTAB-Map), **nvblox** 3D map, **nav2** (planner+MPPI+safety_guard), **YOLO** detections (2D bearing + 3D metric), **pixel→goal**, **EKF** odom fusion | The 8 GB Orin is saturated by perception — **voice/TTS is OFF here in rover mode** |
| **Pi 5 8 GB** (`pi5_ros2_ws`, `~/ros2_ws`) | The **brain**: LangGraph supervisor + agents (chat, vision `local_agent`, `navigate`, status, errands…), Qdrant memory, Telegram, **micro-ROS agent** (ESP32 WiFi bridge, UDP 8888) | `langrobo_core` is a pure LangGraph zone (zero rclpy); only `langrobo_ros` touches ROS |
| **Mac mini** | **VLM** — llama.cpp Gemma multimodal at `singireddys-mac-mini.local:8080` (OpenAI-compatible) | Answers vision questions (`look()`) and pixel queries |
| **ESP32** (`rover-esp32.local` — DHCP, was `.11` in these docs but measured `192.168.1.12` on 2026-08-10; **always resolve the mDNS name before an OTA flash**) | Drivetrain: 2×BTS7960 + 180 RPM encoder motors, **50 Hz PID** closed-loop wheel control | Firmware v2 (`firmware/rover_firmware_v2.ino`); WiFi micro-ROS |

Because voice is off on the Orin in rover mode, **the human interface today is Telegram**
(text) + the phone teleop web page. Wake-word voice (`Hey Chotu`) is a future add that needs
a home for STT/TTS that isn't the perception-saturated Orin (see §6 G4).

---

## 2. Flagship flow — "go near the chair"

What *should* happen end-to-end (★ = gap today, see §6):

```
Telegram/voice "go near the chair"
  → Pi5 brain: supervisor → navigate agent
  → understand target = "chair" (COCO class)
  → ★ decide approach mode:
        (a) if chair pose is known in the map  → navigate_to_pose(x,y)   [nav2, obstacle-aware]
        (b) else if chair visible in 3D dets   → goal from /vision/detections_3d → nav2   ★ (not wired: uses 2D mono servo today)
        (c) else                               → visual search: pan head / rotate to find it   ★ (head servo not driven)
  → nav2 (Jetson): plan on nvblox map → MPPI → velocity_smoother → collision_monitor
                   → safety_guard → /cmd_vel      (cmd_vel_deadband REMOVED 2026-08-10)
  → ESP32: PID wheels; encoders → /wheel_state → EKF (fused odom keeps pose steady through pivots)
  → arrival report back to the originating Telegram chat
```

Today path (b) actually runs the **blind 2D mono servo** (`navigate_to_visible_object`): turns
toward YOLO `bearing_x`, drives straight, no obstacle avoidance, no metric distance. With the
D555 depth + `/vision/detections_3d` giving metric map poses, this should become an
**obstacle-aware nav2 goal**. That single upgrade is the biggest win for the flagship command.

---

## 3. The ROS 2 contract (brain ↔ Jetson ↔ ESP32)

`ROS_DOMAIN_ID=0`, plain multicast (Fast-RTPS). ✓ = wired & matches · ★ = gap.

### Pi5 brain → robot
| Topic / action | Type | Consumer | Status |
|---|---|---|---|
| `/cmd_vel` | `Twist` | ESP32 wheels | ✓ (brain `move_robot`/teleop publish here directly, **bypassing safety_guard**) |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Jetson nav2 | ✓ |
| `/vision/target` | `String` | Jetson `detections_3d` | ✓ |
| `/vision/pixel_query` | `PointStamped` | Jetson `pixel_to_goal` | ✓ |
| `/servo_pan`, `/servo_tilt` | `UInt16` (90=centre) | ESP32 servos | ★ **firmware v2 has no servo driver** |
| `/camera/pan_tilt_state` | `String` (JSON) | Jetson pan/tilt TF broadcaster | ★ **no such Jetson node** |
| `/voice/robot_speech`, `/audio/music_cmd` | `String` | Jetson TTS / audio | ★ **voice/audio off on Orin (rover mode)** |

### Robot → Pi5 brain
| Topic | Type | Producer | Status |
|---|---|---|---|
| `/vision/target_result` | `String` JSON `{found,bearing_x,rel_size,conf}` | Jetson `detections_3d` | ✓ (2D servoing) |
| `/vision/detections_3d` | `String` JSON `{frame:map, objects:[{label,x,y,z}]}` | Jetson `detections_3d` | ✓ (**under-used** by brain approach) |
| `/vision/pixel_result` | `String` JSON | Jetson `pixel_to_goal` | ✓ |
| TF `map→odom→base_link` | TF2 | cuVSLAM + EKF | ✓ (`get_current_pose`) |
| `/voice/tts_speaking`, `/audio/music_state` | `Bool`/`String` | Jetson voice | ★ off in rover mode |

### ESP32 (micro-ROS, this repo `firmware/`)
| Topic | Dir | Type | Status |
|---|---|---|---|
| `/cmd_vel` | in | `Twist` [RELIABLE] | ✓ |
| `/pid_gains` | in | `Vector3` (Kp,Ki,minDuty) [RELIABLE] | ✓ live tuning |
| `/wheel_state` | out | `Vector3` (velL,velR,cmd_vx) [BEST_EFFORT] | ✓ → Pi5 relay → `/wheel_odom` → EKF |
| `/servo_pan` `/servo_tilt` | in | `UInt16` | ★ not implemented |

### Jetson-internal motion chain (corrected 2026-08-10)
```
controller_server ─► /cmd_vel_nav ─► velocity_smoother ─► /cmd_vel_smoothed
   ─► collision_monitor ─► /cmd_vel_shim ─► safety_guard ─► /cmd_vel ─► ESP32
```
`cmd_vel_nav` is ALSO the velocity_smoother's input (nav2_bringup remaps it), so the collision
monitor must NOT publish there — doing so created a feedback loop *and* bypassed the monitor.
collision_monitor's obstacle source is `/perception/depth_points` from `nodes/depth_to_cloud.py`
(10 Hz); it hard-stops the robot if that source goes stale.

### Pose trust (new)
| Topic | Type | Producer | Meaning |
|---|---|---|---|
| `/odom/health` | `String` JSON | Jetson `odom_health` | `trust:false` ⇒ pose unreliable — do not start a move, do not believe "arrived". Catches `VO_STALLED`, `CAMERA_STARVED`, `VO_UNDER_REPORTING`, `VO_DRIFT_STATIONARY`, `VO_SCALE_OFF`. |

### Odometry fusion chain (this session)
```
cuVSLAM /odom (x,y,yaw abs) ─┐
D555 gyro  /imu/base (vyaw)  ├─► ekf_filter_node ─► odom→base_link TF + /odometry/filtered
wheel_odom /wheel_odom (vx)  ┘   (config/ekf.yaml: odom0 pose, imu0 vyaw, odom1 vx)
```
Wheel path = `ESP32 /wheel_state → Pi5 langrobo_ros/wheel_odom_relay → /wheel_odom`.
**Relay is running** (started manually on the Pi5 2026-08-10; no systemd unit, so it does not
survive a reboot). Chain verified end-to-end: `/wheel_state` 1 Hz → `/wheel_odom` 1 Hz → EKF.
**Still blocked:** the flashed ESP32 emits 1 Hz, not the 20 Hz the HEAD firmware produces —
reflash required (see the wheel-odom memory + §6 G1). At 1 Hz the encoder path cannot bridge
cuVSLAM dropouts and `odom_health` cannot use it as ground truth.

---

## 4. Capability matrix — built vs. gap

| Capability | Built | Gap |
|---|---|---|
| **SEE** — localize | cuVSLAM VO + loop closure; EKF odom | encoders not yet fused live (1 Hz reflash) |
| **SEE** — 3D map | nvblox mesh + ESDF + occupancy | **front FOV only** — no head pan → blind sides/back |
| **SEE** — detect | YOLO 2D bearing + 3D metric (D555 depth) | — |
| **UNDERSTAND** — VLM | Mac Gemma via `look()`/`local_agent` | not fused with 3D map (no VLM-grounded object poses) |
| **UNDERSTAND** — semantics | named-pose `save_location`/`navigate_to_pose` | **no object-anchored memory** ("go to the chair" when not visible) |
| **MOVE** — plan | nav2 planner + MPPI + 3-layer safety_guard | — |
| **MOVE** — precise | open-loop timed `move_robot` (PWM/time calib) | should be **encoder closed-loop** (rotations/distances) |
| **MOVE** — approach object | 2D mono visual servo | **upgrade to 3D→nav2 goal** (obstacle-aware) |
| **MOVE** — follow/come-to-me | declined in prompt (awaiting depth) | **enable now** with D555 depth + visual search |
| **SYNC** — head/gaze | pan/tilt coded in brain bridge | **ESP32 servo driver + Jetson pan/tilt TF missing** |
| **TALK/LISTEN** | Telegram text; brain TTS/music tools | wake-word voice needs a non-Orin STT/TTS home |
| **PERSIST** | cuVSLAM map save + relocalize; save_location | nvblox wall-map reload across sessions not wired |

---

## 5. Sync = the feedback loops that make it feel "in sync with the environment"

1. **Pose sync** — EKF fuses cuVSLAM(abs) + gyro(vyaw) + encoders(vx) so odom stays smooth
   *through* the fast pivots that momentarily blind cuVSLAM (README §10). ← this session.
2. **Gaze sync** — head pan/tilt to look where it's going / search for a target, with the
   pan absorbed correctly into TF so map pose stays honest (`ensure_head_centred`). ← §6 G2.
3. **Semantic sync** — objects seen (3D detections + VLM) get remembered as map anchors so
   "the chair" resolves to a place even when out of view. ← §6 G3/G7.
4. **Safety sync** — depth + nvblox virtual bumper gate forward motion; nav2 replans. ✓

---

## 6. Prioritized gap roadmap

**G1 — Encoder odom fusion (IN PROGRESS).** Relay + `ekf.yaml odom1` done; blocked on ESP32
20 Hz reflash. Then validate `/odometry/filtered` on straight + pivot. *Foundation for every
precise move.*

**G2 — Pan/tilt camera head (New_Requirement).** (a) Add `/servo_pan`+`/servo_tilt` subscribers
to `rover_firmware_v2.ino` driving the 2 servos; (b) add a Jetson node that turns
`/camera/pan_tilt_state` into a `base_link→camera_mount→camera` TF so pan is modeled, not
mistaken for base rotation. Unlocks visual search + filling side/back map gaps.

**G3 — "go near the chair" as an obstacle-aware nav2 goal.** In the brain's approach path, prefer
`/vision/detections_3d` metric map pose → `navigate_to_pose(x,y)` over the blind 2D servo; keep
the 2D servo only as a last-resort final-alignment step. *Biggest flagship win.*

**G4 — Voice home.** Decide where wake-word STT/TTS lives (Pi5, or a small dedicated node) since
the Orin is perception-bound. Until then, document Telegram as the primary interface and make the
brain degrade the voice/music topics cleanly when no Jetson audio is present.

**G5 — Person "come to me" / follow.** Now that depth exists, enable the declined follow behavior
with D555-metric person tracking + a left/right visual search when the person isn't in frame.

**G6 — Object-anchored semantic memory.** Persist detected objects (label + map xyz + last-seen)
in Qdrant so "the chair"/"my cup" resolves without live sight; auto-name common furniture.

**G7 — Map persistence across sessions.** Wire nvblox `global_frame=map` + reload so saved walls
line up after a cuVSLAM relocalize (README §6 notes this is unfinished).

**G8 — Doc drift cleanup (§7).**

---

## 7. Doc drift to reconcile (Pi5 repo)

The Pi5 `CLAUDE.md` / `ARCHITECTURE.md` predate the cuVSLAM Jetson stack and describe a
different perception design. Correct these:

- ❌ "cuVSLAM does NOT run on Orin — RTAB-Map is the localizer" → ✅ **cuVSLAM runs** on Orin via
  the standalone pyCuVSLAM cu12 wheel (this repo's whole premise). EKF now fuses it with gyro +
  encoders.
- ❌ Jetson = "USB cam · STT (Whisper) · TTS (Kokoro) · YOLOv8n … separate `speech_vision` repo"
  → ✅ In rover mode the Jetson runs **`orin-nav-stack`** (cuVSLAM/nvblox/nav2/YOLO); **voice is
  off** on the Orin.
- ❌ ESP32 = "4-wheel drive chassis" → ✅ **2-motor differential drive**, BTS7960 + encoders, PID.

---

*Maintenance: update the contract table (§3) whenever a topic is added/renamed on either side —
a silent mismatch here is the #1 cause of "the brain talks but the robot doesn't move."*
