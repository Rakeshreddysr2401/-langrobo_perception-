# Sim/Real Parity — one stack, two data sources

**Principle: the simulator does not get its own stack. It impersonates the
hardware.** Everything above the hardware line is byte-identical in both
modes — same containers on the same Jetson, same RTAB-Map, same nvblox, same
Nav2 (same yaml, same behavior tree), same vision AI, same Pi5 client and
brain, same Mac mini VLM, same Telegram flow. The stack cannot tell which
mode it is in; that is the guarantee behind "if it works in sim, it works on
the rover."

```
            INVARIANT (never branches on mode)
  Telegram ↔ Pi5 brain ↔ Mac mini VLM
  pi5/langrobo_client.py
  Nav2 (nav2_real.yaml + no-blind-backup BT) · nvblox · RTAB-Map · vision AI
  ─────────────── the hardware contract (§2) ───────────────
            SWAPPED (config/hardware: real | sim)
  real: D555 on ethernet (driver)   +  ESP32 rover on /cmd_vel
  sim:  rover_sim on the laptop, publishing the exact same camera
        topics AND consuming /cmd_vel as the simulated rover
```

Mode lives in `config/hardware` (`real` | `sim`), mirroring
`config/localization`. `run_all.sh` branches ONLY in stage 1: real starts
the D555 driver; sim *waits* for the laptop to provide the same topics.
Stage 0 (robot TF) and stages 2–5 are identical code paths.

## 1. Operating it

```bash
echo sim  > config/hardware && robot restart   # simulation mode
echo real > config/hardware && robot restart   # real rover
scripts/check_contract.sh                      # the parity gate, either mode
```

Interlocks (enforced by `run_all.sh` and `check_contract.sh`):

- **sim mode refuses to start while the real rover is on the network**
  (micro-ROS node `rover_esp32` visible ⇒ a live ESP32 would replay the sim
  mission on the floor). Power the rover off, or stop `langrobo-microros`
  on the Pi5. Physically disconnect/power off the D555 too — its onboard
  DDS stack is a participant even without the driver.
- **real mode refuses to treat a running simulator as the camera**
  (`/rover_sim/status` flowing ⇒ abort).

## 2. The hardware contract

Captured from the live D555 rig 2026-07-17; calibration snapshots in
`config/d555_contract/` (camera_info yaml per stream + the full static frame
tree). `scripts/check_contract.sh` enforces every row.

**Produced** (all stamped within 2 s of the Jetson wall clock):

| Topic | Type / format | Floor |
|---|---|---|
| `/camera/camera0/infra1/image_rect_raw` | Image mono8 896×504, frame `camera0_infra1_optical_frame` | 10 Hz |
| `/camera/camera0/infra1/camera_info` | CameraInfo (intrinsics: see snapshot) | with image |
| `/camera/camera0/depth/image_rect_raw` | Image **16UC1** (mm) 896×504, frame `camera0_depth_optical_frame` | 10 Hz |
| `/camera/camera0/depth/camera_info` | CameraInfo | with image |
| `/camera/camera0/color/image_raw` | Image rgb8 896×504, frame `camera0_color_optical_frame` | 5 Hz |
| `/camera/camera0/color/camera_info` | CameraInfo | with image |
| `/camera/camera0/motion/sample` | Imu, 200 Hz combined gyro+accel (Phase 2) | best effort |
| `/tf_static` | `camera0_link` → infra1/infra2/depth/color/motion frames + `*_optical_frame`s (see `config/d555_contract/tf_static_camera0.yaml`) | once, latched |
| `/rover_sim/status` | **sim only** — the mode marker; never publish from real hardware | ≥0.2 Hz |

Notes that matter for realism: depth is computed in the **infra1 viewpoint**
(RTAB-Map feeds infra1 as "rgb" + raw depth, registered by construction —
the sim's grayscale+depth cameras must share one viewpoint/frame the same
way). The IR emitter is off in real mode, so real depth is noisy; sim depth
should have noise enabled, not be perfect.

**Consumed** — the simulated rover must behave like the real one:

- `/cmd_vel` (`geometry_msgs/Twist`): differential drive; 0.30 m/s = full
  scale; per-wheel PWM deadband as in the firmware
  (`pwm = PWM_MIN 130/255 + |v|·(1−130/255)`, i.e. it *cannot* creep);
  open-loop (commanded ≠ achieved; add slip); **500 ms command watchdog**
  → stop, exactly like `rover_firmware.ino`.
- Publishes **no odometry and no TF** — localization comes from RTAB-Map on
  the Jetson in both modes. The sim's ground-truth pose is for evaluation
  dashboards only, never on the ROS graph the stack sees.
- Robot geometry = the real chassis: footprint from `nav2_real.yaml`,
  camera mounted at the `run_robot_tf.sh` offsets (single source of truth —
  currently placeholders pending measurement, task #1).

**Clock discipline:** no `/clock`, no `use_sim_time` anywhere. The sim runs
at real-time factor 1.0 and stamps outgoing messages with the wall clock at
the bridge; the laptop stays chrony/NTP-synced to the LAN (<100 ms). This
keeps every timeout, TF cache window, and DDS behavior identical to real —
`use_sim_time` is itself a sim/real divergence and is banned.

## 3. rover_sim v2 — the laptop side (to build; needs laptop SSH access)

Gazebo (Harmonic) + `ros_gz`, one package:

1. **World**: rooms with walls, furniture, and YOLO-recognizable props
   (bottle, chair, cup — textured models, not gray boxes) at D555-visible
   heights.
2. **Rover model**: diff-drive matching the measured chassis + a camera
   sensor rig at the `run_robot_tf.sh` mount offsets producing grayscale
   (infra1), depth (same viewpoint), and color streams with the intrinsics
   from `config/d555_contract/` and sensor noise on.
3. **Contract bridge** (the only custom node): remaps/renames `ros_gz`
   topics to the contract names, re-stamps with wall clock, publishes the
   `camera_info`s from the snapshots and the `/tf_static` tree, publishes
   `/rover_sim/status`, and implements the `/cmd_vel` deadband + watchdog
   behavior on the drive plugin.
4. **Network**: gigabit ethernet to the Jetson (the camera streams are
   ~300 Mbit/s — WiFi will not carry them; the D555's cable/port works).
   Plain multicast DDS, `ROS_DOMAIN_ID=0`, no discovery server, no
   `ROS_DISCOVERY_SERVER` — same rules as everything else.

The old `rover_sim` (Gazebo + Fast DDS discovery server) and the old sim
profiles in this repo predate this architecture and are superseded.
`scripts/run_perception_{sim,real}.sh` were deleted 2026-07-18; still-present
Phase 5 deletion candidates: `config/nav2_sim.yaml`, `config/nvblox_sim.yaml`,
`launch/perception.launch.py` (its docstring holds the cuVSLAM-SIGILL
rationale — also captured in `scripts/run_localization.sh`). There is
deliberately no "sim yaml": one Nav2 config, one nvblox config, one BT, in
both modes.

## 5. PAUSED — pending sim work (2026-07-18)

The laptop simulator is functional (RTAB-Map tracks, nvblox builds, Nav2
plans, YOLO detects against it; camera rates 30+Hz via the C++ restamper)
but PAUSED while the real camera-only stack is brought up step by step.
Resume checklist, in order:

1. `check_contract.sh` in sim mode was 20/28. Script-side fixes for the QoS
   warning pollution + stamp arithmetic are DONE (2026-07-18, untested in
   sim). Still open, all cross-host visibility questions:
   - `/rover_sim/status` published on the laptop was not seen by
     `ros2 topic echo` inside the Jetson container even though the camera
     topics (also laptop-published) were — diagnose (QoS? echo default
     reliability vs publisher?), or move the check to the laptop side.
   - `/cmd_vel` subscriber count from inside the container did not include
     the laptop's `contract_bridge.py` subscription — check from the laptop
     or count publishers on `/rover_sim/drive_cmd` instead.
2. Rerun `check_contract.sh` (sim) → target 0 failures.
3. `run_all.sh` sim look-feed gate now waits 180s (was 60) — verify it
   passes cold.
4. Scripted missions per §4 (`go-object bottle`, full Telegram run).
5. Promotion gate §4 → then enable the real rover.

Start/stop on the laptop: `~/langrobo/rover_sim/run_sim.sh` / `stop_sim.sh`
(deploy from repo with `scripts/deploy_rover_sim.sh`).

## 4. Promotion gate — "works in sim ⇒ enable real rover"

1. `config/hardware` = `sim`, `robot restart` → all stages green.
2. `scripts/check_contract.sh` → **0 failures** in sim mode.
3. Scripted missions all succeed in sim, via the *same* entry points used
   on the real robot (`langrobo_client.py go / go-object / go-pixel`, plus
   at least one full Telegram → "go to the bottle" → arrived run), with
   zero wall contacts in the sim viewer.
4. Only then: `config/hardware` = `real`, `robot restart`,
   `check_contract.sh` green again (now with the rover powered →
   `/cmd_vel` consumer check turns green), first mission supervised at low
   speed.

Any change to configs, code, or firmware behavior goes through the gate
again: sim first, then real. If a change requires editing something in the
swapped layer only (e.g. the D555 driver), mirror the consequence into the
contract + `rover_sim` bridge in the same commit.
