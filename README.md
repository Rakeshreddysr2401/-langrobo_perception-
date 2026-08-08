# LangRobo Perception — Jetson stack

Voice/VLM-driven rover: say "go near the chair" on Telegram and the robot finds
it, plans a path, and drives there.

```
                  ┌──────────────── WiFi (192.168.1.x) ────────────────┐
D555 camera ──eth──► Jetson Orin Nano ◄──────► Pi5 (brain) ◄──────► Mac mini
(192.168.11.55)     RTAB-Map · nvblox       LangGraph agent        llama.cpp
                    Nav2 · YOLO · grounding  Telegram · micro-ROS   Gemma-12B VLM
                          │ /cmd_vel_nav → deadband shim → /cmd_vel
                          └────────────────► ESP32 rover (192.168.1.11)
                                              wheels + pan/tilt head
```

First fully autonomous mission (pixel-grounded goal → plan → drive →
"arrived") completed 2026-07-16 on this architecture.

**Localization is pluggable** — `config/localization` holds one word:
`rtabmap` (default, verified under motion) or `cuvslam` (parked: explodes
under motion on this hardware — the Humble sidecar starves it of frames and
TF across the container boundary; re-evaluate when NVIDIA ships Orin/JP7
cuVSLAM builds that can live in the main container). Switch the word, then
`robot restart`.

## Daily operation — from the Pi5

```bash
robot status     # health table: camera → SLAM → Nav2 → wheels → brain → LLM
robot start      # bring up the Jetson perception stack (~2.5 min cold)
robot restart    # fresh SLAM origin + clean map (do this if pose looks wrong)
robot stop       # stop perception (brain + wheels bridge stay up)
```

Everything else is automatic: the Pi5 brain (`langrobo-brain`), the ESP32
bridge (`langrobo-microros`) and the Jetson stack
(`langrobo-perception.service`) all start at boot. When `robot status` is
green, talk to the robot on Telegram.

### Manual teleop from your phone

Open **`http://192.168.1.16:8091`** on a phone on the same wifi to drive the rover
by hand. A top button toggles **AUTO** (default — nav2/brain drive, teleop is
hands-off) vs **MANUAL** (you drive; hold-to-move, and it hard-cancels any active
nav2 goal so there's no contention). Runs on the Pi5 (`langrobo-teleop`). See
[`pi5/teleop/`](pi5/teleop/README.md).

## What the brain can use (all verified end-to-end)

| Capability | Topic contract | Backed by |
|---|---|---|
| look() — see through the camera | `/camera/color/image_raw/compressed` 2Hz JPEG | detections_3d |
| Known objects in the map | `/vision/detections_3d` JSON (map frame) | YOLOv8n + depth |
| Visual servoing ("drive at the cup") | `/vision/target` → `/vision/target_result` (`found`, `bearing_x`, `rel_size`) | detections_3d target finder — works without SLAM |
| VLM pixel grounding ("that thing there") | `/vision/pixel_query` → `/vision/pixel_result` + `/vision/pixel_goal` | pixel_to_goal + D555 depth |
| Map navigation | `/navigate_to_pose` action, `/goal_pose` | Nav2 + nvblox costmaps |
| Wheels | `/cmd_vel` (Twist, ≤0.30 m/s) | ESP32 via micro-ROS |

Pi5-side client for scripts/tests: `pi5/langrobo_client.py`
(`look | objects | ground u v | go x y | go-pixel u v | go-object label | cancel | status`).

## Mounting the camera on the rover — checklist

1. Mount the D555 facing **forward, level**; power + ethernet to the Jetson.
2. Measure camera position from the **wheel-axle midpoint** (metres):
   forward = x, left = y, up = z — edit the static TF in
   `scripts/run_d555_stereo.sh` (`--x 0.10 --z 0.25` are placeholder values).
3. `robot restart` — fresh origin at the rover's parking spot.
4. **Verify rotation direction** (one-time): send
   `ros2 topic pub -r 10 --times 15 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.4}}"`.
   The rover must turn **LEFT (counter-clockwise from above)**. If it turns
   right, flip a side's `*_MOTOR_DIR` (and matching `ENC_*_DIR`) in
   `ESP_32_frimware/rover_firmware_v2.ino` (Pi5 repo `pi5_ros2_ws`) → reflash.
   Drivetrain reference: `orin-nav-stack/firmware/HARDWARE.md`.
5. Drive a slow manual square (Telegram: "turn left", "go forward") and watch
   RViz — the orange path should mirror what the rover did.
6. First autonomous test: `robot status` all green → Telegram "go one meter
   forward", then "go near the <visible object>". Keep a hand near the power.

## Repo map

| Path | What |
|---|---|
| `scripts/run_all.sh` / `status_all.sh` / `stop_all.sh` | orchestrator (systemd + the Pi5 `robot` CLI call these) |
| `config/localization` | one-word localization backend switch: `rtabmap` \| `cuvslam` |
| `scripts/run_localization.sh` | backend dispatcher (called by run_all) |
| `scripts/run_rtabmap.sh` | RTAB-Map backend: rgbd_odometry + rtabmap on infra1 + raw depth |
| `scripts/run_cuvslam_sidecar.sh` | cuVSLAM backend (parked — see Known quirks) |
| `scripts/cmd_vel_deadband.py` | motor dead-zone shim: `/cmd_vel_nav` → effective-PWM `/cmd_vel`. REVISIT/REDUCE now that firmware v2 is closed-loop PID (`orin-nav-stack/firmware/HARDWARE.md`) |
| `scripts/run_*.sh` | other stages (camera, nvblox, nav2, vision, rviz) |
| `docker/cuvslam-sidecar/` | the Isaac ROS 3.2.6 Humble sidecar that makes cuVSLAM run on Orin/JP7 |
| `langrobo_perception/` | ROS nodes: detections_3d (YOLO + target finder), pixel_to_goal |
| `pi5/` | Pi5-side: `robot` CLI, `langrobo_client.py` (deploy: `scripts/deploy_pi5.sh`) |
| `systemd/langrobo-perception.service` | Jetson boot service |
| `config/` | Nav2, nvblox, RViz configs (`*_sim.yaml` = old Gazebo profile, still usable via `perception.launch.py mode:=sim`) |
| `CUVSLAM_ORIN_GUIDE.md` | architecture, performance, integration contracts |
| `ISSUES_AND_SOLUTIONS.md` | every problem hit and how it was solved |

## Known quirks (read before debugging)

- **D555 restarts**: never restart a healthy camera driver — the D555's DDS
  goes stale and only a **physical power unplug (15s)** recovers it.
  `robot restart` deliberately leaves the driver running.
- **Camera "not found" after a Jetson reboot**: check `ip link show enP8p1s0`
  says **mtu 9000** BEFORE power-cycling — the NM profile once reverted it to
  1500, which breaks DDS device discovery while ping still works
  (ISSUES_AND_SOLUTIONS.md Part 3). Fixed persistently 2026-07-16.
- **No discovery server anywhere** — everything is multicast (verified across
  this WiFi AP). A discovery-server client cannot see the D555 (a raw DDS
  participant) and silently splits the ROS graph (cost us half a day).
- **IMU fusion is OFF**: default noise params diverge ~850m stationary;
  visual-only is 0.000m. Calibrate before enabling (guide §7).
- **pkill -f** matches its own docker-exec shell — kill by comm name.
- **Driver params silently dropped at launch**: `depth_module.*` params only
  exist after the device connects, so launch args like `emitter_enabled:=0`
  are DISCARDED with a buried warning. run_all/run_d555_stereo enforce
  `emitter_enabled=false` (dot pattern corrupts feature tracking) and
  `global_time_enabled=false` (per-stream clock models drift the stereo
  stamps apart) at runtime on EVERY start. Same trap family:
  `align_depth.enable` and `pointcloud.enable` accept a value but publish
  nothing on the DDS driver — RTAB-Map uses infra1 + raw depth instead
  (natively registered, no alignment needed).
- **Moved the rover by hand? `robot restart`.** Any tracker treats a
  kidnap as chaos; fresh origin is cheap.
- **Nav goals must carry a ZERO timestamp** ("use latest TF"): Nav2
  re-transforms the original stamp on every replan, so `now()` stamps age
  out of the 10s TF cache mid-drive and abort with "extrapolation into the
  past". langrobo_client and the Pi5 brain already do this — keep it that
  way in new tools.
- **Motor dead zone** (old open-loop era): below ~60% PWM the wheels hummed but
  didn't turn, so `cmd_vel_deadband.py` rescales Nav2's output. Firmware v2
  (`rover_firmware_v2.ino`, Pi5 repo `pi5_ros2_ws`) is now closed-loop PID with a
  breakaway floor (`orin-nav-stack/firmware/HARDWARE.md`), so the shim should be
  reduced/removed — re-tune, then point the collision monitor's `cmd_vel_out_topic`
  back to `cmd_vel`.
- **Jazzy↔Humble DDS is NOT trustworthy** (relevant only if reviving the
  cuVSLAM sidecar): /tf_static never deserializes across it and large
  image messages drop even with 16MB buffers ("sequence size exceeds
  remaining buffer"). Anything critical must live in ONE distro.
