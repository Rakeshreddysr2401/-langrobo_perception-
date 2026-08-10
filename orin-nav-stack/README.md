# Orin Nav Stack

**Autonomous rover perception + navigation, fully on-device.** GPU visual SLAM (cuVSLAM),
live 3D mapping (nvblox), path planning/driving (nav2), and object detection (YOLO) — all in
**one self-contained Docker image (`orin-nav:1.1`)** on a Jetson Orin Nano, with a LangGraph
"brain" on a Pi5 and a VLM on a Mac mini talking to it over ROS 2.

This folder is the **single source of truth**: every node, config, model and script the robot
needs lives here and is baked into the image. No external workspaces.

> **Bring-up / stop / fix:** see [`skills/`](skills/) (start & stop runbooks) and
> [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md). Fast health check: `./run_stack.sh status`.
> The D555 is an **ethernet/PoE DDS camera** (`192.168.11.55`), not USB — if it logs
> `No RealSense devices were found`, power-cycle it, then `./run_stack.sh cam`.

---

## 1. What this project is

A small tethered rover that can:
- **Know where it is** without GPS/markers — cuVSLAM stereo visual odometry (30 FPS, GPU)
- **Build a live 3D map** of any new place as it moves — nvblox (mesh + obstacle costmap)
- **Drive itself to goals** — nav2 plans on the nvblox map and outputs motor commands
- **See objects** — YOLO answers "where is the bottle?" with bearing + 3D position
- **Take orders from the brain** — the Pi5 (LangGraph) and Mac (VLM) send targets/goals over ROS

The breakthrough that enables it: NVIDIA's Isaac ROS 4.x cuVSLAM binaries are Thor-only
(crash on Orin). We run the **standalone pyCuVSLAM cu12 wheel** — the Orin-native build —
inside the Jazzy container alongside nvblox/nav2. Modern stack, no version mismatches.

## 2. Hardware

| Piece | Details |
|---|---|
| Compute | Jetson Orin Nano 8 GB, JetPack 7.2 (L4T R39.2), Ubuntu 24.04, CUDA 13.2 |
| Camera | Intel RealSense **D555** (PoE/ethernet, DDS) at `192.168.11.55`, S/N 261522301413 |
| Rover base | L **30 cm** × W **17 cm**; camera 5 cm behind nose, 20 cm high; **powered by cable tether** |
| Motors | ESP32 (wifi, `rover-esp32.local` — DHCP, measured `192.168.1.12` on 2026-08-10; **always resolve the mDNS name, never type the IP**) via micro-ROS agent on the Pi5 (`:8888`); 500 ms cmd watchdog |
| Brain | Pi5 `192.168.1.16` (`rakhi24-desktop`, passwordless ssh) — LangGraph + micro-ROS agent |
| VLM | Mac mini on the LAN — vision-language decisions, sends pixel queries |
| Jetson net | wifi `192.168.1.15` (LAN/ROS), eth `192.168.11.70` + `192.168.2.20` (camera subnet) |

## 3. Architecture / data flow

```
                 ┌────────────────────── one container: orin-nav:1.1 ──────────────────────┐
 D555 (ethernet) │                                                                          │
  infra1+infra2 ─┼─► cuvslam_ros_node (FULL SLAM: loop closure + pose graph + map persist)  │
                 │      ├─► TF odom→base_link (smooth odometry) + /odom + /visual_slam/…    │
                 │      ├─► TF map→odom (LIVE SLAM correction — jumps on loop closure)      │
                 │      ├─► /slam/status (1 Hz JSON) + srv /slam/save_map /slam/localize    │
  depth ─────────┼─► nvblox_node ──────► 3D mesh + ESDF + occupancy grid (frame: odom)      │
  color ─────────┼─► detections_3d (YOLO) ─► /vision/target_result, /vision/detections_3d   │
                 │        └─► 2 Hz JPEG republish on /camera/color/image_raw/compressed      │
                 │            (feeds Pi5 brain look()/VLM — old topic name kept on purpose;  │
                 │            pausing YOLO also blinds the brain's vision)                   │
  imu (motion) ──┼─► imu_to_base ─► /imu/base                                               │
                 │        nav2 (planner+MPPI, SAFE BT) ─► cmd_vel_nav                       │
                 │        cmd_vel_deadband (PWM floor + SPEED CAPS) ─► /cmd_vel_shim        │
                 │        safety_guard (pose watchdog + nvblox virtual bumper) ─► /cmd_vel  │
                 └──────────────────────────────────────────────────────────────────────────┘
   base_link→camera0_link (static x0.10 z0.20) completes the TF tree (map→odom is dynamic!)
   /cmd_vel ─► Pi5 micro-ROS agent ─► ESP32 ─► wheels      Pi5/Mac ─► /vision/*, nav goals
   SLAM maps persist on the host: ~/orin-nav-stack/maps/  (mounted at /maps, gitignored)
```

## 4. Quick start (nothing autostarts at boot — you run this)

```bash
cd ~/orin-nav-stack
./run_stack.sh up        # camera + cuVSLAM + nvblox        (~40 s)
./run_stack.sh nav2      # navigation (safe no-recovery BT) (~20 s)
./run_stack.sh vision    # YOLO + goal nodes + motor shim
```
Rover must be power-cycled once if the Pi5/agent restarted since (firmware reconnect bug).

## 5. Command reference (`run_stack.sh`)

| Command | What it does |
|---|---|
| `up` | Start container + D555 (stereo/depth/color/imu, **emitter OFF** — ON breaks cuVSLAM scale ~4x, see §10) + TFs + cuVSLAM **full SLAM** + nvblox. Repo is bind-mounted RO over `/opt/orin-nav` (edits apply on restart, no rebuild); `./maps` mounted rw at `/maps` |
| `nav2` | nav2 with the **no-blind-recovery BT** (map frame comes live from cuVSLAM — no static bridge) |
| `vision` | YOLO hunt, pixel→goal, depth→cloud for the collision monitor, **safety_guard** (pose watchdog + virtual bumper), imu relay, 2 Hz look feed → Pi5 brain |
| `stop` | **E-STOP** — kills every motion node + publishes zero velocity |
| `down` | **Hard stop** — removes the container (watchdog halts rover ≤0.5 s) |
| `remap` | Fresh map + pose→(0,0,0): restarts cuVSLAM+nvblox **without touching the camera** |
| `rviz` | RViz (nav2 view) on the Jetson monitor |
| `logs <name>` | Tail `/tmp/<name>.log` in the container: `realsense cuvslam nvblox nav2 detections_3d pixel_to_goal cmd_vel_deadband safety_guard imu_to_base` |

## 6. Common tasks (all commands run on the Jetson host)

**Health check**
```bash
docker exec orin_nav bash -lc 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0
  ros2 topic hz /odom                       # ~20-30 Hz = cuVSLAM alive
  ros2 topic hz /nvblox_node/static_occupancy_grid   # ~9 Hz = mapping
  ros2 topic info /cmd_vel'                 # "Subscription count: 1" = rover connected
```

**Send a coordinate goal** (⚠️ rover moves — supervise, pause YOLO first: `docker exec orin_nav pkill -f detections_3d`)
```bash
docker exec orin_nav bash -lc 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 0.8, y: 0.0}, orientation: {w: 1.0}}}}"'
```

**Hunt + approach a visible object** (⚠️ rover moves)
```bash
docker exec orin_nav bash -lc 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0
  python3 /opt/orin-nav/nodes/visual_approach.py bottle --vx 0.26'
```

**Calibrated straight/rotate test** (⚠️ rover moves; speeds <0.2 stall — use ≥0.25)
```bash
docker exec orin_nav bash -lc 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0
  python3 /opt/orin-nav/nodes/drive_test.py straight 50 0.28'   # 50 cm forward
```

**Manual drive + map an area — from your phone** (Pi5 mobile teleop → [`../pi5/teleop/`](../pi5/teleop/README.md))
Open `http://192.168.1.16:8091` on a phone on the same wifi; tap **MANUAL** and hold the D-pad
to drive (dead-man — release = stop). MANUAL hard-cancels any active nav2 goal; **AUTO**
(default) hands `/cmd_vel` back to nav2/brain. To map a fresh area: mount the camera, then
`./run_stack.sh remap` (fresh 0,0,0 origin + empty map) and drive **slowly** in MANUAL — nvblox
builds the map as you move; watch it fill on the laptop RViz (§7), then save it (below). ⚠ Teleop
publishes **directly** to `/cmd_vel` (bypasses safety_guard — drive by sight); keep speeds low and
turns to **short taps** (fast pivots explode cuVSLAM tracking, §10). Runs as the always-on
`langrobo-teleop` service on the Pi5.

**Ask YOLO what it sees**
```bash
docker exec orin_nav bash -lc 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0
  ros2 topic pub --once /vision/target std_msgs/msg/String "{data: bottle}"
  ros2 topic echo --once /vision/target_result'
```

**Fresh map / reset pose after drift or a pose jump** → `./run_stack.sh remap`

**SLAM map: save / relocalize / status** (maps live on the host in `~/orin-nav-stack/maps/current`).
`/slam/save_map` saves BOTH layers: cuVSLAM features (`data.mdb`, for relocalization) and the
nvblox walls (`nvblox_map.nvblx`, what you see in RViz). ⚠ Wall-map RELOAD across sessions is
not wired yet — nvblox is anchored to `odom`, which is different each boot; aligning reloaded
walls needs the nvblox global_frame=map switch + a live loop-closure test (next driven session).
```bash
docker exec orin_nav bash -lc 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0
  ros2 topic echo --once /slam/status                        # lc/pgo health, corrections
  ros2 service call /slam/save_map std_srvs/srv/Trigger      # persist current SLAM db
  ros2 service call /slam/localize std_srvs/srv/Trigger'     # relocalize in saved map (2 m search)
```

**Safety guard** — `/safety/state` says `ok`, `FWD_BLOCKED:depth <x>m`, or `TRIPPED:<reason>`.
Trips auto-clear after 10 s of sane pose. Manual latch: `ros2 topic pub --once /safety/trip
std_msgs/msg/Bool "{data: true}"` (release with `false`). While tripped: nav goals cancelled,
wheels zeroed. THREE bumper layers, all forward-only (rotate/reverse always free):
1. **Depth bumper** (added 2026-07-19 after user watched nav head for a person): central-ROI
   min depth < 0.50 m → forward blocked BEFORE the target enters the D555's 0.4 m blind zone;
   a mostly-invalid ROI (something already in the blind zone / lens covered) also blocks.
   Verified live with injected 0.3 m frames → `FWD_BLOCKED:depth 0.30m`.
2. **nvblox virtual bumper**: occupied cell in the 0.35×0.32 m box ahead (remembers walls
   the camera saw even once inside the blind zone).
3. Costmap inflation widened 0.35→0.45 m (cost_scaling 10→6) so planner/MPPI keep distance.
RViz shows the bumper as a box in front of the robot: **green = clear, red = blocked**.

## 7. Viewing (RViz / laptop)

- On the Jetson monitor: `./run_stack.sh rviz` (close it when navigating — RAM).
- From the **HP laptop (`192.168.1.10`, HP Pavilion 14, hostname `rakhi24`, passwordless SSH
  from the Jetson)** — already installed (2026-07-19).

  ⚠ **The IP moved.** This was documented as `192.168.1.12` until 2026-08-10; DHCP has since
  given `.12` to the **ESP32** (`rover-esp32.local` resolves there, ssh refused). Everything
  on this LAN is DHCP — **verify before trusting any hardcoded IP here**. Fastest check:
  `./run_stack.sh view`, which resolves the laptop and tells you if someone is logged in.

  **Step 1: log into the laptop desktop physically** (see the warning below), then run:
  ```bash
  ~/rover_view.sh        # sources Jazzy, domain 0, opens ~/laptop_view.rviz
  ```
  ⚠ **Laptop must be logged in** — at the Ubuntu login screen nothing can display (a
  remotely-launched rviz runs invisibly, with **no error**). Verify from the Jetson with
  `ssh rakhi24@192.168.1.10 loginctl list-sessions`: if the only seat0 session belongs to
  `gdm`, **nobody is logged in** and RViz cannot appear no matter what you run.
  Check rviz is really up with `pgrep -x rviz2` — **never `pgrep -f rviz2` over ssh: it
  matches your own ssh command line** (same self-match trap as the run_stack.sh kill loops).
  (Any other Jazzy laptop: `scp rakhi24@192.168.1.15:~/orin-nav-stack/config/laptop_view.rviz /tmp/`,
  `unset ROS_DISCOVERY_SERVER; export ROS_DOMAIN_ID=0; rviz2 -d /tmp/laptop_view.rviz`.)
  Shows: TF frames, live nvblox walls, `/plan`, the `/odom` "pencil trail" (300 arrows),
  optional nav2 costmap (durability already set to TRANSIENT_LOCAL) + camera image.
  ⚠ The **"2D Goal Pose" toolbar button publishes a REAL nav goal** — if nav2 is up,
  the rover MOVES. Treat it like a motion command (§8 rules apply).
  If nothing shows: laptop must be on the same WiFi, multicast (no discovery-server
  env), and `ros2 topic list` should show the stack's topics first.
- Mac: Foxglove Studio → `ws://192.168.1.15:8765` after starting
  `ros2 run foxglove_bridge foxglove_bridge` in the container (works, but laggy over wifi).

## 8. SAFETY — read before moving the rover

1. **Tethered power cable**: blind rotations tangle it. The BT (`config/bt_navigate_to_pose.xml`)
   has **NO Spin/BackUp** — never replace it with nav2's default (caused a live incident 2026-07-19).
2. **Speed caps live in `config/nav2.yaml`** — MPPI `vx_max: 0.30` / `vx_min: -0.10` /
   `wz_max: 1.0`, and velocity_smoother `max_velocity: [0.25, 0.0, 1.0]`.
   ⚠ `nodes/cmd_vel_deadband.py` is **RETIRED** (2026-08-10) and is no longer started by
   `run_stack.sh vision`. It was built for the old open-loop L298N firmware and re-floored
   every command to vx ≥ 0.20 / wz ≥ 0.80, so a fine MPPI correction of (0.02, 0.05) reached
   the wheels as (0.21, 0.80) — a hard swerve. That was the weave. **There are no hardware
   speed floors any more**: firmware v2's 50 Hz encoder PID with `gMinDuty` does the
   static-friction job, so low speeds are now genuinely controllable.
3. **E-stop**: `./run_stack.sh stop`; if in doubt `./run_stack.sh down` (always works).
4. **Pause YOLO during motion tests** — CPU overload causes cuVSLAM pose jumps → nav chaos.
   Kill by full path (`pkill -f "nodes/detections_3d.py"`) — a bare `-f detections_3d` inside
   `docker exec bash -c` matches its own wrapper shell and kills the wrong process.
5. **Keep YOLO paused until the pose is verified stable AFTER motion too** (incident
   2026-07-19: restarting YOLO seconds after a nav goal finished — model-load CPU burst —
   blew an already-shaky pose up to −34 m). Sequence: motion done → watch `logs cuvslam`
   stay flat ~10 s → then restart YOLO. Note the rover coasts ~15 cm odom past the
   controller's stop (momentum + 500 ms watchdog) — budget for it near obstacles.
6. **Emergency zero-velocity when the container is already down**:
   ```bash
   docker run --rm --network host --entrypoint bash orin-nav:1.1 -lc \
     'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0
      ros2 topic pub -r 20 -t 60 /cmd_vel geometry_msgs/msg/Twist "{}"'
   ```
   (`--entrypoint bash` is required — the image entrypoint mangles a plain `bash -lc`.)
7. Rover moves only with an operator watching the tether.
8. **safety_guard is the last gate** on nav-driven motion (`/cmd_vel_shim`→`/cmd_vel`): it
   cancels goals + zeroes wheels on pose jumps >0.35 m, |z|>0.25 m, or tilt >0.7 rad
   (auto-clears after 10 s sane), and blocks forward vx when nvblox shows an obstacle in
   the 0.35 m box ahead. ⚠ `drive_test.py` and manual `ros2 topic pub /cmd_vel` **bypass
   the guard** — those remain fully operator-supervised.

## 9. Troubleshooting

| Symptom | Cause → fix |
|---|---|
| realsense log: "No RealSense devices" | D555 DDS session stuck from a previous run → `./run_stack.sh down`, wait 30 s, `up`. If still: **power-cycle the D555, unplugged ≥30–60 s** |
| `/cmd_vel` Subscription count: 0 | ESP32 lost its agent session (old firmware bug) → **power-cycle the rover**; verify on Pi5: `journalctl -u langrobo-microros -f` shows "session established" |
| Robot pose suddenly huge (e.g. +50 m) | cuVSLAM jumped after frame gaps (CPU overload) → `./run_stack.sh remap`; keep YOLO paused while navigating |
| nav2 goal REJECTED/ABORTED instantly | Goal in unseen space, or pose corrupted (see above). Map the area first (slow sweep); `allow_unknown: true` is already on |
| Goal "SUCCEEDED" but no motion | NavFn tolerance quirk when goal unreachable — treat as failure; re-map and retry |
| `ros2 topic hz` says no data but things work | The CLI probe is flaky under load — trust `logs` counters and downstream consumers |
| MPPI "Optimizer fail" / control loop misses 20 Hz | CPU saturated → pause YOLO, close RViz, retry |
| Wheels hum but no motion | Was the open-loop L298N torque floor. Firmware v2's encoder PID + `gMinDuty` handles breakaway, so this should no longer happen — if it does, suspect **battery sag** (row below) or a wheel-telemetry problem (`ros2 topic hz /wheel_state` should be ~20 Hz; measured 2026-08-10 it runs at **10 Hz** — see `learn/03-imu.md`) |
| Nav goal SUCCEEDED but rover stopped short/long of it | Pose inflated during motion (seen 2026-07-19: reported 1.0 m while physically shorter) — nav2 closes the loop on the *reported* pose. Re-map before the next goal; if it repeats, treat as tracking degradation, not calibration |
| `remap` prints nothing / container gone after it | **FIXED 2026-07-19 (root cause)**: the kill-loop's `pgrep -f cuvslam_ros_node.py` matched the wrapper shell's own cmdline → kill -9'd itself → exit 137 → `set -e` aborted before restarting nodes. Patterns are now bracketed (`[c]uvslam…`). If you ever add a kill-loop to run_stack.sh, use `pgrep -f "[x]name"` |
| Pi5 brain says "I cannot see right now" | look() feed comes from detections_3d (2 Hz JPEG republish on `/camera/color/image_raw/compressed`) — it is DOWN whenever YOLO is paused or the vision layer isn't up |
| Pose explodes during forward drive (z≠0, teleport; guard trips) | Fast breakaway (~0.5 m/s) + LC burst starves the tracker. Recover: `remap`. Prevent: short bursts, nav2 off while teleop-mapping, keep node defaults `async_sba=true` `lc_throttle_ms=2000`. Durable fix = ESP32 flash (controllable slow speed) |
| Laptop RViz "running" but nothing on screen | Laptop at the login screen (log in first! check `loginctl list-sessions` — only `gdm` on seat0 = nobody logged in), or your `pgrep -f rviz2` matched itself over ssh — use `pgrep -x rviz2`. Launch as the logged-in user: `bash ~/rover_view.sh` on the laptop |
| Can't reach the laptop / ssh refused | **The laptop is `192.168.1.10`, not `.12`** — DHCP gave `.12` to the ESP32 (2026-08-10). Run `./run_stack.sh view`, which resolves it and checks the login state for you |
| Wheels stall even at cmd 0.30 | Battery sag — breakaway threshold rises as the pack drains. Charge/swap; no software fix until the ESP32 PWM-floor firmware is flashed |

## 10. Facts & calibration (measured)

- **Tape-verified accuracy**: commanded 1.00 m → real 0.95–1.00 m (closed-loop on cuVSLAM).
- **Motors are binary + battery-dependent** (first mapping run, 2026-07-19 night): stall-hum
  or ~0.5 m/s breakaway, nothing between. Fully charged, cmd 0.25 breaks away fast; drained
  (same evening), even cmd 0.30 stalls. Open-loop PWM can't compensate → ESP32 firmware
  flash (`e282a13`, Pi5 repo) is the real fix.
- **Fast translation explodes cuVSLAM tracking** (~0.5 m/s real, close-range floor features,
  30 fps): two pose explosions during the mapping run, both on straight bursts / at
  loop-closure revisit moments. Mitigations that WORK: `async_sba=True` +
  `lc_throttle_ms=2000` (now node defaults — after tuning, a rotate out-and-back returned
  to 0.000/−0.001 m vs 30 cm drift before), stop nav2 while teleop-mapping (CPU headroom),
  short motion bursts. safety_guard caught both explosions live (trip + auto-clear verified).
- cuVSLAM (and RTABMap before it) under-read real distance ×~1.2 → assumed to be **D555
  factory stereo calib scale**; `CAL=1.2` inside `drive_test.py` compensates. Baseline
  0.0949 m, 896×504@30 IR.
  ⚠ **Suspect this now.** That ×1.2 was measured with the **emitter ON**, which is itself a
  scale bug (previous bullet). With the emitter OFF, 100 cm read 97 cm — i.e. ~3% error, not
  20%. So `CAL=1.2` is very likely over-correcting today. **Re-measure before trusting
  `drive_test.py` distances** (`learn/02-odometry.md` does exactly this).
- IR **emitter OFF** — `depth_module.emitter_enabled:=0` in `run_stack.sh launch_cam`.
  **Corrected 2026-08-09; this line previously said "emitter ON, does not hurt cuVSLAM" and
  that was wrong.** The projector is rigidly mounted on the camera, so its dot pattern is
  re-painted from the camera's own viewpoint every frame: on a low-texture/reflective floor
  the dots stay ~fixed in the image while the rig *translates*, and cuVSLAM (which tracks
  infra1/2 features) sees almost no motion. Measured — emitter ON: a 100 cm push read as
  **26 cm** on `/odom` (~4× under). Emitter OFF: 100 cm → **97 cm** (3%).
  Passive-stereo DEPTH stays metric either way (140 cm wall → 1.40 m verified), so nvblox
  still works; emitter-off depth is just noisier on textureless surfaces. Localization beats
  perfect depth, so **the emitter stays off**. See memory: `emitter-breaks-cuvslam-scale`.
- D555 IMU: single combined `motion` stream 200 Hz; DDS driver reports identity extrinsics
  (why VIO is parked — needs Kalibr).

## 11. Files

```
Dockerfile                 # orin-nav:1.1 (FROM isaac_ros:cuvslam-unified + everything baked)
run_stack.sh               # all operations (see §5)
cuvslam_ros_node.py        # cuVSLAM wrapper — FULL SLAM (map→odom, save/localize services)
config/  nav2.yaml | nvblox.yaml | bt_navigate_to_pose.xml (safe BT) | laptop_view.rviz
nodes/   detections_3d | pixel_to_goal | cmd_vel_deadband | safety_guard | imu_to_base | visual_approach | drive_test
models/  yolov8n.pt
maps/                      # persistent SLAM maps (host side of /maps; gitignored)
standalone/                # ROS-free cuVSLAM dev tools (see standalone/README.md)
```
No rebuild needed for code/config edits — the repo is bind-mounted RO over `/opt/orin-nav`;
restart the affected node (`remap`, or re-run `vision`/`nav2` after `stop`). Rebuild the image
only for dependency changes: `docker build -t orin-nav:1.1 ~/orin-nav-stack`

**Do not delete** images `isaac_ros:langrobo-prod` / `isaac_ros:cuvslam-unified` — they are the
layer parents of `orin-nav:1.1` (until a lean rebuild replaces them).

## 12. Development, repo & host setup

- **Git home**: this directory lives in the `langrobo_perception` repo
  (`github.com/Rakeshreddysr2401/-langrobo_perception-`), branch **`dev_0.0.2_cuVslam_nav`**.
  `~/orin-nav-stack` is a **symlink** to `~/langrobo_perception/orin-nav-stack` — there is only
  ONE working copy; edit, then `git add/commit/push` from the repo.
- **Shell alias** (in `~/.bashrc`): `navstack up|nav2|vision|stop|remap|rviz|logs|down`.
- **isaac-ros CLI: installed (dormant) — credit where due**: it BOOTSTRAPPED
  this whole stack. The original `isaac-ros init docker`+`activate` dev container (Jazzy + the
  Isaac ROS apt repo) is where nvblox/nav2/realsense were installed — that environment became
  `isaac_ros:langrobo-prod`, still the base of `orin-nav:1.1`. Its output lives on in those
  image layers; the CLI itself is no longer needed at runtime, targets 4.x (cuVSLAM Thor-only),
  pointed at the deleted workspace, and re-pulled GB images when touched — hence removed.
  - **Rebuild without the CLI** (preferred for the lean image): in a Dockerfile add
    `deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-4.4 noble-jetpack main`
    then `apt install ros-jazzy-isaac-ros-nvblox ...` (same source the CLI used).
  - **CLI status (2026-07-19)**: reinstalled and configured (v2.3.0, release-4.4 repo) but
    DORMANT — do NOT run `isaac-ros init docker`/`activate` casually: it targets 4.x (cuVSLAM
    Thor-only), expects ISAAC_ROS_WS, and pulls GB images. Keep for future dev-env needs;
    watch isaac_ros_visual_slam issue #223 for Orin support in 4.x.
- **Host caveat**: apt has a pre-existing broken-dependency state (opencv/cmake from old host
  experiments). Everything here runs in Docker and is unaffected — but avoid
  `apt --fix-broken install` casually; it may churn many packages.

## 13. Roadmap / known limitations

1. **Free space under the robot**: camera can't see its own feet → coordinate goals rely on
   `allow_unknown: true`. Better: seed robot cell free / startup mapping arc.
2. **Loop closure + map save/reload — DONE 2026-07-19**: cuVSLAM runs full SLAM
   (planar constraints, loop closure, live map→odom). `/slam/save_map` + `/slam/localize`
   verified stationary; **relocalization against a real driven map still needs a live
   test** (a stationary 15 s map is too sparse to relocalize in — expected).
3. **VLM→goal refinement loop**: coarse far goal from a detection, refined as depth improves.
4. **Lean image**: rebuild from a minimal isaac-ros base (drops RTABMap etc., frees ~50 GB).
5. **ESP32 firmware flash** (committed, awaiting USB): fixes reconnect bug + PWM floor →
   deadband shim can then be removed.
6. VIO (IMU fusion) parked until Kalibr extrinsic calibration.

Old langrobo_perception source: `~/archives/langrobo_perception_20260719.tar.gz`.
