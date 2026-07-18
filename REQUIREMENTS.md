# LangRobo — Requirements & Roadmap

Written 2026-07-17 from the owner's requirements (originally typed into
`pi5/requirements.md`). This is the cross-session plan; progress is tracked
in the session task list (Phases 0–5).

## Requirements (owner's words, structured)

1. **Telegram end-to-end**: ask anything on Telegram → the robot responds.
   "Go to bottle" → robot navigates near the bottle.
2. **No crashing**: the rover must move autonomously and *smoothly* without
   hitting walls. (During testing it hit walls and broke things — this is
   the top priority.)
3. **Wheel slip**: if wheels skip/slip, the robot must still work, using the
   D555's built-in IMU.
4. **Sim/real parity, prod-grade**: a working simulation (laptop) and the
   real robot running the *same* stack — localization, nvblox, Nav2 —
   differing only in simulated-vs-real robot.
5. **Pi5**: proper code with all fixes consolidated (`pi5/` in this repo is
   the source of truth; deploy with `scripts/deploy_pi5.sh`).
6. **Cleanup**: remove unnecessary files from all three machines — Jetson,
   Pi5, simulation laptop.
7. **Constraint — ai_stack**: the previous ai_stack (`/home/rakhi24/robot`
   folder + docker images) stays. Do not delete the latest image
   (`ai_stack:dev-1.0.6`).

## Why it hits walls — root-cause analysis (2026-07-17)

Ordered by likely contribution:

1. **The deadband shim forbids slow motion.** `scripts/cmd_vel_deadband.py`
   rescales every nonzero command onto [0.20, 0.30] m/s and amplifies wz up
   to 1.4 rad/s. MPPI's gentle approach/correction speeds become lunges, on
   an *open-loop* drivetrain (commanded ≠ physical speed). Near a wall there
   is no such thing as a small correction.
   *Fix:* the firmware **source** on the Pi5
   (`~/ros2_ws/ESP_32_frimware/rover_firmware.ino`) already contains the
   per-wheel PWM remap (`PWM_MIN 130`), correct `AGENT_IP 192.168.1.16`,
   retry-forever sessions, OTA, and a 500 ms watchdog — inspected
   2026-07-17. Unknown: whether the ESP32 is *flashed* with it (rover was
   powered off; couldn't check). **If it IS flashed and the shim is also
   running, deadband compensation is applied twice** — shim floors at
   0.20 m/s (~65 %), firmware maps that to ~83 % PWM — which alone explains
   lunging into walls.
   *Procedure when the rover is next powered:* `ping rover-esp32.local`
   (the OTA hostname only exists in the new firmware). Responds → new
   firmware is on: remove the shim (drop the deadband block from
   `scripts/run_nav2.sh`, set collision monitor
   `cmd_vel_out_topic: "cmd_vel"` in `nav2_real.yaml`). No response →
   flash over USB first (WiFi password + OTA password must be filled in
   locally, never committed), then remove the shim.
2. **Unknown space is treated as free.** `track_unknown_space: False` in the
   costmaps and `allow_unknown: true` in NavFn mean Nav2 confidently plans
   through space the narrow forward-facing D555 has never seen — i.e.
   straight into unmapped walls.
   *Fix:* `track_unknown_space: True`, `allow_unknown: false`; the mission
   flow does a look-around (rotate scan) before/while driving into unknown
   territory.
3. **Blind recovery behaviors.** `backup` and `spin` from the behavior
   server move into space the camera cannot see (no rear/side sensing at
   all).
   *Fix:* disable backup or cap it to a few cm; cap spin; prefer
   wait+replan.
4. **Thin collision-monitor margins.** Stop-only circle of 0.18 m (footprint
   half-length is 0.15 m) fed by a ~5 Hz pointcloud; at real speeds the
   robot covers ~6 cm between updates.
   *Fix:* add an outer *slowdown* polygon, enlarge the stop zone, revisit
   `min_points`/height band.
5. **Placeholder geometry.** The Nav2 footprint is still the sim X3's dims
   ("MEASURE the real chassis" comment in `nav2_real.yaml`), and the
   camera→base static TF in `run_d555_stereo.sh` is placeholder
   (`--x 0.10 --z 0.25`). If the real chassis is larger, every clearance
   number above is optimistic.
   *Fix:* measure and update (owner action — needs a tape measure).

## Plan

- **Phase 0 — decisions & measurements**: localization backend (see below),
  chassis footprint + camera TF measurement, laptop access, cleanup
  confirmation.
- **Phase 1 — anti-crash** (fixes 1–5 above), each validated with a
  supervised low-speed run. Already applied in `config/nav2_real.yaml`
  (2026-07-17, needs `robot restart` + supervised test): global costmap
  `track_unknown_space: True` + planner `allow_unknown: false` (no more
  planning through unseen space), collision-monitor stop ring 0.18→0.25 m
  plus a 0.45 m slowdown ring, blind `backup`/`drive_on_heading` recoveries
  removed, MPPI reverse capped to −0.10 m/s. Still open: firmware
  flash-state check + shim removal (above), real footprint + camera TF
  (owner measurement).
- **Phase 2 — IMU**: enable `enable_motion:=true`
  (`/camera/camera0/motion/sample`, 200 Hz), calibrate noise against
  `motion/imu_info` (the earlier uncalibrated attempt drifted ~850 m),
  fuse with visual odometry, add slip/bump detection (commanded motion vs
  IMU disagreement → stop + replan).
- **Phase 3 — sim/real parity**: architecture built 2026-07-17, see
  **`SIM_REAL_PARITY.md`**. The simulator impersonates the hardware behind
  a verified topic contract; there is NO separate sim stack/config. Done on
  the Jetson side: `config/hardware` mode switch, mode-aware `run_all.sh`
  with both-direction interlocks, `scripts/run_robot_tf.sh` (shared robot
  geometry), `scripts/check_contract.sh` (the promotion gate — validated
  live: 27/28, only failure = rover powered off), D555 ground truth
  snapshotted in `config/d555_contract/`. Remaining: build `rover_sim` v2
  on the laptop per the spec in SIM_REAL_PARITY.md §3 (blocked on laptop
  SSH access), then run the §4 promotion gate.
- **Phase 4 — Telegram hardening**: object-search behavior when the label
  isn't visible, standoff parameter, LLM latency, ESP32 reflash, Pi5 code
  consolidation.
- **Phase 5 — structure & cleanup**: consistent layout, single source of
  truth per machine, remove confirmed-unnecessary files/images on Jetson,
  Pi5, laptop (ai_stack untouched).

## Localization backend — the one requirement conflict

The requirement says cuVSLAM in sim and real. On this hardware that is not
currently possible: NVIDIA ships no Orin cuVSLAM for JetPack 7 (Thor-only
SVE binaries — SIGILL), and the Humble-sidecar workaround explodes under
motion (frame/TF starvation across the container boundary). Full evidence:
`ISSUES_AND_SOLUTIONS.md` Parts 1 & 9. RTAB-Map is the verified-under-motion
production tracker and the backend is already pluggable
(`config/localization`: `rtabmap` | `cuvslam`).

**Recommendation:** RTAB-Map on *both* sim and real (true parity), keep the
pluggable switch, re-adopt cuVSLAM the day NVIDIA ships an Orin/JP7 build
(re-adoption gate documented in `CUVSLAM_ORIN_GUIDE.md`). nvblox and Nav2
stay identical everywhere regardless.

## Decisions (owner, 2026-07-17)

- **Localization: RTAB-Map everywhere** (sim + real); `config/localization`
  switch stays for future cuVSLAM re-adoption.
- **Sim laptop: reached over SSH from the Jetson** (user@IP still to be
  provided).
- **Phase 5 deletions approved**: `isaac_ros:langrobo-nav-stack-1.3-dds`
  image + `isaac_ros_backup` container, and `ai_stack:dev-1.0.0` (repoint
  the exited `ai_stack` container to `dev-1.0.6` first). **Keep**
  `cuvslam-sidecar:3.2.6`, `ai_stack:dev-1.0.6`, `/home/rakhi24/robot`.

## Camera-only bring-up — step-by-step plan (owner, 2026-07-18)

Simulator PAUSED (resume list: `SIM_REAL_PARITY.md` §5). Next work session
walks the REAL stack up one subsystem at a time, with ONLY the D555
connected (no ESP32 rover, no Pi5 required). `config/hardware` = `real`.

Each step: start it, verify it, only then move on. All scripts run from the
Jetson host.

1. **Camera (D555)** — `scripts/run_d555_stereo.sh`
   Verify: infra1 ≥10Hz + depth ≥10Hz 16UC1 + color ≥5Hz, stamps on wall
   clock, emitter OFF (the script + run_all.sh enforce it).
   `check_contract.sh` sections "camera source" + "clock discipline" + "TF
   camera frames" must be green. If the driver won't stream ("No RealSense
   devices were found!") but the D555 still pings: it is almost always a
   stale host-side DDS session, NOT the device — do the clean DDS teardown
   (see "IMPORTANT correction" below) BEFORE any power cycle. Also check
   mtu 9000 on enP8p1s0.
2. **Localization (RTAB-Map)** — `scripts/run_localization.sh`
   (`config/localization` = `rtabmap`; cuVSLAM stays PARKED — SIGILL on
   Orin Nano, see `CUVSLAM_ORIN_GUIDE.md` re-adoption gate. Do not debug it
   camera-only; only re-test after an NVIDIA Orin/JP7 build exists.)
   Verify: /odom ≥1.5Hz, map→base_link TF resolves, no db corruption
   warning; wave the camera slowly — pose must track and not explode.
3. **nvblox** — `scripts/run_nvblox.sh`
   Verify: /nvblox_node/static_map_slice ≥1Hz; watch the mesh/costmap in
   `run_rviz_real.sh` (fixed 2026-07-18: it no longer sets the old
   discovery server).
4. **Nav2** — `scripts/run_nav2.sh`
   Verify: "Managed nodes are active" in /tmp/nav2.log, /navigate_to_pose
   action listed. Send a goal from RViz: expect a PLAN + /cmd_vel output
   (nothing moves — no rover; /cmd_vel-no-consumer FAIL in
   check_contract.sh is EXPECTED camera-only).
5. **Vision AI** — `scripts/run_vision_ai.sh`
   Verify: /vision/detections_3d flowing when a bottle is in view,
   /vision/pixel_result answers. (Needs Mac mini VLM up for grounding.)
6. **Full pass** — `scripts/run_all.sh` then `scripts/check_contract.sh`:
   everything green EXCEPT the /cmd_vel consumer line (no rover yet).

Then, rover day (separate session): power ESP32 → `ping rover-esp32.local`
→ flash state check (REQUIREMENTS "Firmware finding") → deadband
single-application test → supervised low-speed mission.

## Camera-only bring-up — RESULTS (2026-07-18, D555 only, no rover)

Walked the real stack up subsystem by subsystem. All five come up and work:

| Step | Subsystem | Verdict | Evidence |
|---|---|---|---|
| 1 | D555 camera | ✅ | infra1 mono8 @30Hz, depth 16UC1, color @30Hz, stamps = wall clock (0s delta), all camera TF frames resolve, emitter+global_time OFF |
| 2 | RTAB-Map | ✅ | /odom 3–8Hz, odom quality 250–310 features (healthy, not lost), map→odom→base_link chain resolves, no DB corruption |
| 3 | nvblox | ✅ | static_map_slice @8.7Hz, CUDA streams clean |
| 4 | Nav2 | ✅ | active in ~3s, /navigate_to_pose up, custom no-blind-backup BT loaded, collision monitor SlowZone+SafeZone active; test goal → controller emits real cmd_vel |
| 5 | Vision AI | ✅ | YOLO yolov8n on CUDA, detections_3d @~2Hz, look feed @2Hz, pixel_result endpoint present |

`check_contract.sh`: **25/28**. The 3 fails are understood:
- `/cmd_vel has no consumer` — EXPECTED (no rover connected).
- `/vision/detections_3d rate none` — transient: it stalled during the 2-min
  check under peak load; re-measured clean at 2.2Hz. Not a real fault.
- `depth rate 2.4Hz < 10Hz floor` — REAL, see below.

### Bug fixed during bring-up — collision monitor froze the robot
`config/nav2_real.yaml` collision_monitor had `source_timeout: 1.0` but its
source (nvblox `back_projected_depth`) measured 2.8–3.3Hz with **max gaps up
to 1.5s**. Every slow frame tripped "stop due to invalid source", holding
cmd_vel at zero forever — would have frozen the real rover. Raised to
`source_timeout: 2.5`. After the fix: 0 invalid-source stops, controller
emits real velocity commands, only clean SlowZone slowdowns remain.

### Real finding — depth rate degrades under CPU load
Depth swings 2.4–8Hz (below the 10Hz contract floor) and tracks system load;
infra1 stays rock-solid at 30Hz. Root causes, in order of impact:
1. **Orin Nano CPU oversubscription** — load avg ~15 on 6 cores. Top procs:
   rgbd_odometry 65%, detections_3d 46%, realsense 36%, rtabmap 32%,
   nvblox 32%. Part of this is DEV TOOLING (`code`/VS Code 21%, `claude`
   15%) that is absent on a headless production run — production load is
   meaningfully lower than measured here.
2. **infra2 dead weight** — the second stereo-IR channel had ZERO
   subscribers (cuVSLAM parked; RTAB-Map is RGBD) yet streamed a full 30Hz
   896x504 mono channel. FIXED: `run_d555_stereo.sh` now
   `enable_infra2:=false` (re-enable on cuVSLAM re-adoption). Also dropped
   the dead `pointcloud.enable:=true` arg (produced no topic).

### VERIFIED FIXED (2026-07-18, same day) — depth clears the floor
Restarted the camera with `enable_infra2:=false` and re-measured under the
FULL stack (RTAB-Map + nvblox + Nav2 + vision AI all running):
- **depth: 13–15Hz** under load (was 2.4–8Hz with infra2 on) — clears the
  10Hz floor. ✅
- infra1: 28Hz, system load 5.2 (was ~15).
Baseline camera-only depth: 25.7Hz. The infra2-off change is confirmed.

### IMPORTANT correction — the "stale-DDS needs a power cycle" belief is wrong
The camera restart initially failed: driver logged "No RealSense devices were
found!" for ~20 min despite the D555 pinging at 0.15ms and (tcpdump-confirmed)
actively sending RTPS discovery multicast from 192.168.11.55:8888. TWO physical
power cycles did NOT fix it. The real cause was a **stale host-side DDS
participant holding the device's session** — the device won't re-advertise to a
new driver while an old participant lingers. Recovery that WORKED (no power
cycle needed):
  1. `scripts/stop_all.sh --full`   (tear down every ROS node)
  2. `ros2 daemon stop` + `rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps*`
     (166 stale segments had accumulated from repeated SIGKILLs)
  3. launch the camera driver ALONE, let it settle, THEN bring the stack back
So: prefer a clean host DDS teardown BEFORE resorting to a power cycle; and
avoid `pkill -9` on the driver (ungraceful exit is what strands the session).
The notes in `run_all.sh`/`stop_all.sh` overstate the power-cycle requirement.

GOTCHA when relaunching the camera by hand: `run_d555_stereo.sh` also starts
`run_robot_tf.sh` (base_link→camera0_link). Launching the bare rs_launch.py
skips it, so RTAB-Map can't find base_link (no /odom) and Nav2's costmap fails
to activate. Always run the script, or start `run_robot_tf.sh` alongside.

### Next
- Everything else is rover-day work (power ESP32, firmware/deadband, mission).
