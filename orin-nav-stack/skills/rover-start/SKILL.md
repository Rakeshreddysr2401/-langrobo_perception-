---
name: rover-start
description: Full LangRobo rover bring-up — Jetson nav stack (SLAM+nav2+vision), Pi5 brain/micro-ROS check, ESP32 wheel link, HP-laptop RViz view. Use when the user says "start the rover", "bring the stack up", or similar.
---

# Rover full-system start (run from the Jetson, /home/rakhi24)

Machines: Jetson (this host, 192.168.1.15) · Pi5 `rakhi24@192.168.1.16` (passwordless SSH) ·
HP laptop `rakhi24@192.168.1.10` (passwordless SSH) · ESP32 rover `rover-esp32.local`
(= 192.168.1.12 as of 2026-08-10) · Mac VLM `singireddys-mac-mini.local` (mDNS ONLY, IP moves).

⚠ **This LAN is all DHCP and the addresses have already swapped once.** The laptop was
documented as `.12` and the ESP32 as `.11`; measured 2026-08-10, the **laptop is `.10`** and
the **ESP32 holds `.12`**. Resolve the mDNS name (`getent hosts rover-esp32.local`) rather
than typing an IP, and use `./run_stack.sh view` to locate the laptop.

## 1. Jetson perception + nav (order matters)

```bash
cd ~/orin-nav-stack
./run_stack.sh up        # camera + cuVSLAM FULL SLAM + nvblox   (~40 s)
./run_stack.sh nav2      # nav2, safe no-recovery BT             (~20 s)
./run_stack.sh vision    # YOLO + goal nodes + depth_to_cloud + safety_guard + look feed
./run_stack.sh status    # one-shot health of every layer (run after each step)
```

For visual+inertial fusion (EKF owns `odom->base_link`, adds `/odom/health` pose trust)
run `./run_stack.sh fuse` after `up`. ⚠ Both `fuse` and `remap` **wipe the nvblox map** —
save it first if it matters.
⚠ `cmd_vel_deadband` is **retired** (2026-08-10) and no longer part of `vision`.

`up` now GATES on the D555 actually streaming — it no longer plows ahead on a blind
sleep. If the camera doesn't come up it ABORTS with a power-cycle message (see §1a),
leaving the container running so `./run_stack.sh cam` can retry after you fix it.

Wait/verify (don't skip) — `./run_stack.sh status` measures **rates**, TF freshness and
pose trust, then flags anything running-but-wrong. Healthy looks like:
- `camera IR left ~29 Hz` · `camera depth ~25 Hz` (⚠ **<10 Hz => cuVSLAM freezes silently**)
- `cuVSLAM odom ~28 Hz` (0 Hz **with the camera alive** = cuVSLAM frozen → restart it)
- `nvblox slice ~9 Hz` · `TF map->odom` and `odom->base_link` both < 1 s old
- `ESP32 encoders ~20 Hz` (measured **10.0 Hz** on 2026-08-11 — half rate, known bug: `learn/03-imu.md`)
- `orin load1 < 8` — CPU is a safety property here; load starves the camera
- `cuVSLAM "slam_pose_ok": true` · `safety data: ok` · `/cmd_vel subs: 1` · `nav2 active`

It separates real problems ("running but wrong") from layers you simply haven't started yet.
Publisher counts are NOT a health check — every failure this rig has had kept a count of 1
while the rate collapsed.

### 1a. Camera (D555) — ethernet/PoE DDS at `192.168.11.55`, NOT USB
The D555 is a **PoE/ethernet DDS camera** on `enP8p1s0` (Jetson eth `192.168.11.70`).
It is **NOT USB** — never diagnose with `lsusb` or `/dev/video*`.

⚠ **It pings even when its on-camera DDS server is dead** — so ping is NOT a health
check. The real signal is `./run_stack.sh status` → `infra1 publisher >=1`.

If the realsense log shows `No RealSense devices were found` (check
`./run_stack.sh logs realsense`), the camera's DDS server is offline. **No software
restart recovers this** — a clean `down`+`up` won't fix it either. The user must
**physically power-cycle the D555**: unplug its PoE ethernet cable ~5 s, replug,
wait ~15 s, then:
```bash
./run_stack.sh cam       # relaunch ONLY the camera; cuVSLAM re-locks automatically
```

## 2. Pi5 brain + micro-ROS agent

```bash
ssh rakhi24@192.168.1.16 'systemctl is-active langrobo-brain langrobo-microros'
```
Both `active`. Restart brain WITHOUT sudo: kill its MainPID (Restart=always relaunches in 10 s).
Brain health: `ssh rakhi24@192.168.1.16 'curl -s 127.0.0.1:8090/health'`.

## 3. ESP32 wheel link (the usual failure)

```bash
docker exec orin_nav bash -lc 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0; ros2 topic info /cmd_vel' | grep -i subscription
```
`Subscription count: 1` = wheels connected. Firmware v2 (flashed 2026-08, Pi5 repo
`pi5_ros2_ws` — see `orin-nav-stack/firmware/HARDWARE.md`) auto-reconnects to the agent
(drops only after 3 missed pings), so a WiFi blip recovers on its own. **If it stays 0:**
confirm the ESP32 is powered and on WiFi.

## 4. HP-laptop RViz live view

```bash
./run_stack.sh view      # resolves the laptop, checks it's logged in, prints the next step
```

⚠ The laptop must be **logged in to the desktop** — if it sits at the Ubuntu login screen,
nothing can show and rviz launched over ssh runs **invisibly with no error** (2026-07-19
lesson; hit again 2026-08-10). `view` checks this for you: it reports NOT LOGGED IN when the
only seat0 session belongs to `gdm`.

Easiest: user opens a terminal (Ctrl+Alt+T) **on the laptop** and runs `bash ~/rover_view.sh`.
Remote alternative from the Jetson (only after they log in):
```bash
ssh rakhi24@192.168.1.10 'DISPLAY=:1 XDG_RUNTIME_DIR=/run/user/1000 nohup bash ~/rover_view.sh >/tmp/rviz.log 2>&1 &'
ssh rakhi24@192.168.1.10 'pgrep -x rviz2'    # -x EXACT match — pgrep -f matches your own ssh cmdline!
```
Data check on laptop: `unset ROS_DISCOVERY_SERVER; export ROS_DOMAIN_ID=0; source /opt/ros/jazzy/setup.bash; ros2 topic hz /odom`.
⚠ RViz "2D Goal Pose" button sends a REAL nav goal — rover moves.

## 5. Safety rules before ANY motion (non-negotiable)

- Motion only with the user explicitly OK and watching the power tether.
- Pause YOLO first: `docker exec orin_nav pkill -9 -f "nodes/detections_3d.py"` (full path!).
  Keep it paused until pose verified flat ~10 s AFTER motion ends. Pausing YOLO blinds
  the brain's look()/VLM — that's expected.
- safety_guard gates nav commands only; `drive_test.py` / manual `/cmd_vel` pubs bypass it.
- Never replace `config/bt_navigate_to_pose.xml` with nav2's default BT (blind recovery spins).
- Drivetrain (2026-08): closed-loop BTS7960 + encoder PID (firmware v2 — see
  `orin-nav-stack/firmware/HARDWARE.md`), ~0.86 m/s top speed. Fast straights can still
  stress cuVSLAM tracking — prefer short bursts.
- Wheel odometry **is wired** into the EKF (`config/ekf.yaml` odom1, via the Pi5
  `wheel_odom_relay` → `/wheel_odom`), but is **degraded**: `/wheel_state` arrives at
  **10 Hz** instead of the 20 Hz the firmware asks for (`rover_firmware_v2.ino:391`). The
  cause is not yet found — tracked as `learn/03-imu.md`. The relay also has **no systemd unit**, so it must be restarted
  by hand after any Pi5 reboot: `ros2 run langrobo_ros wheel_odom_relay`.
- **Check `/odom/health` before any move** (needs `fuse`): `trust: false` ⇒ do not drive and
  do not believe any "arrived".
