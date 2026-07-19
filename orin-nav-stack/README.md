# Orin Nav Stack

**Autonomous rover perception + navigation, fully on-device.** GPU visual SLAM (cuVSLAM),
live 3D mapping (nvblox), path planning/driving (nav2), and object detection (YOLO) — all in
**one self-contained Docker image (`orin-nav:1.1`)** on a Jetson Orin Nano, with a LangGraph
"brain" on a Pi5 and a VLM on a Mac mini talking to it over ROS 2.

This folder is the **single source of truth**: every node, config, model and script the robot
needs lives here and is baked into the image. No external workspaces.

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
| Motors | ESP32 (wifi, `192.168.1.11`) via micro-ROS agent on the Pi5 (`:8888`); 500 ms cmd watchdog |
| Brain | Pi5 `192.168.1.16` (`rakhi24-desktop`, passwordless ssh) — LangGraph + micro-ROS agent |
| VLM | Mac mini on the LAN — vision-language decisions, sends pixel queries |
| Jetson net | wifi `192.168.1.15` (LAN/ROS), eth `192.168.11.70` + `192.168.2.20` (camera subnet) |

## 3. Architecture / data flow

```
                 ┌────────────────────── one container: orin-nav:1.1 ──────────────────────┐
 D555 (ethernet) │                                                                          │
  infra1+infra2 ─┼─► cuvslam_ros_node ──► TF odom→base_link + /odom + /visual_slam/…odometry│
  depth ─────────┼─► nvblox_node ──────► 3D mesh + ESDF + occupancy grid (frame: odom)      │
  color ─────────┼─► detections_3d (YOLO) ─► /vision/target_result, /vision/detections_3d   │
  imu (motion) ──┼─► imu_to_base ─► /imu/base                                               │
                 │        nav2 (planner+MPPI, SAFE BT) ─► cmd_vel_nav                       │
                 │        cmd_vel_deadband (PWM floor + SPEED CAPS) ─► /cmd_vel             │
                 └──────────────────────────────────────────────────────────────────────────┘
   map→odom (static) + base_link→camera0_link (static x0.10 z0.20) complete the TF tree
   /cmd_vel ─► Pi5 micro-ROS agent ─► ESP32 ─► wheels      Pi5/Mac ─► /vision/*, nav goals
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
| `up` | Start container + D555 (stereo/depth/color/imu, emitter ON) + TFs + cuVSLAM + nvblox |
| `nav2` | map→odom bridge + nav2 with the **no-blind-recovery BT** |
| `vision` | YOLO hunt, pixel→goal, motor deadband (capped vx≤0.22 wz≤0.90), imu relay |
| `stop` | **E-STOP** — kills every motion node + publishes zero velocity |
| `down` | **Hard stop** — removes the container (watchdog halts rover ≤0.5 s) |
| `remap` | Fresh map + pose→(0,0,0): restarts cuVSLAM+nvblox **without touching the camera** |
| `rviz` | RViz (nav2 view) on the Jetson monitor |
| `logs <name>` | Tail `/tmp/<name>.log` in the container: `realsense cuvslam nvblox nav2 detections_3d pixel_to_goal cmd_vel_deadband imu_to_base` |

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

**Ask YOLO what it sees**
```bash
docker exec orin_nav bash -lc 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0
  ros2 topic pub --once /vision/target std_msgs/msg/String "{data: bottle}"
  ros2 topic echo --once /vision/target_result'
```

**Fresh map / reset pose after drift or a pose jump** → `./run_stack.sh remap`

## 7. Viewing (RViz / laptop)

- On the Jetson monitor: `./run_stack.sh rviz` (close it when navigating — RAM).
- From an **Ubuntu laptop** with ROS 2 Jazzy on the same LAN: `ROS_DOMAIN_ID=0 rviz2` —
  add TF, `/nvblox_node/static_occupancy_grid` (live map), `/plan`, camera images.
  Note: `/global_costmap/costmap` QoS is **TRANSIENT_LOCAL** — set the display's durability accordingly.
- Mac: Foxglove Studio → `ws://192.168.1.15:8765` after starting
  `ros2 run foxglove_bridge foxglove_bridge` in the container (works, but laggy over wifi).

## 8. SAFETY — read before moving the rover

1. **Tethered power cable**: blind rotations tangle it. The BT (`config/bt_navigate_to_pose.xml`)
   has **NO Spin/BackUp** — never replace it with nav2's default (caused a live incident 2026-07-19).
2. Output speeds capped in `nodes/cmd_vel_deadband.py`: vx ≤ 0.22 m/s, wz ≤ 0.90 rad/s.
   Hardware floors (0.20 / 0.80) are needed to move at all → "slow" = short supervised bursts.
3. **E-stop**: `./run_stack.sh stop`; if in doubt `./run_stack.sh down` (always works).
4. **Pause YOLO during motion tests** — CPU overload causes cuVSLAM pose jumps → nav chaos.
5. Rover moves only with an operator watching the tether.

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
| Wheels hum but no motion | Below torque floor → speeds ≥ 0.25 m/s for tests; deadband handles nav2 commands |

## 10. Facts & calibration (measured)

- **Tape-verified accuracy**: commanded 1.00 m → real 0.95–1.00 m (closed-loop on cuVSLAM).
- cuVSLAM (and RTABMap before it) under-read real distance ×~1.2 → **D555 factory stereo calib
  scale**; `CAL=1.2` inside `drive_test.py` compensates. Baseline 0.0949 m, 896×504@30 IR.
- IR **emitter ON** (dense depth; does not hurt cuVSLAM in this single-container setup).
- D555 IMU: single combined `motion` stream 200 Hz; DDS driver reports identity extrinsics
  (why VIO is parked — needs Kalibr).

## 11. Files

```
Dockerfile                 # orin-nav:1.1 (FROM isaac_ros:cuvslam-unified + everything baked)
run_stack.sh               # all operations (see §5)
cuvslam_ros_node.py        # the Orin cuVSLAM ROS wrapper
config/  nav2.yaml | nvblox.yaml | bt_navigate_to_pose.xml (safe BT)
nodes/   detections_3d | pixel_to_goal | cmd_vel_deadband | imu_to_base | visual_approach | drive_test
models/  yolov8n.pt
standalone/                # ROS-free cuVSLAM dev tools (see standalone/README.md)
```
Rebuild after any edit: `docker build -t orin-nav:1.1 ~/orin-nav-stack`
(then `./run_stack.sh down && ./run_stack.sh up …`)

**Do not delete** images `isaac_ros:langrobo-prod` / `isaac_ros:cuvslam-unified` — they are the
layer parents of `orin-nav:1.1` (until a lean rebuild replaces them).

## 12. Roadmap / known limitations

1. **Free space under the robot**: camera can't see its own feet → coordinate goals rely on
   `allow_unknown: true`. Better: seed robot cell free / startup mapping arc.
2. **Loop closure + map save/reload** (cuVSLAM SlamConfig + LocalizeInMap): drift correction,
   persistent maps, true relocalization — the next big feature.
3. **VLM→goal refinement loop**: coarse far goal from a detection, refined as depth improves.
4. **Lean image**: rebuild from a minimal isaac-ros base (drops RTABMap etc., frees ~50 GB).
5. **ESP32 firmware flash** (committed, awaiting USB): fixes reconnect bug + PWM floor →
   deadband shim can then be removed.
6. VIO (IMU fusion) parked until Kalibr extrinsic calibration.

Old langrobo_perception source: `~/archives/langrobo_perception_20260719.tar.gz`.
