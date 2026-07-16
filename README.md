# LangRobo Perception — Jetson stack

Voice/VLM-driven rover: say "go near the chair" on Telegram and the robot finds
it, plans a path, and drives there.

```
                  ┌──────────────── WiFi (192.168.1.x) ────────────────┐
D555 camera ──eth──► Jetson Orin Nano ◄──────► Pi5 (brain) ◄──────► Mac mini
(192.168.11.55)     cuVSLAM · nvblox        LangGraph agent        llama.cpp
                    Nav2 · YOLO · grounding  Telegram · micro-ROS   Gemma-12B VLM
                          │ /cmd_vel                │ udp 8888
                          └────────────────► ESP32 rover (192.168.1.11)
                                              wheels + pan/tilt head
```

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
   right, swap the two motor connectors of ONE side, or negate `angularZ` in
   `ESP_32_frimware/rover_firmware.ino` (Pi5 repo) → reflash.
5. Drive a slow manual square (Telegram: "turn left", "go forward") and watch
   RViz — the orange path should mirror what the rover did.
6. First autonomous test: `robot status` all green → Telegram "go one meter
   forward", then "go near the <visible object>". Keep a hand near the power.

## Repo map

| Path | What |
|---|---|
| `scripts/run_all.sh` / `status_all.sh` / `stop_all.sh` | orchestrator (systemd + the Pi5 `robot` CLI call these) |
| `scripts/run_*.sh` | individual stages (camera, cuVSLAM, nvblox, nav2, vision, rviz) |
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
