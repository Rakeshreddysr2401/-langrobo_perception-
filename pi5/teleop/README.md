# Pi5 mobile teleop (`langrobo-teleop`)

Drive the rover from a phone on the same wifi, with an AUTO/MANUAL switch so nav2
stays in charge until *you* take over. Runs on the **Pi5** (192.168.1.16) because
that's where the micro-ROS agent bridges `/cmd_vel` to the ESP32.

## Open it
`http://192.168.1.16:8091` (same wifi).

- **AUTO** (default): teleop is hands-off — publishes nothing, so nav2 / the brain
  own `/cmd_vel` and the rover explores/maps on its own. Drive buttons are locked;
  the always-shared link can't move the robot until you choose to.
- **MANUAL** (tap the top button): you drive. Switching to MANUAL **hard-cancels any
  active nav2 goal** (`navigate_to_pose` + `navigate_through_poses`) so nav2 fully
  lets go — no `/cmd_vel` contention. **Hold-to-move** (dead-man): the rover moves
  only while a button is held; release / lost connection / backgrounding the page
  stops it within ~0.4 s, backed by the ESP32's 500 ms watchdog.

Buttons: forward / back, left / right (pivot in place), slight-left / slight-right
(tight forward turn — the inner wheel counter-rotates), and a big STOP.

> ⚠️ Publishes **directly** to `/cmd_vel` (bypasses `safety_guard`) — drive by sight,
> there is no automatic obstacle stop in MANUAL.

## Map an area by driving

nvblox builds the map from whoever is driving, so you can map a new place by hand:

1. Mount the D555 **rigidly** on the rover (it must not move relative to the body).
2. On the Jetson: `./run_stack.sh remap` — fresh `(0,0,0)` origin + empty map.
3. Phone → **MANUAL**, drive **slowly** to explore. Turn to face walls so the depth
   camera sees them; watch the 2D map fill in on the laptop RViz.
4. Save when done (in the `orin_nav` container):
   `ros2 service call /slam/save_map std_srvs/srv/Trigger` — persists cuVSLAM
   features + the nvblox walls under `~/orin-nav-stack/maps/`.

Keep it **slow, short bursts, turns in short taps** — fast motion (especially the
near-max-power pivots) can make cuVSLAM lose tracking and corrupt the map. If the map
suddenly jumps/smears, `./run_stack.sh remap` and start over.

## Files
- `teleop_web.py` — rclpy node + stdlib HTTP server (binds `0.0.0.0:8091`).
- `run_teleop.sh` — sources ROS (domain 0, FastRTPS, multicast — mirrors the brain).
- `langrobo-teleop.service` — systemd unit.

Speed constants (tunable) live at the top of `teleop_web.py`: `VX`, `WZ`,
`VX_SLIGHT`, `WZ_SLIGHT`. Forward/back is `VX` (m/s). **Turns need near-max PWM**: the
ESP32 sets each wheel from the sign of `vx/0.30 -/+ wz/2.0` (0.02 park deadstick, 0.51
PWM floor), and a pivot scrubs both tires so it needs *more* torque than driving
straight — a low `wz` just buzzes. `WZ=2.0` = ~100% PWM per wheel (teleop bypasses the
nav shim, so the `wz<=0.90` cap doesn't apply here); `slight` uses a high `wz` over a
small `vx` so the inner wheel clearly counter-rotates.

## Deploy on the Pi5

Already installed as the `langrobo-teleop` systemd service (enabled at boot,
2026-07-22). To (re)install from scratch — copy this folder to the Pi5 as
`~/langrobo_teleop`, then:
```bash
sudo cp ~/langrobo_teleop/langrobo-teleop.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now langrobo-teleop
systemctl is-active langrobo-teleop      # -> active
```
Quick run without systemd: `nohup ~/langrobo_teleop/run_teleop.sh >/tmp/teleop.log 2>&1 &`
