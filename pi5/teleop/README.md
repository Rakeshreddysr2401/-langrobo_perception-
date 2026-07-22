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

## Files
- `teleop_web.py` — rclpy node + stdlib HTTP server (binds `0.0.0.0:8091`).
- `run_teleop.sh` — sources ROS (domain 0, FastRTPS, multicast — mirrors the brain).
- `langrobo-teleop.service` — systemd unit.

Speed constants (tunable) live at the top of `teleop_web.py`: `VX`, `WZ`,
`VX_SLIGHT`, `WZ_SLIGHT`. Rover caps: `vx <= 0.22 m/s`, `wz <= 0.90 rad/s`.

## Deploy on the Pi5
```bash
# copy this folder to the Pi5 as ~/langrobo_teleop, then:
sudo cp ~/langrobo_teleop/langrobo-teleop.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now langrobo-teleop
systemctl is-active langrobo-teleop      # -> active
```
Quick run without systemd: `nohup ~/langrobo_teleop/run_teleop.sh >/tmp/teleop.log 2>&1 &`
