# Orin nav stack — troubleshooting

Quick health of everything: `./run_stack.sh status`

| Line | Good value | If bad, see |
|------|-----------|-------------|
| `camera D555 (infra1 publisher)` | `>=1` | [Camera](#camera-d555--no-images) |
| `cuVSLAM "slam_pose_ok"` | `true` | [SLAM](#cuvslam-not-tracking) |
| `safety data` | `ok` | vision layer not up → `./run_stack.sh vision` |
| `ESP32 wheels (/cmd_vel subs)` | `1` | [Wheels](#esp32-wheels-not-linked) |
| `nav2` | `active` | [nav2](#nav2-fails-to-activate) |

> `nav2` is a **live** `bt_navigator` lifecycle check (`ros2 lifecycle get`), not a
> `/tmp/nav2.log` grep — so it reports `<not active / not started>` the moment nav2
> crashes, instead of falsely staying `active` from a stale log line.

---

## Camera (D555) — no images

**The D555 is an ethernet/PoE DDS camera at `192.168.11.55` (Jetson eth `enP8p1s0`,
`192.168.11.70`). It is NOT USB.** Do not use `lsusb` or `/dev/video*` to diagnose it.

⚠ **It responds to `ping` even when its on-camera DDS server is dead.** Ping proves the
network link only, never that the camera will stream. The real health signal is a live
publisher on `/camera/camera0/infra1/image_rect_raw` (what `status` and `up` check).

### Symptom: `No RealSense devices were found` (in `./run_stack.sh logs realsense`)
The camera's on-camera DDS server is offline. This commonly follows a
`device is offline` (`dds-device.cpp`) event where the camera dropped mid-stream.

**No software restart fixes this** — a full `down`+`up` with a fresh container (zero
stale DDS participants) will still report no devices. The **only** fix is a physical
power-cycle:

1. Unplug the D555's PoE ethernet cable (that's its power) for ~5 s — or cycle the PoE
   switch port.
2. Replug, wait ~15 s for it to boot and start advertising over DDS.
3. `./run_stack.sh cam` — relaunches only the camera node. cuVSLAM keeps running and
   re-locks automatically once images return.
4. `./run_stack.sh status` — confirm `infra1 publisher >=1`.

`up` gates on this now: it aborts with the power-cycle message instead of starting SLAM
against a dead camera, and leaves the container up so `cam` can retry.

---

## cuVSLAM not tracking

`slam_pose_ok: false` / `frames` not increasing almost always means **no camera images**
(fix the camera first, above). With images flowing, cuVSLAM needs texture and gentle
motion to lock. Restart pose/map without touching the camera: `./run_stack.sh remap`.

Fast straights can explode tracking — prefer short bursts, and pause YOLO before motion
(`pkill -9 -f nodes/detections_3d.py`). (The old stall-or-~0.5 m/s open-loop motors were
replaced 2026-08 by the closed-loop BTS7960 + encoder drivetrain — `firmware/HARDWARE.md`.)

---

## nav2 fails to activate

`Failed to activate local_costmap ... transform from base_link to odom did not become
available` means nav2 started **before** cuVSLAM was publishing the `odom` TF. Order
matters: bring `up` fully (camera streaming + SLAM tracking) *then* `nav2`. If it aborted,
just re-run `./run_stack.sh nav2` once `status` shows `slam_pose_ok: true`.

Never swap `config/bt_navigate_to_pose.xml` for nav2's default BT — the default has blind
recovery spins (Spin/BackUp) that are unsafe on the tethered rover.

---

## ESP32 wheels not linked

`/cmd_vel` subscription count `0` = wheels not connected. Firmware v2 (flashed 2026-08 —
see `firmware/HARDWARE.md`) auto-reconnects to the agent (drops only after 3 missed pings),
so a brief WiFi blip recovers on its own. If it stays `0`: confirm the ESP32 is powered and
on WiFi, then re-check `./run_stack.sh status`.
