# Phase 1 — where am I?

Three sensors, three separate answers, one tape measure as referee.

```
./rover camera     D555 streaming, alone
./rover pose       + visual odometry + gyro
./rover wheels     check the ESP32 link (diagnose only)
./rover compare    the side-by-side readout
./rover status     rates for every layer
./rover selftest   frame math, no hardware needed
```

## The three checks that close Phase 1

| | command | pass |
|---|---|---|
| scale | `./rover compare --expect 2.00` | reads 1.90 – 2.10 m |
| drift | `./rover compare --return` | ends ≤ 0.10 m from the start |
| heading | `./rover compare --spin 360` | heading error ≤ 10° |

Push by hand for all three. Motor-driven wheels slip and you cannot hold a
tape-exact 2.00 m under PID.

---

# Measured on this rig — 2026-08-15

Only things actually observed, with the number. Beliefs go in PLAN.md, not here.

## Camera

`./rover camera` passes: **IR left 30.0 Hz, depth 25.1 Hz**. 30 Hz is the
configured cap (`depth_module.infra_profile:=896x504x30`), so the camera is
saturating its profile.

IR image content at 19:53 local, emitter off: mean 48–55, std ~48, range
16–255, 99.9% of pixels above 16. Passive stereo is well exposed indoors in the
evening — darkness is not a limiting factor here.

**Two launch flags are load-bearing and must not be "tidied":**

- `depth_module.emitter_enabled:=0` — the projector is bolted to the camera, so
  its dot pattern is repainted from the camera's own viewpoint every frame. On a
  low-texture floor the dots stay put in the image while the rig translates, and
  the tracker sees almost no motion. Passive depth stays metric either way.
- `enable_sync:=false` — with the cross-stream syncer on, enabling colour gates
  the IR pair behind colour alignment and starves IR.

**The D555 is a network device** (DDS over PoE at `192.168.11.55`), not USB. It
answers ping perfectly while completely dead, so ping is not a health check —
a live publisher on `infra1/image_rect_raw` is. If it goes offline, only a
physical PoE power-cycle recovers it.

## `infra2/camera_info` has the opposite sign to REP-104

REP-104 says a rectified right camera carries `P[3] = -fx · baseline`. This
driver publishes the **opposite sign** for infra2:

```
-P[3]/P[0] = -0.0949      |value| = 9.49 cm = the D555's real stereo baseline
```

So the magnitude is right and only the convention differs. `vo_node.py` takes
`abs()` and keeps the plausibility check on the size, which is what would
actually catch a broken `camera_info`.

## pyCuVSLAM 16.0.0 API

- `track()` returns a `PoseEstimate` whose **`world_from_rig` is a
  `PoseWithCovariance`, not a `Pose`** — the pose is one level down at
  `est.world_from_rig.pose`.
- Passing the **same numpy array object** for both cameras raises
  `ValueError: The same image buffer for images 0, 1`. Left and right must be
  distinct buffers.
- `tf_transformations` is **not installed** in `orin-nav:1.1`. Don't import it;
  the image has no build recipe so nothing should be installed into it.
- The packaged `isaac_ros_visual_slam` is a Thor build and does not run on this
  Orin. The `cuvslam` wheel's own libs must come first on `LD_LIBRARY_PATH`.
- Harmless `nanobind: leaked N functions!` spew on exit — shutdown noise, not a
  fault.

## Pose, running

`./rover pose` passes: **visual odometry 27–29 Hz, gyro 200 Hz**.
Tracker comes up at 896×504, fx 448.3, baseline 9.49 cm, SLAM off.

**Parked, the pose is bit-identical, not merely small.** 2,729 consecutive
frames returned the exact same identity pose (`frozen_frames: 1422` on a later
run) while `landmarks: 162`. Landmarks prove the tracker is genuinely seeing the
world, so this reads as a true zero-velocity result rather than a freeze — but
**it has not yet been confirmed against real motion.** That is the first thing a
push settles.

**Rate is not a health signal.** A dead tracker returns identity at a perfect
30 Hz. `/vo/status` therefore reports `landmarks` and `frozen_frames`, and
`healthy` requires `landmarks > 20`.

## Phase 1 runs odometry only, deliberately

SLAM's loop closure exists to hide accumulated drift by snapping the pose back
when it recognises a place. That is exactly what `--return` is trying to
measure, so SLAM stays off until Phase 1 has its numbers. `slam:=true` enables
it.

## Drivetrain — blocked, needs a reflash

`/wheel_state` arrives at **1.000 Hz, min 0.989 / max 1.012, σ = 3.4 ms over 55
samples**. That regularity rules out packet loss; it is a timer.

- The micro-ROS agent is fine: `langrobo-microros.service` exists, is enabled,
  and started this boot. The old "no systemd unit, start it by hand" belief is
  **obsolete** — units exist for microros, brain, discovery and teleop.
- The ESP32 holds **one stable session** (`_CREATED_BY_BARE_DDS_APP_`), so it is
  `AGENT_CONNECTED`, not reconnecting in a loop.
- But `rover_firmware_v2.ino` publishes `/wheel_state` **only** in
  `AGENT_CONNECTED`, at `EXECUTE_EVERY_N_MS(50, …)` = 20 Hz. No state publishes
  at 1 Hz, and the macro's `static` is correctly scoped per expansion.

**Conclusion: the flashed binary is not built from the repo source.** Neither the
Jetson nor the Pi 5 has `arduino-cli`, `pio`, `esptool` or any USB serial, so a
reflash must come from your own machine. OTA is compiled in.

## Network

| Thing | Address | Note |
|---|---|---|
| ESP32 | **192.168.1.3** | moved off `.12`; DHCP, resolve don't assume |
| Pi 5 | 192.168.1.16 | |
| D555 | 192.168.11.55 | separate subnet |

**The Pi 5 has no RTC battery.** At boot its clock resumes at its last known
value, systemd stamps service start times with that wrong clock, then NTP jumps
it forward. So `systemctl status` will claim a service is "3 days old" seconds
after boot. `uptime -s` computes from monotonic time and is correct. Don't
diagnose from systemd timestamps on that machine.
