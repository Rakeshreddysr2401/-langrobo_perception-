# TODO — known problems, in one place

Status: 🔴 blocks a gate · 🟠 real, worked around · 🟡 unverified · ⚪ accepted

---

## 🔴 1. `/wheel_state` publishes at 1.000 Hz, not 20 Hz

**The cause is the best-effort output stream on the ESP32, not the firmware
logic and not the network.** Diagnosed 2026-08-15 afternoon. Several earlier
theories in this file were wrong and are recorded as dead at the bottom.

### What is ruled out, and by what

| ruled out | evidence |
|---|---|
| `loop()` running slow | round-trip probe: the board consumed **19.71 of 20** `/cmd_vel` per second with **zero lag growth** over 12 s |
| the flashed binary not matching source | that same probe — `cmdVelCb` and the telemetry publish are in the same `loop()`, and the inbound half is healthy |
| a second ESP32 | agent log names its client: `session established … address: 192.168.1.3:47138` |
| WiFi / power-save | ping to `.3` is 2.4–10 ms, −36 dBm, 866 Mbit/s |
| the Pi 5 → Jetson DDS hop | same best-effort listener, same instant: Pi 5 **1.000 Hz ±2.9 ms**, Jetson **1.000 Hz ±38.7 ms** |
| packet loss | ±2.9 ms on a 1000.1 ms gap is a timer; random loss cannot be that regular |
| burst-then-idle buffering | zero inter-arrival gaps under 100 ms — it is genuinely one message per second |
| dead encoders | see §10 — both sides verified by hand |

### What it actually is

`/wheel_state` is created with `rclc_publisher_init_best_effort`. In micro-ROS a
best-effort message is written into an output stream buffer and only goes onto
the wire when the XRCE session next runs — `rcl_publish` does not send it. The
session is driven by `rclc_executor_spin_some`, so with no traffic to service
the stream drains at about 1 Hz.

The decisive observation is that **the telemetry rate tracks inbound traffic**:

| condition | `/wheel_state` rate |
|---|---|
| silent | 0.996 Hz |
| while sending `/cmd_vel` at 20 Hz | 1.87 Hz, and 11.5 Hz on an earlier run |

And the controlled comparison is already in the data: `/cmd_vel` is **reliable**
and moves 19.71 msg/s on the same board, same link, same loop, while
`/wheel_state` is **best-effort** and manages 1. Reliable streams are flushed
inside `rmw_publish` via `uxr_run_session_until_confirm_delivery`; best-effort
ones are not.

### Fix

One line in `rover_firmware_v2.ino`:

```c
rclc_publisher_init_best_effort(&wheelStatePub, …)   // current
rclc_publisher_init_default(&wheelStatePub, …)       // reliable, flushes on publish
```

The best-effort choice was deliberate (see the comment above `createEntities`) on
the reasoning that high-rate telemetry should not be reliable. On this transport
that reasoning is inverted. Needs an OTA flash.

**Still unconfirmed:** whether the board emits 1 UDP packet/s or 20 that the
agent then drops. Needs root on the Pi 5:

```bash
ssh -t 192.168.1.16 "sudo timeout 10 tcpdump -i any -nn 'src 192.168.1.3 and udp' -w /dev/null"
```

~10 packets → the flush theory above is confirmed, flash the fix.
~200 packets → the board is fine and the **agent** is dropping; do not flash.

### Dead theories, kept so they are not re-litigated

- ~~"the flashed binary is not built from this source"~~ — the inbound half of
  the same `loop()` works perfectly.
- ~~"`loop()` is blocked by `ArduinoOTA.handle()` or the WiFi stack"~~ — measured
  fast.
- ~~"`controlTask` (prio 2, core 1) starves `loopTask` (prio 1, core 1)"~~ —
  plausible on inspection, but it blocks on `vTaskDelayUntil(20 ms)` and the
  measurement says `loop()` is fast.
- ~~"rate at the Pi 5 is 1 Hz, measured with `ros2 topic hz`"~~ — that tool
  defaults to **reliable** QoS, which is incompatible with this best-effort
  publisher and silently receives nothing. The number happened to be right;
  the method was not. Always measure this topic with `qos_profile_sensor_data`.

**Consequence while unfixed:** at 1 Hz the wheels are a coarse sanity check, not
a reference. `compare.py` integrates them trapezoidally across gaps up to
`WHEEL_MAX_DT`, but the firmware reports *instantaneous* velocity measured over
one 20 ms control period, so at 1 Hz we point-sample a signal that updates 50×
faster.

---

## 🔴 2. The drift gate probably cannot pass without the wheels

2 m out-and-back, clean run (peak 16 cm/s, no teleports):

| | endpoint error |
|---|---|
| cuvslam alone | 28.4 cm |
| FUSED (gyro heading) | **12.2 cm** |
| gate | ≤ 10 cm |

Fusion fixed the heading half. The residual is **distance**: the return leg
registered 186.5 cm against the outbound 198.0, so reversing under-reads by ~6%.
cuVSLAM is the only translation source, so nothing can contradict it. Encoders
are exactly that missing measurement — see §1.

Note the 10 cm bar was **chosen, not measured** (2 voxels at 5 cm). If a re-run
lands near 12 cm again, that is the rig telling us encoders are required, not a
bug to chase.

---

## 🟠 3. cuVSLAM loses tracking above ~25 cm/s

Measured 2026-08-15:

| run | median | peak | teleports |
|---|---|---|---|
| clean 2 m push | 14.3 cm/s | 18.8 cm/s | 0 |
| failed out-and-back | 10.2 cm/s | **76.5 cm/s** | **1** |

The teleport was **202 cm in a single frame**, at a healthy 28 Hz, with nothing
logged — heading stepped +20.3° in one 0.25 s sample. SLAM is off, so no loop
closure could legitimately do it.

`compare.py` now detects jumps and refuses to grade a run containing one, and
warns live above 25 cm/s. **This is also a speed limit on autonomous driving
later** — nav2's `vx_max` must respect it.

---

## 🟠 4. cuVSLAM is much worse in reverse

Same out-and-back, split by leg:

| leg | heading error vs gyro |
|---|---|
| outbound (0 → 198 cm) | −1.74°, tracking within 0.13° most of the way |
| return (198 → 11 cm) | **+7.68°** |

Inherent: driving forward, features expand outward from the image centre with
long track histories; reversing, they shrink toward the centre and new ones must
enter at the edges where they are worst observed.

Worked around by taking heading from the gyro. **Design consequence for Phase 3:**
prefer plans that turn and drive forward over plans that reverse.

---

## 🟡 5. The 360° spin gate has never been run

Rotation is visual odometry's weakest case and Phase 2 onward depends on it.
`./rover compare --spin 360`.

---

## 🟡 6. Camera mount pitch never measured

`cam_x` 17.0 cm, `cam_z` 16.3 cm and a 2.06° mount **yaw** are all measured. Any
**pitch** (nose up/down) is not. It does not affect Phase 1's numbers much but it
will move the ground plane in Phase 2's mapping.

---

## ⚪ 7. The D555 dies if it loses power, and only a physical power-cycle helps

Went offline at 21:20 on 2026-08-15 when the rover was unplugged for the ESP32
work. `dds-device.cpp:46 throwing: device is offline`, driver process still
alive, 0 publishers. It answers ping the whole time — **ping is not a health
check**. Recovery is unplug 5 s, replug, then `./rover camera`.

---

## ⚪ 8. The container image cannot be rebuilt

`orin-nav:1.1` was made by `docker commit`, not from a Dockerfile, and neither it
nor its 57.8 GB base has a recipe. If it is deleted, everything stops. The only
insurance is `docker save` to external storage. `/` has ~45 GB free of 227 GB, so
this needs somewhere else to go.

---

## ✅ 10. Encoders — both sides verified good (2026-08-15)

Rover lifted, each wheel spun by hand in isolation, watching `/wheel_state`
(`x = velL`, `y = velR`, computed in the 50 Hz control task on its own core):

| spun | velL | velR |
|---|---|---|
| LEFT wheel, right held still | **14 of 15 samples non-zero, peak 0.068** | 0.000 throughout |

Left drives the left channel, right drives the right, correct sign, no
crosstalk. So the encoders, `ENC_*_DIR`, `METRES_PER_COUNT` and the control task
are all sound, and §1 is purely a transport problem.

**Beware a false negative here.** A first attempt reported "LEFT encoder: NO
SIGNAL" simply because samples arrive once a second and the test's phase
boundaries did not line up with which wheel was being spun. Any hand test on
this rig must name one wheel and hold the others still —
`logs/spin_one.py` does that.

---

## ⚪ 9. The Pi 5 has no RTC battery

At boot its clock resumes at its last known value, so `systemctl status` reports
service start times hours or days wrong until NTP corrects it. Use `uptime -s`.
This nearly caused a misdiagnosis: a service that had started 40 seconds earlier
appeared to be "3 days old".
