# TODO — known problems, in one place

Status: 🔴 blocks a gate · 🟠 real, worked around · 🟡 unverified · ⚪ accepted

---

## 🔴 1. `/wheel_state` publishes at 1.000 Hz, not 20 Hz — cause unknown

**Reflashing did not fix it**, so the original "wrong binary" theory is probably
wrong.

Measured 2026-08-15:

| check | result |
|---|---|
| rate at the Pi 5, beside the agent | **1.000 Hz**, min 0.989 max 1.012, **σ 3.4 ms** |
| so, packet loss? | ruled out — random loss cannot give 3 ms jitter |
| session churn in a 20 s window | **zero** new sessions — it is not reconnect-looping |
| publisher identity | `_CREATED_BY_BARE_DDS_APP_`, i.e. genuinely the ESP32 |
| systemd agent | `langrobo-microros.service` enabled and running |
| after reflash | **unchanged, still 1.000 Hz** |

`rover_firmware_v2.ino` publishes `/wheel_state` **only** in `AGENT_CONNECTED`,
at `EXECUTE_EVERY_N_MS(50, …)` = 20 Hz. No state publishes at 1 Hz, and the
macro's `static` is correctly scoped per expansion. `EXECUTE_EVERY_N_MS(50, …)`
can only fire as often as `loop()` iterates — so **`loop()` is running at ~1 Hz**,
blocked by something. `ArduinoOTA.handle()` and the WiFi stack are the suspects.

**The one test that would settle it, not yet done:** connect the ESP32 to the
Pi 5 by USB and read its debug serial. `DEBUG_SERIAL` prints
`IN vx=… | OUT velL=… dt=0.020` at ~2 Hz from the control task, which runs on its
own FreeRTOS task independent of WiFi.

- lines at ~2 Hz with `dt≈20 ms` → control loop healthy, fault is inside `loop()`
- much slower → the whole board is slow, micro-ROS is innocent
- no lines at all → the running binary is not the one we think

A reader is already staged at `~/esp32_serial.py` **on the Pi 5**. It needs
`sudo chmod a+rw /dev/ttyUSB*` first — the user is not in `dialout`.

**Deferred by choice 2026-08-15: continuing over WiFi for now.**

**Consequence:** at 1 Hz the wheels are a coarse sanity check, not a reference.
`compare.py` integrates them trapezoidally across gaps up to `WHEEL_MAX_DT`, but
the firmware reports *instantaneous* velocity measured over one 20 ms control
period, so at 1 Hz we point-sample a signal that updates 50× faster.

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

## ⚪ 9. The Pi 5 has no RTC battery

At boot its clock resumes at its last known value, so `systemctl status` reports
service start times hours or days wrong until NTP corrects it. Use `uptime -s`.
This nearly caused a misdiagnosis: a service that had started 40 seconds earlier
appeared to be "3 days old".
