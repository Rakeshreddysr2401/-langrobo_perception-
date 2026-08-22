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
| the flashed binary not matching source | the sweep below reproduces exactly what the source predicts once you account for the executor blocking |
| a second ESP32 | agent log names its client: `session established … address: 192.168.1.3:47138` |
| WiFi / power-save | ping to `.3` is 2.4–10 ms, −36 dBm, 866 Mbit/s |
| the Pi 5 → Jetson DDS hop | same best-effort listener, same instant: Pi 5 **1.000 Hz ±2.9 ms**, Jetson **1.000 Hz ±38.7 ms** |
| packet loss | ±2.9 ms on a 1000.1 ms gap is a timer; random loss cannot be that regular |
| burst-then-idle buffering | zero inter-arrival gaps under 100 ms — it is genuinely one message per second |
| dead encoders | see §10 — both sides verified by hand |

### What it actually is

**`loop()` runs once per inbound message, or once per second if none arrives.**
`rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5))` blocks until data is
available rather than returning after its 5 ms timeout. The telemetry publish
sits immediately after that call, so it can only fire as often as the call
returns.

Proven by sweeping the rate we publish TO the board, 12 s per step:

| `/cmd_vel` out | `/wheel_state` in | ratio |
|---|---|---|
| silent | 1.00 Hz | — |
| 2 Hz | 3.13 Hz | 1.56 |
| 5 Hz | 5.08 Hz | 1.02 |
| 10 Hz | 10.08 Hz | 1.01 |
| 20 Hz | 11.76 Hz | 0.59 |
| 40 Hz | 16.44 Hz | 0.41 |

It tracks 1:1 up to 10 Hz then saturates. The 2 Hz row is the giveaway: 2 from
arriving messages plus ~1 from the idle timeout is exactly the 3.13 observed.

### Workaround, in place now, no flash

Publish zero `/cmd_vel` at 40 Hz and the board keeps turning. `logs/keepalive.py`
does this and `./rover compare` starts it automatically. Measured **17.1 Hz**,
gaps 58 ms against the intended 50, no gap over 800 ms. `./rover wheels` passes
at 17.0 Hz.

All-zero commands cannot cause motion — `pidStep()` returns 0 for both sides —
they only keep `lastCmdMs` fresh.

**Never run the keepalive while teleoperating.** It publishes to `/cmd_vel`, so
it would interleave with real commands and make the rover stutter. Hand-pushed
measurement runs only.

### Proper fix, still to do

Stop `loop()` blocking on the executor. Options, cheapest first:

1. `rclc_executor_spin_some(&executor, 0)` — non-blocking poll, so `loop()` free-runs
   at its `delay(1)` rate and the 50 ms timer fires properly. One line, but
   untested: if the timeout is being ignored entirely, zero may block too.
2. Set a spin period on the executor (`rclc_executor_set_timeout`) explicitly.
3. Move telemetry into `controlTask`, which provably runs at 50 Hz on its own
   core. **Risky** — micro-ROS sessions are not thread-safe, so this needs the
   publish handed to `loop()` rather than called from the task.

Do this next time the board is being flashed anyway; the workaround holds until
autonomous driving, where nav2's own `/cmd_vel` stream keeps `loop()` fed.

### Confirmed not the cause

Reliable QoS was flashed on 2026-08-15 (`e580d5e`) on the theory that
best-effort messages sat unflushed in an output stream. `ros2 topic info -v`
confirms the publisher is now `RELIABLE` and the rate was **still exactly
1.000 Hz**. The theory was wrong. The change is harmless and has been kept, but
it is not the fix.

That flash also invalidated the round-trip probe that appeared to show `loop()`
consuming 19.71 cmd/s with zero lag: micro-ROS keeps a shallow input queue that
retains the newest message, so "lag 0" appears no matter how slowly `loop()`
runs. **The sweep above is the measurement to trust**, because it varies the
input rate instead of assuming queue semantics.

### Dead theories, kept so they are not re-litigated

- ~~"the flashed binary is not built from this source"~~ — the source predicts
  the observed behaviour exactly once the executor block is accounted for. Cost
  two pointless reflashes; the lesson is that "the binary must be wrong" is what
  you reach for when the real mechanism is in a library you did not read.
- ~~"`loop()` is blocked by `ArduinoOTA.handle()` or the WiFi stack"~~ — right
  that `loop()` was blocked, wrong about where.
- ~~"`controlTask` (prio 2, core 1) starves `loopTask` (prio 1, core 1)"~~ —
  plausible on inspection, but it blocks on `vTaskDelayUntil(20 ms)`, and
  starvation cannot explain the rate tracking inbound traffic 1:1.
- ~~"best-effort messages sit unflushed in an output stream"~~ — flashed
  reliable, publisher confirms `RELIABLE`, rate unchanged at 1.000 Hz.
- ~~"rate at the Pi 5 is 1 Hz, measured with `ros2 topic hz`"~~ — that tool
  defaulted to **reliable** QoS, incompatible with what was then a best-effort
  publisher, and silently received nothing. The number happened to be right;
  the method was not. Now moot (the publisher is reliable), but measure with
  `qos_profile_sensor_data` regardless.

**Consequence at 1 Hz, if the keepalive is ever not running:** the wheels are a
coarse sanity check, not a reference. `compare.py` integrates them trapezoidally
across gaps up to `WHEEL_MAX_DT`, but the firmware reports *instantaneous*
velocity measured over one 20 ms control period, so at 1 Hz we point-sample a
signal that updates 50× faster. At 17 Hz that objection largely goes away.

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

## 🟡 3. Teleports are driven by TEXTURE, not speed — the 25 cm/s limit was wrong

Every run we have, cross-tabulated:

| run | peak speed | landmarks | teleports |
|---|---|---|---|
| clean 2 m push | 18.8 cm/s | healthy | 0 |
| failed out-and-back | 76.5 cm/s | — | 1 |
| hard drive, bare wall | 94 cm/s | **min 4** | **32** |
| hard drive #2 | 84 cm/s | **min 17** | **12** |
| **teleop loop, 2026-08-21** | **42 cm/s** | **162** | **0** |

**42 cm/s with good texture produced zero teleports.** The runs that failed were
not the fast ones, they were the blind ones. And the 84–94 cm/s figures are
themselves *inflated by* the teleports — peak speed is computed from cuVSLAM's
own position deltas — so the real driving speed in those runs was lower than it
looks, which weakens the speed correlation further.

The original entry claimed a ~25 cm/s limit from correlating teleports with peak
speed across two runs. That correlation was real and the causation was not:
landmarks were the confound, and they move together because a bare wall is both
featureless and where you tend to drive faster.

**Consequence for nav2:** do not cap `vx_max` at 25 cm/s on this evidence. The
honest rule is to **watch landmarks, not the speedometer** — `compare.py` and
`fusion_node` already drop cuVSLAM below 30 of them, and `/fusion/status`
publishes the count. A texture-aware speed limit would be better than a fixed
one, but this needs a deliberate test: drive the same textured route at
increasing speed until it breaks. That has not been done.

**Not yet ruled out:** that speed matters *at the margin*, when texture is
already thin. Nothing here separates those.

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

## 🟠 7. The D555 drops out constantly, in three distinct ways

It went down **four times on 2026-08-15 alone**. For an autonomous rover this is
the least solved thing on the vehicle — a robot that needs a human to reseat a
cable is not autonomous. The three failures look similar and are not:

| symptom | what is true | fix |
|---|---|---|
| `No RealSense devices were found` **and** `ethtool` says `Link detected: no` | no electrical link at all — cable or PoE injector | reseat the cable; nothing on the camera can help |
| `dds-device.cpp:46 device is offline`, driver alive, 0 publishers | link fine, on-camera DDS server dead | unplug 5 s, replug |
| streams at 30 Hz but `ros2 param set` times out | **half-alive**: image traffic works, option traffic does not | wait ~30 s after power-up, then retry |

That third one is new and dangerous. Seen 2026-08-15: infra1 streaming at
30 Hz with the emitter stuck ON, and every attempt returning
`timeout waiting for reply: {"id":"set-option","option-name":"Emitter Enabled"}`.
Everything looks healthy and the pose is quietly worthless — this is the failure
that made a 60 cm push read 1.1 cm. **The camera answers image traffic before it
answers option traffic**, so attaching the driver too soon after a power-cycle
lands here. `./rover camera` now retries the emitter set for ~30 s and refuses to
continue if it never takes.

**Ping is not a health check** in any of the three. Nor is a live image stream.
The only proof is the emitter read-back.

---

## ⚪ 8. The container image cannot be rebuilt

`orin-nav:1.1` was made by `docker commit`, not from a Dockerfile, and neither it
nor its 57.8 GB base has a recipe. If it is deleted, everything stops. The only
insurance is `docker save` to external storage. `/` has ~45 GB free of 227 GB, so
this needs somewhere else to go.

---

## 🟠 14. A pivot needed ~2× the duty the teleop was sending — RESOLVED, retest pending

**On the floor**, a pivot command drove instead of turning: counter-rotating in
**0%** of samples, the rover reversing along a slight curve. It cost a mapping
session — 10.7 m of "room loop" driven inside a 1.8 m box, because every attempt
to turn just drove the rover back and forth.

**Lifted on blocks, the wheels counter-rotate perfectly and symmetrically:**

| commanded | velL | velR | |
|---|---|---|---|
| `wz −2.00` | **+0.411** | **−0.407** | counter-rotating |
| `wz +2.00` | **−0.398** | **+0.398** | counter-rotating |

So the wiring, `R_MOTOR_DIR` and the reverse PWM path are all **correct**. The
electrical side does exactly what it is told. It is **traction**: four tyres
scrubbing sideways need more torque than the rover was being given.

### The cause: a units mismatch in the teleop

`rover_firmware_v2.ino` reads `wz` as **rad/s** and computes each wheel as
`vx ± wz × 0.34/2`. The teleop was written for an earlier firmware
(`rover_sim contract_bridge.py`) where `wz` was a **PWM fraction** — and its own
comment still claims `WZ = 2.0` gives "~100% PWM per wheel".

| WZ | wheel target | duty |
|---|---|---|
| **2.0** (was) | 0.34 m/s | **48%** |
| 4.0 | 0.68 m/s | 87% |
| **5.0** (now) | 0.85 m/s | **100%** |

Full authority is `2 × 0.86 / 0.34 = 5.06 rad/s`, where both wheels reach maximum
speed in opposite directions. `WZ` is now 5.0 and `WZ_SLIGHT` 4.25, scaled by the
same factor.

**Retest on the floor before believing it.** Unloaded counter-rotation proves the
duty is now available; it does not prove it is enough to break the scrub. If it
still will not pivot, the remaining candidates are current sag (two motors share
one BTS7960 per side, and a pivot is the highest-current manoeuvre) or simply
too much grip for these motors — in which case the answer is a wider turning
radius rather than more duty.

---

## ✅ 13. Skid-steer scrub — measured, and it changes what the wheels are for

This rover has four driven wheels and no steering, so a turn drags every tyre
sideways. Two consequences, both measured 2026-08-21.

**The turning geometry is not the physical track.** `wz = (vR - vL) / W` needs an
effective width that includes the scrub:

| turn | wheels read | truth | ratio | implied width | peak rate |
|---|---|---|---|---|---|
| 90 left | 146.49 | 90 | 1.628 | 0.5534 m | — |
| 360 left | 556.49 | 360 | 1.546 | 0.5256 m | 21.4 °/s |
| 360 left | 548.34 | 360 | 1.523 | 0.5179 m | 76.2 °/s |
| 360 left | 553.08 | 360 | 1.536 | 0.5224 m | 74.3 °/s |
| 360 left | 551.49 | 360 | 1.532 | 0.5209 m | 75.1 °/s |

`WHEEL_BASE_ROT_M = 0.5216`, the mean of the four 360s. Using the physical
0.34 m made the wheels **63% wrong on every turn**. Note the faster turns
over-read slightly less — scrub is not a constant, so re-measure on carpet.

**The wheels only agree with each other in a straight line.** Per-wheel counts,
same side, same BTS7960, mechanically obliged to sweep the same arc:

| | LEFT front/rear | RIGHT front/rear |
|---|---|---|
| straight (200 cm) | 1.00× | 1.03× |
| turning (360°) | **1.60×** | **1.29×** |

Repeated on the next run: LEFT 37.5%, RIGHT 23.0% against 37.6% and 22.5%. The
scrub is **reproducible to half a percent**, so it is a deterministic property of
this chassis and loading, not random slip. That is what makes it safe to freeze
the calibration through turns rather than trying to filter it.

So encoder distance is an excellent reference straight and a poor one mid-turn.
`compare.py` now **freezes the encoder/cuVSLAM scale calibration while turning**,
because folding those samples in would drag a good calibration off with scrub
that is not travel at all.

**Design consequence:** heading comes from the gyro (−0.2% to −0.9% across four
turns), distance from the encoders while straight, and neither trusts the wheels
to measure a rotation.

---

## ✅ 12. Yaw sign verified against REP-103 (2026-08-21)

A LEFT teleop command produces a **positive** yaw rate, as REP-103 requires.
Checked with `logs/check_yaw_sign.py` because it cannot be read off the source —
it depends on how the IMU is bolted in and how `gyro_node` re-frames it. An
inverted sign would have made nav2 steer away from every goal, and would not
have shown up until the rover was driving itself.

This also resolved an ambiguity: a 360° run reported as clockwise read
`+361.53°`. The convention being correct means it was a mislabelled left turn,
not a sign bug.

---

## ✅ 11. cuVSLAM under-reads distance by ~2.2%, consistently

Four independent measurements against a tape:

| pushed | cuVSLAM read | error |
|---|---|---|
| 200 cm | 197.3 cm | −1.35% |
| 200 cm | 195.1 cm | −2.45% |
| 120 cm | 116.9 cm | −2.6% |
| 200 cm | 195.4 cm | −2.3% |

That is a **systematic scale error, not noise** — it is the same sign and
roughly the same size every time. It passes the ±5% scale gate, so Phase 1 does
not care, but over a long route it compounds: 100 m of driving is 2.2 m short.

A constant scale factor points at the stereo baseline, which is what converts
disparity into metres. `−P[3]/P[0]` gives 9.49 cm; a true baseline of
9.49 × 1.022 = **9.70 cm** would remove the error exactly. Worth checking against
the physical lens spacing before anyone hard-codes a fudge factor.

The encoders are now the better distance reference (§10), so FUSED takes
magnitude from them and direction from cuVSLAM, which removes this from the
fused pose without touching the driver.

---

## ✅ 10. Encoders — all four verified good AND calibrated (2026-08-15)

`ENCODER_CPR = 1560` is **correct**. Measured against a 200 cm tape push:

| wheel | counts | implied CPR | vs configured |
|---|---|---|---|
| LF | 11434 | 1526.6 | 0.98× |
| LR | 11378 | 1519.2 | 0.97× |
| RF | 11623 | 1551.9 | 0.99× |
| RR | 11275 | 1505.4 | 0.97× |

All four within 3% of each other and of the configured value. No firmware
change needed. `logs/calibrate_encoders.py <cm>` repeats it.

**Two false alarms got here first, both mine, both worth remembering:**

1. *"LEFT encoders are dead"* — the four-wheel test printed its prompts through
   a buffered pipe, so the operator never saw them and nothing was spun at the
   right moment. A test that needs the human and the script to agree on WHEN is
   fragile; `logs/wheels_selfpaced.py` asks only for a total instead.
2. *"Encoders are 2× out and the rear reads 25% more than the front"* — that
   compared cumulative counts, which measure **arc length**, against cuVSLAM's
   `straight`, which is **displacement**. On a push that curves or doubles back
   those are different quantities. Calibrate on a straight forward push against
   a tape, and against nothing else.

---

## ✅ 12. Encoders — both sides verified good (2026-08-15)

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
