# Phase 1 — Perception

**Goal:** know where the rover is. Every later phase stands on this, because a
map built on a wrong pose is a wrong map, and nav2 plans against `odom → base_link`.

Everything below was measured on this rig on **2026-08-15**. Nothing is inherited
from documentation, datasheets, or a previous build.

---

## 1. The sensors, and what each one is honestly good for

| sensor | interface | rate | gives | used for |
|---|---|---|---|---|
| D555 stereo IR (`infra1`/`infra2`) | Ethernet/DDS, 896×504 | 30 Hz | image pair | **x, y, θ** via cuVSLAM |
| D555 IMU (`motion/sample`) | same | 200 Hz | 3 rates + 3 accelerations | **yaw**, **roll/pitch** |
| 4× wheel encoders | ESP32 → micro-ROS → WiFi | 20 Hz | cumulative counts | **distance** |
| D555 depth | same | 27 Hz | depth image | *nothing in Phase 1* — see below |

### Why depth is deliberately unused

Depth is **computed from** the same stereo IR pair cuVSLAM already consumes. It
carries no independent information about where the rover is — using it would be
asking one witness the same question twice and counting two answers. It is
Phase 2's input, for building the map.

### Why the accelerometer is not used for position

Integrating acceleration twice makes error grow with **t²**. A 0.01 m/s² bias —
entirely typical — becomes 0.5 m of error after 10 seconds and **180 m after a
minute**. No filter fixes this; it is arithmetic.

Its real signal is **gravity**, which never drifts. That gives roll and pitch as
absolute angles with zero accumulated error — something no other sensor here
provides. It was being thrown away.

---

## 2. Techniques

### Stereo visual odometry — cuVSLAM 16.0.0

Tracks features between consecutive stereo frames and solves for the camera
motion that explains their movement. Scale comes from the **baseline** (the known
distance between the two lenses), which is why a wrong baseline is a wrong
distance, and why the error is proportional rather than random.

SLAM/loop-closure is **off** in Phase 1, deliberately: we are measuring raw drift.
Loop closure would mask exactly the error we want to see.

### Frame conjugation

cuVSLAM reports the pose of the *camera* in *optical* axes (x right, y down,
z forward). ROS wants the pose of `base_link` in REP-103 axes (x forward, y left,
z up). Both the axes and the origin must move:

```
odom_from_base(t) = B · world_from_rig(t) · B⁻¹        B = base_link ← left_optical
```

Conjugation, not a single multiply — the transform is applied on both sides, or
the rover appears to swing around a point 17 cm in front of itself.

### Gyro bias — continuously re-estimated

A MEMS gyro reads a non-zero rate while perfectly still. Integrate that and the
heading walks away linearly.

**The offset is not constant.** Measured across three runs the same day:
`+0.0838`, `+0.1050`, `+0.1808 °/s` — over 2× apart, moving as the board warms.
A startup average therefore goes stale and starts *adding* error: one run
over-corrected and walked **−9.52° in 107 s with the rover standing still**,
against a 10° gate.

Fix: whenever the rover is definitely still, whatever the gyro reads **is** the
bias, so it is eased toward that continuously (~25 s time constant).

"Definitely still" needs **both** the wheels and cuVSLAM to agree. Either alone
can be fooled — wheels by a slippery floor, cuVSLAM by a blank wall — and a false
"still" mid-turn would absorb real rotation into the bias and corrupt heading
permanently.

### Complementary filters

Two sensors, opposite weaknesses, combined by frequency.

**Roll/pitch** — the gyro is smooth and instant but drifts; the accelerometer is
absolute but noisy and reports fake tilt under acceleration. Integrate the gyro
short-term, let gravity pull it back long-term. τ = 0.25 s.

**Yaw** — gyro dominant (it beat cuVSLAM by 7.68° on a return leg), pulled slowly
onto cuVSLAM's heading, which does not drift because it is re-measured against
the world every frame. τ ≈ 30 s: long enough that a teleport cannot yank the
heading, short enough to bound gyro drift.

> The filter must be **recursive on its own previous value**. Written as
> `fused = gyro + (vo − gyro)·k` it recomputes from the gyro every message and
> the correction never accumulates — it was 99.9% gyro no matter how long it ran,
> and reported −9.51° while cuVSLAM sat at +0.01°.

### Encoder-corrected distance

cuVSLAM supplies the **direction**, the encoders supply the **magnitude**.

cuVSLAM under-reads distance by a systematic **~2.2%**, and under-reads
*reversing* by ~6%. Being the only translation source, nothing could contradict
it. Encoders do not care about visual texture or direction of travel — they are
exactly the missing measurement. Rescale is bounded to 0.5–2.0× so a bad sample
cannot run away.

### Chord-based path accumulation

Summing per-frame `|Δposition|` adds a magnitude that can never cancel, so `path`
ratcheted up **9.0 cm while the rover sat still** (position noise is ~26 µm/frame).
Travel is now accumulated only once the pose has moved 5 mm from the last anchor.

### Teleport rejection

A step is a teleport if it exceeds **15 cm** *or* implies over **1 m/s** — no one
hand-pushes a rover at 4.4 m/s, so a 14.7 cm hop in one 33 ms frame is the tracker
re-initialising, not motion. Every number after a teleport is measured from a
corrupted origin, so a run containing one is **thrown away, not graded**.

---

## 3. Measured facts about this rig

| quantity | value | how |
|---|---|---|
| camera x offset | **0.170 m** | tape (4.5 cm + 25.0/2) |
| camera z offset | **0.163 m** | tape |
| camera mount yaw | **2.06°** | tape |
| camera mount pitch | **−1.63°** | **gravity vector** — never previously measured |
| stereo baseline | 9.49 cm reported | `−P[3]/P[0]`; ~9.70 cm would fix the scale error |
| wheel diameter | 0.085 m | tape |
| wheel base | 0.34 m | tape |
| encoder CPR | **1560, verified** | 200 cm tape push |
| cuVSLAM scale error | **−2.2%, systematic** | four tape measurements |
| cuVSLAM speed limit | **~25 cm/s** | teleports correlate with peak speed |
| gyro bias | 0.084–0.181 °/s, **varies** | three runs |
| healthy landmarks | 100–200; **<30 is fragile** | recorded at every teleport |

### Encoder calibration, 200 cm by tape

| wheel | counts | implied CPR | vs configured 1560 |
|---|---|---|---|
| LF | 11434 | 1526.6 | 0.98× |
| LR | 11378 | 1519.2 | 0.97× |
| RF | 11623 | 1551.9 | 0.99× |
| RR | 11275 | 1505.4 | 0.97× |

All four agree within 3%. **No firmware change needed.**

---

## 4. What was achieved

| check | target | result |
|---|---|---|
| camera rate | ≥15 Hz | ✅ 30.0 Hz, emitter verified OFF |
| cuVSLAM rate | ≥10 Hz | ✅ 30.0 Hz, 116 landmarks |
| gyro rate | ≥50 Hz | ✅ 200.9 Hz |
| wheel rate | ≥15 Hz | ✅ 20.0 Hz |
| all 4 encoders | respond | ✅ verified individually |
| **scale** | 2.00 m ±5% | ✅ 195.4 cm (−2.3%) |
| **drift** | ≤10 cm out-and-back | ✅ **2.5 cm** hand-pushed (was 12.2 cm) |
| **drift, driven hard** | ≤10 cm | ✅ **4.9 cm** through 12 teleports |
| **stationary stability** | no phantom motion | ✅ **0.08° over 161 s** (was −9.52°) |
| **heading / 360° spin** | ≤10° | ⏳ **not yet run** |
| **teleop** | `/cmd_vel` moves and stops wheels | ✅ **balance 1.00, 0.197 of 0.200 m/s** |

### Teleop — proven 2026-08-21

Driven from a phone at `http://192.168.1.16:8091` (hold-to-move, 10 Hz, 0.4 s
dead-man release backed by the ESP32's own 500 ms watchdog).

| check | result |
|---|---|
| command reaches the wheels | ✅ phone → Pi 5 → `/cmd_vel` → WiFi → ESP32 → PID → motors |
| both sides drive | ✅ balance **1.00** — it goes straight, so the DIR constants are right |
| PID tracking | ✅ 0.197 m/s measured against 0.200 commanded (1.5%) |
| release stops it | ✅ decays 0.197 → 0.043 → 0.004 — a coast, as the firmware intends |

`velL`/`velR` here are **measured from the encoders**, not echoed from the
command, so they are real evidence the wheels turned.

**The page defaults to AUTO and the buttons do nothing until it is flipped to
MANUAL** — that is nav2/the brain owning `/cmd_vel`, not a fault. In MANUAL it
publishes at 10 Hz whether or not a button is held, so `/cmd_vel` traffic alone
proves nothing; only a non-zero velocity does.

### The run that proves fusion works

Driven under teleop with repeated lefts, rights, forwards and reverses, in a
room where the camera had little to look at. cuVSLAM lost tracking **12 times**
and landmarks fell to **17** against a healthy 100–200.

| source | endpoint error | heading |
|---|---|---|
| cuvslam | 79.6 cm | +6.98° |
| wheels | 55.7 cm | −31.51° |
| gyro | — | +1.71° |
| **FUSED** | **4.9 cm** | **+1.75°** |

**FUSED beat both of its own inputs** — 16× better than cuVSLAM, 11× better than
the wheels — and its heading landed on the gyro's, which was the only heading
worth having.

Two mechanisms did it, both visible in the run:

- `cuvslam DROPPED (few landmarks) 501x` — below 30 landmarks cuVSLAM is ignored
  outright, not merely filtered. Refusing individual teleports is not enough:
  between them the messages keep arriving at a confident 30 Hz with a corrupted
  direction in them.
- `FUSED carried 101 frames on wheels+gyro (65.1 cm)` — travel cuVSLAM never saw,
  reconstructed from raw encoder ticks and gyro heading.

The previous attempt at the same test gave **42.4 cm and FAILED**, worse than the
wheels alone at 17.4 cm, because dead reckoning silently measured travel in 5 mm
chords and reported zero, while FUSED went on trusting a blind tracker's heading.

### The two headline improvements

**Drift: 12.2 cm → 2.5 cm.** Gyro heading plus encoder-corrected distance.

**Stationary heading: −9.52° → +0.08°.** Continuous bias tracking plus a yaw
filter that actually filters.

### Faults found and fixed along the way

| fault | why it mattered |
|---|---|
| IR emitter silently ON (`:=0` is a no-op for a Boolean) | projector dots move *with* the rig; a 60 cm push read 1.1 cm |
| `docker exec` without a TTY | Ctrl-C never proxied — every run's verdict and CSV was lost |
| `/wheel_state` at 1.000 Hz | `loop()` blocked in `rclc_executor_spin_some`; fixed with timeout 0 → 20 Hz |
| yaw filter not recursive | fusion was doing nothing at all |
| gyro bias assumed constant | −9.52° of phantom rotation while parked |
| `path` ratcheting | 9.0 cm of travel accumulated while stationary |
| camera "half-alive" | streams at 30 Hz but refuses option changes — emitter stuck on, everything else green |

---

## 5. Commands

### Bring-up — one layer at a time, each with its own PASS/FAIL

```bash
./rover camera      # D555 alone; verifies emitter OFF by read-back
./rover pose        # cuVSLAM + gyro, re-framed into base_link
./rover wheels      # ESP32 telemetry rate
./rover status      # every layer's rate, one screen
./rover values      # one-shot readout of every sensor
./rover stop        # tear down
```

Order matters. Each layer checks the one beneath it, so a failure names its own
layer instead of hiding in a wall of log.

### The Phase 1 gates

```bash
./rover compare --expect 2.00    # push 2.00 m straight  -> want 190-210 cm
./rover compare --return         # out 2.00 m and back   -> want straight <= 10 cm
./rover compare --spin 360       # rotate 360 deg by hand -> want error <= 10 deg
./rover compare                  # no gate: just watch every sensor live
```

Keep still for the first **5 seconds** (gyro calibration), push, then **Ctrl-C**
to grade and write the CSV.

**Push under 25 cm/s**, and keep clutter in view — a bare wall starves the tracker.

### Logs

Every run writes a timestamped CSV to `logs/`, on the host, surviving container
teardown:

```bash
ls -lt logs/compare-*.csv | head        # most recent runs
```

Columns: per-source `x, y, th_deg, straight, path, hz` for **cuvslam / wheels /
gyro / FUSED**, plus the raw sensor values — `roll_deg`, `pitch_deg`,
`landmarks`, `accel_x/y/z`, `gyro_x/y/z`, `velL`, `velR`, and all four cumulative
tick counts. A surprising result is worth re-reading, and you cannot re-read what
was never written down.

### Diagnostics

```bash
# all six streams, plus the ESP32's own report of its loop rate
docker exec rover bash -lc 'source /opt/ros/jazzy/setup.bash; \
  export ROS_DOMAIN_ID=0; python3 -u /logs/check_rates.py'

# per-wheel encoder calibration against a tape
docker exec -it rover bash -lc 'source /opt/ros/jazzy/setup.bash; \
  export ROS_DOMAIN_ID=0; python3 -u /logs/calibrate_encoders.py 200'

# are all four encoders alive? spin each wheel, any order, rover lifted
docker exec rover bash -lc 'source /opt/ros/jazzy/setup.bash; \
  export ROS_DOMAIN_ID=0; python3 -u /logs/wheels_selfpaced.py 60'
```

---

## 6. Reading the output

```
  source        x cm     y cm    th deg   straight cm   path cm      Hz
  cuvslam       -0.1      0.0     -0.00          0.1       0.0    30.0
  wheels         0.0      0.0      0.00          0.0       0.0    20.0
  gyro             —        —      0.07            —         —   200.9
  FUSED         -0.1      0.0      0.03          0.1       0.0    30.0
```

| column | meaning |
|---|---|
| `x`, `y` | displacement from the start, cm |
| `th` | heading change since the start, degrees |
| `straight` | straight-line distance from the start — **what a tape measures** |
| `path` | total distance travelled — larger if it wandered or reversed |
| `Hz` | publish rate. **Every failure on this rig began as a rate collapse.** |

Sources are shown **separately on purpose**. A single fused number cannot tell
you which sensor is lying: when cuVSLAM under-read a 100 cm push as 26 cm, a
fused value would have looked entirely plausible. Two rows disagreeing would not.

`FUSED` is the row to navigate on. The others are how you know it is honest.

Also watch:

- **`landmarks`** — under 30 and a teleport is coming
- **`gyro bias removed … STILL — retuning bias`** — the bias tracker working
- **`tilt:`** — roll, pitch, and the camera's measured mount pitch

---

## 7. What Phase 1 does **not** yet do

- **No runtime fused pose.** `FUSED` lives inside `compare.py`, a measurement
  tool. Nothing publishes it for nav2. This is Phase 1b and it is the critical path.
- **No teleport guard at runtime.** `compare.py` refuses to grade a run containing
  one; the live pose has no such protection. During autonomous navigation a 2 m
  teleport would make nav2 react violently to a position the rover was never in.
  This is a **safety** issue, not an accuracy one.
- **No `nav_msgs/Odometry` from the wheels.** `/wheel_odom` carries three numbers;
  something must wrap them properly.
- **No footprint.** nav2 needs the chassis outline to plan clearances.

Open faults are tracked in `TODO.md`.
