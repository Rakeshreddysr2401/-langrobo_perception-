# Phase 1 — Perception

**Goal: know where the rover is.** Every later phase stands on this. A map built
on a wrong pose is a wrong map, and nav2 plans against `odom → base_link`.

**Status: complete.** All gates pass. Measured on this rig, 2026-08-15 and
2026-08-21. Nothing here is inherited from a datasheet, a tutorial, or a previous
build — every number was measured against a tape measure or a floor line.

---

## 1. The rig

```
  D555 depth camera ──Ethernet/DDS──┐
                                    │
                              Jetson Orin Nano ── cuVSLAM, gyro re-framing,
                                    │              fusion, all measurement
                                    │
  ESP32 ──WiFi/UDP──► Pi 5 ──DDS────┘
   │                  (micro-ROS agent, teleop web)
   └── 4× BTS7960-driven encoder motors
```

| machine | role |
|---|---|
| Jetson Orin Nano | cuVSLAM, pose, fusion. Everything runs in `orin-nav:1.1` |
| Pi 5 | micro-ROS agent (ESP32 ↔ ROS), teleop web on `:8091` |
| ESP32 | 50 Hz PID control loop, encoder counting, motor drive |
| D555 | stereo IR + depth + IMU, over Ethernet, **not USB** |

---

## 2. The four sensors, and what each honestly measures

| sensor | rate | measures | trusted for |
|---|---|---|---|
| D555 stereo IR (896×504) | 30 Hz | image pair → cuVSLAM | **direction** |
| D555 IMU | 200 Hz | 3 rates + 3 accelerations | **heading**, **tilt** |
| 4× wheel encoders | 20 Hz | cumulative counts | **distance, straight** |
| D555 depth | 30 Hz | depth image | *nothing in Phase 1* |

### Why depth is deliberately unused

Depth is **computed from** the same stereo pair cuVSLAM already consumes. It is
not an independent witness — using it would be asking one witness the same
question twice and counting two answers. It is Phase 2's input, for the map.

### Why the accelerometer is never integrated for position

Integrating acceleration twice makes error grow with **t²**. A 0.01 m/s² bias —
entirely typical — is 0.5 m of error after ten seconds and **180 m after a
minute**. No filter fixes this; it is arithmetic.

Its real signal is **gravity**, which never drifts. That gives roll and pitch as
absolute angles with zero accumulated error — the only drift-free attitude
reference on the vehicle. It was being thrown away until 2026-08-21.

---

## 3. Techniques

### Stereo visual odometry — cuVSLAM 16.0.0

Tracks features between consecutive stereo frames and solves for the camera
motion that explains their movement. **Scale comes from the baseline** — the
known distance between the two lenses — which is why a wrong baseline gives a
proportional error, not a random one.

Loop closure is **off** on purpose. Phase 1 measures raw drift; loop closure
would mask exactly what we are trying to see.

### Frame conjugation

cuVSLAM reports the *camera* in *optical* axes (x right, y down, z forward). ROS
wants `base_link` in REP-103 (x forward, y left, z up). Both the axes and the
origin move, so the transform applies on **both sides**:

```
odom_from_base(t) = B · world_from_rig(t) · B⁻¹      B = base_link ← left_optical
```

A single multiply instead of a conjugation makes the rover appear to swing around
a point 17 cm in front of itself.

### Continuously re-estimated gyro bias

A MEMS gyro reads non-zero while perfectly still; integrate that and the heading
walks away linearly.

**The offset is not constant.** Measured across three runs the same day:
`+0.0838`, `+0.1050`, `+0.1808 °/s` — over 2× apart, moving as the board warms.
A startup average goes stale and starts *adding* error: one run over-corrected
and walked **−9.52° in 107 s with the rover standing still**, against a 10° gate.

Now: whenever the rover is definitely still, whatever the gyro reads **is** the
bias, and it is eased toward that (~25 s time constant). Result: **+0.08° over
161 s parked.**

**"Definitely still" requires both the wheels and cuVSLAM to agree.** Either
alone can be fooled — wheels by a slippery floor, cuVSLAM by a blank wall — and a
false "still" mid-turn would absorb real rotation into the bias and corrupt the
heading permanently.

### Complementary filters

Two sensors, opposite weaknesses, combined by frequency.

**Roll/pitch** — the gyro is smooth but drifts; the accelerometer is absolute but
noisy and reports fake tilt under acceleration. Integrate the gyro short-term,
let gravity pull it back long-term. τ = 0.25 s.

**Yaw** — gyro dominant, with cuVSLAM correcting only in a straight line
(see §4).

> **A complementary filter must be recursive on its own previous value.**
> Written as `fused = gyro + (vo − gyro)·k` it recomputes from the gyro every
> message and the correction never accumulates — it stayed 99.9% gyro no matter
> how long it ran, and reported −9.51° while cuVSLAM sat at +0.01°. It looked
> like fusion and did nothing.

### Encoder distance from raw cumulative ticks

Distance is integrated from **raw cumulative counts**, not from velocity and not
from a chord-accumulated path.

- **Velocity would have to be integrated**, so a dropped message loses that
  travel permanently. Counts are cumulative: however many messages go missing,
  the next one carries the exact total.
- **Chord accumulation has a 5 mm floor.** At 30 Hz the increment is usually
  exactly zero, which silently made dead reckoning report *"carried 0 frames"*
  through 32 teleports while the rover was moving the whole time.

Four separate wheel values also expose one wheel slipping, which a per-side
average hides.

### Chord-based path accumulation

Summing per-frame `|Δposition|` adds a magnitude that can never cancel, so `path`
ratcheted up **9.0 cm while the rover sat still** (position noise ≈ 26 µm/frame).
Travel accumulates only once the pose has moved 5 mm from the last anchor.

### Teleport rejection

A step is a teleport if it exceeds **15 cm** *or* implies over **1 m/s** — nobody
hand-pushes a rover at 4.4 m/s, so a 14.7 cm hop in one 33 ms frame is the
tracker re-initialising, not motion.

### Skid-steer effective track width

This rover has four driven wheels and **no steering**, so turning drags every
tyre sideways. The geometry converting a left/right speed difference into a yaw
rate is **not** the physical track — it is an effective width including the
scrub, and it is always larger.

Using the physical 0.34 m made the wheels **63% wrong on every turn**.

---

## 4. The fusion — how the three sensors cover for each other

No sensor is good at everything, and **a sensor that is bad at a job does not get
that job**. Fusion here is not averaging; it is assignment plus fallback.

| job | primary | why |
|---|---|---|
| **heading** | gyro | −0.2% to −0.9% across five measured turns |
| **distance** | encoders, straight only | front/rear agree 1.00× straight |
| **direction** | cuVSLAM | does not drift; re-measured against the world each frame |
| **tilt** | accelerometer (gravity) | absolute, never drifts |

### Every failure has a cover

| when this fails | this carries it | proven by |
|---|---|---|
| cuVSLAM goes blind (<30 landmarks) | wheels + gyro | dropped 501 frames in one run |
| cuVSLAM teleports | wheels + gyro | 101 frames, 65.1 cm dead-reckoned |
| cuVSLAM stops publishing entirely | wheels + gyro | killed `vo_node`; FUSED held 20 Hz |
| wheels slip in a turn | gyro heading | wheels said −31.51°, gyro +1.71° |
| wheels go stale | cuVSLAM distance | last good scale retained |
| gyro goes stale | cuVSLAM heading | a *frozen* gyro step is worse than none |
| gyro drifts long-term | cuVSLAM, straight-line only | bias also re-estimated continuously |

**FUSED has its own 20 Hz pulse**, independent of any single sensor. Until this
was added, both the fusion and its fallback were driven by cuVSLAM's callback —
so cuVSLAM was still the heartbeat, and if it went silent the pose simply froze.

### Two rules that matter more than they look

**Refusing bad messages is not enough — you must refuse a bad *sensor*.**
Rejecting individual teleports left cuVSLAM publishing at a confident 30 Hz
between them with an equally corrupted *direction*. With landmarks at 4, FUSED
took its heading from it and finished **42.4 cm** from the start while the wheels
**alone** managed 17.4 cm. Fusion did worse than its own worst input because it
was still listening to the broken one. Landmarks are the honest health signal —
a dead tracker still emits a perfect pose at a perfect rate.

**Do not calibrate during the manoeuvre that breaks your reference.** The
encoder/cuVSLAM scale ratio freezes while turning, because the wheels are
scrubbing and would drag a good calibration off with distance the rover never
went.

---

## 5. Measured facts about this rig

| quantity | value | how measured |
|---|---|---|
| camera x offset | 0.170 m | tape |
| camera z offset | 0.163 m | tape |
| camera mount yaw | 2.06° | tape |
| **camera mount pitch** | **−1.3°** | **gravity vector** — never previously known |
| stereo baseline | 9.49 cm reported | `−P[3]/P[0]`; ~9.70 would fix the scale error |
| wheel diameter | 0.085 m | tape |
| wheel base (physical) | 0.34 m | tape |
| **effective track width (turning)** | **0.5216 m** | four 360° runs, 1.5% spread |
| encoder CPR | **1560, verified** | 200 cm tape push, all four within 3% |
| cuVSLAM scale error | **−2.2%, systematic** | four tape measurements |
| cuVSLAM path over-read | ~19% | encoder cross-check between teleports |
| cuVSLAM failure driver | **landmarks, not speed** | 42 cm/s at 162 landmarks gave 0 teleports; 84 cm/s at 17 gave 12 |
| gyro bias | 0.084–0.181 °/s, **varies** | three runs, moves with temperature |
| healthy landmarks | 100–200; **<30 is fragile** | recorded at every teleport |
| yaw sign | **+z = left, REP-103 correct** | commanded-vs-measured check |

### Encoder calibration — 200 cm by tape

| wheel | counts | implied CPR | vs configured 1560 |
|---|---|---|---|
| LF | 11434 | 1526.6 | 0.98× |
| LR | 11378 | 1519.2 | 0.97× |
| RF | 11623 | 1551.9 | 0.99× |
| RR | 11275 | 1505.4 | 0.97× |

### Rotation calibration — four full turns

| turn | wheels read | truth | ratio | implied width | peak rate |
|---|---|---|---|---|---|
| 90 left | 146.49 | 90 | 1.628 | 0.5534 m | — |
| 360 left | 556.49 | 360 | 1.546 | 0.5256 m | 21.4 °/s |
| 360 left | 548.34 | 360 | 1.523 | 0.5179 m | 76.2 °/s |
| 360 left | 551.49 | 360 | 1.532 | 0.5209 m | 75.1 °/s |

Faster turns over-read slightly **less** — scrub is not a chassis dimension, it
is a calibrated average. Re-measure on carpet.

### Wheel scrub — the wheels only agree in a straight line

Two wheels on the same side, bolted to the same chassis, driven by the same
BTS7960, are mechanically obliged to sweep the same arc:

| | LEFT front/rear | RIGHT front/rear |
|---|---|---|
| straight (200 cm) | 1.00× | 1.03× |
| **turning (360°)** | **1.60×** | **1.29×** |

Repeated next run: 37.5% and 23.0% against 37.6% and 22.5% — **reproducible to
half a percent**, so it is deterministic, not random slip. The rover pivots about
a point behind its geometric centre and does so consistently.

---

## 6. Results

| gate | target | result |
|---|---|---|
| camera rate | ≥15 Hz | ✅ 30.0 Hz, emitter verified OFF |
| cuVSLAM rate | ≥10 Hz | ✅ 30.0 Hz |
| gyro rate | ≥50 Hz | ✅ 200.9 Hz |
| wheel rate | ≥15 Hz | ✅ 20.0 Hz |
| all 4 encoders | respond | ✅ verified individually |
| **scale** | 2.00 m ±5% | ✅ 195.4 cm (−2.3%) |
| **drift, hand-pushed** | ≤10 cm | ✅ 2.5 cm |
| **drift, driven hard** | ≤10 cm | ✅ **4.9 cm** through 12 teleports |
| **stationary stability** | no phantom motion | ✅ 0.08° over 161 s |
| **heading, 360° spin** | ≤10° | ✅ **3.68°** |
| **teleop** | moves and stops | ✅ balance 1.00 |

### The run that proves fusion works

Driven under teleop with repeated lefts, rights, forwards and reverses, in a room
with little for the camera to see. cuVSLAM lost tracking **12 times**, landmarks
fell to **17**.

| source | endpoint error | heading |
|---|---|---|
| cuvslam | 79.6 cm | +6.98° |
| wheels | 55.7 cm | −31.51° |
| gyro | — | +1.71° |
| **FUSED** | **4.9 cm** | **+1.75°** |

**FUSED beat both of its own inputs** — 16× better than cuVSLAM, 11× better than
the wheels — and its heading landed on the gyro's, the only one worth having.

The same test one iteration earlier gave **42.4 cm and FAILED**, worse than the
wheels alone.

### The 360° spin

Headings wrap to ±180, so unwrapped onto 360:

| source | read | error |
|---|---|---|
| wheels | 348.65° | −11.35° |
| **gyro** | **363.57°** | **+3.57°** |
| cuvslam | 371.15° | +11.15° **FAIL** |
| **FUSED** | **363.68°** | **+3.68° PASS** |

All four cluster around 360, confirming a real full rotation rather than a small
one passing by accident. The wheels at 3.2% error are the track-width calibration
paying off — they were 63% wrong before it.

### Teleop

Phone at `http://192.168.1.16:8091`, hold-to-move, 10 Hz, 0.4 s dead-man backed by
the ESP32's 500 ms watchdog.

| check | result |
|---|---|
| command reaches the wheels | ✅ phone → Pi 5 → `/cmd_vel` → WiFi → ESP32 → PID → motors |
| both sides drive | ✅ balance **1.00** — goes straight, DIR constants correct |
| PID tracking | ✅ 0.197 m/s measured against 0.200 commanded |
| release stops it | ✅ 0.197 → 0.043 → 0.004, a coast as the firmware intends |

**The page defaults to AUTO and the buttons do nothing until flipped to MANUAL.**
In MANUAL it publishes at 10 Hz whether or not a button is held, so `/cmd_vel`
traffic alone proves nothing — only a non-zero velocity does.

---

## 7. Faults found — and what each one teaches

Most of these presented as **healthy**. That is the lesson.

| fault | why it mattered | the general lesson |
|---|---|---|
| IR emitter silently ON (`:=0` is a no-op for a Boolean) | projector dots move *with* the rig; a 60 cm push read **1.1 cm** | set it at runtime and **read it back** |
| `/wheel_state` at 1.000 Hz | `loop()` blocked in `rclc_executor_spin_some`; two pointless reflashes chased the wrong theory | the mechanism was in a library nobody read |
| `docker exec` without a TTY | Ctrl-C never proxied — **every run's verdict and CSV was lost** | a test that cannot report is not a test |
| yaw filter not recursive | fusion appeared to work and did **nothing** | verify a filter changes the answer |
| gyro bias assumed constant | −9.52° phantom rotation while parked | a calibration can go stale |
| per-step scale ratio | could only ever inflate; path +28% | quantised inputs break per-step ratios |
| dead reckoning on chord path | reported "carried 0 frames" through 32 teleports | same trap, different place |
| trusting a degraded sensor | FUSED **worse** than its own worst input | refuse the sensor, not just the message |
| wheels using physical track | 63% heading error on every turn | skid-steer ≠ differential drive |
| camera "half-alive" | streams at 30 Hz but refuses option changes | a live stream is not a health check |
| `path` ratcheting | 9.0 cm accumulated while stationary | noise that cannot cancel accumulates |
| `ros2 topic hz` default QoS | silently receives nothing from a best-effort publisher | match QoS or measure nothing |

### Two false alarms worth remembering

**"Both LEFT encoders are dead."** The four-wheel test printed prompts through a
buffered pipe, so the operator never saw them and nothing was spun at the right
moment. *A test requiring the human and the script to agree on **when** is
fragile.*

**"Encoders are 2× out, rear reads 25% more than front."** That compared
cumulative counts — **arc length** — against cuVSLAM's `straight` — **displacement**.
On a path that curves or doubles back those are different quantities. *Calibrate
against a tape on a straight forward push, and against nothing else.*

---

## 8. Commands

### Bring-up — layered, each with its own PASS/FAIL

```bash
./rover camera      # D555 alone; verifies emitter OFF by read-back
./rover pose        # cuVSLAM + gyro, re-framed into base_link
./rover wheels      # ESP32 telemetry rate
./rover status      # every layer's rate, one screen
./rover values      # one-shot readout of every sensor
./rover stop        # tear down
```

Order matters — each layer checks the one beneath it, so a failure names its own
layer instead of hiding in a wall of log.

### The gates

```bash
./rover compare --expect 2.00    # push 2.00 m straight  -> want 190–210 cm
./rover compare --return         # out and back          -> want ≤ 10 cm
./rover compare --spin 360       # rotate 360°           -> want ≤ 10°
./rover compare                  # no gate: watch every sensor live
```

Keep still **5 seconds** (gyro calibration), drive, then **Ctrl-C** to grade and
write the CSV. Keep clutter in view — a bare wall starves the tracker.

### Teleop

`http://192.168.1.16:8091` on a phone. **Flip to MANUAL.**

| button | commanded | speed at the camera |
|---|---|---|
| forward / back | 0.20 m/s | 20 cm/s ✅ |
| left / right pivot | 2.0 rad/s | **34 cm/s** ⚠ over the tracking limit |

### Calibration and diagnostics

```bash
# all six streams + the ESP32's own report of its loop rate
python3 -u /logs/check_rates.py

# per-wheel encoder calibration against a tape
python3 -u /logs/calibrate_encoders.py 200

# effective track width, and per-wheel slip through a turn
python3 -u /logs/calibrate_rotation.py 360

# are all four encoders alive? rover lifted, spin each wheel, any order
python3 -u /logs/wheels_selfpaced.py 60

# does a LEFT command produce a POSITIVE yaw rate? (REP-103)
python3 -u /logs/check_yaw_sign.py 25

# watch a teleop button press travel all the way to the wheels
python3 -u /logs/watch_teleop.py 25
```

Each runs inside the container after
`source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0`.

### Logs

```bash
ls -lt logs/compare-*.csv | head
```

Per-source `x, y, th_deg, straight, path, hz` for cuvslam / wheels / gyro /
FUSED, plus the raw values: `roll_deg`, `pitch_deg`, `landmarks`,
`accel_x/y/z`, `gyro_x/y/z`, `velL`, `velR`, and all four tick counts. *A
surprising result is worth re-reading, and you cannot re-read what was never
written down.*

---

## 9. Reading the output

```
  source        x cm     y cm    th deg   straight cm   path cm      Hz
  cuvslam       -0.1      0.0     -0.00          0.1       0.0    30.0
  wheels         0.0      0.0      0.00          0.0       0.0    20.0
  gyro             —        —      0.07            —         —   200.9
  FUSED         -0.1      0.0      0.03          0.1       0.0    29.0

  health: cuvslam OK  wheels OK  gyro OK   |  all three contributing
```

| column | meaning |
|---|---|
| `x`, `y` | displacement from the start, cm |
| `th` | heading change since the start, degrees |
| `straight` | straight-line distance from the start — **what a tape measures** |
| `path` | total distance travelled — larger if it wandered or reversed |
| `Hz` | publish rate. **Every failure here began as a rate collapse.** |

**Sources are shown separately on purpose.** A single fused number cannot tell
you which sensor is lying: when cuVSLAM under-read a 100 cm push as 26 cm, a
fused value would have looked entirely plausible. Two rows disagreeing would not.
`FUSED` is the row to navigate on; the others are how you know it is honest.

Also watch:

- **`landmarks`** — under 30 and cuVSLAM is dropped from the fusion
- **`health:`** — who is covering for whom, right now
- **`STILL — retuning bias`** — the gyro bias tracker working
- **`encoder scale`** — the measured wheels/cuVSLAM ratio, and whether it clamped
- **`tilt:`** — roll, pitch, and the camera's measured mount pitch

---

## 10. Phase 1b — the fused pose is published

**Done 2026-08-21.** `FUSED` is no longer trapped inside a measurement tool.

| topic | rate | what |
|---|---|---|
| `/odom` | 20 Hz | `nav_msgs/Odometry`, fused pose + twist + covariance |
| TF `odom → base_link` | 20 Hz | **owned by the fusion node**, not `vo_node` |
| `/fusion/status` | 1 Hz | JSON health — which sensors are alive, what is covering |

```bash
./rover fused        # after ./rover camera and ./rover pose
```

### One algorithm, two consumers

`fusion.py` holds the estimator with no ROS in it. It was extracted from
`compare.py` **programmatically** — the method bodies are the same source text,
not retyped — so the thing validated over two days of tape-measured runs is the
thing that ships. `compare.py` grades it, `fusion_node.py` publishes it.

**Verified by running both at once and driving:**

| | straight | yaw |
|---|---|---|
| `/odom` (fusion_node) | 67.1 cm | +90.60° |
| `compare` FUSED | 66.5 cm | +91.07° |
| difference | **0.6 cm** | **0.47°** |

Separate instances, separate bias calibrations, separate start moments — so the
residual is startup timing, not a behavioural difference.

### The fusion node owns the transform

`vo_node` now runs `publish_tf:=false`. Only one thing may publish a transform,
and the raw cuVSLAM pose is the one that teleports. nav2 consuming that would
react violently to a position the rover was never in — a **safety** issue.

Two publishers of `odom → base_link` make TF non-deterministic, and the symptom
is a robot jittering between two poses with nothing in any log.

### Covariance, from the gates

| state | position σ | yaw σ | grounded in |
|---|---|---|---|
| healthy | 2 cm | 1.0° | 2.5 cm hand-pushed, 4.9 cm driven hard |
| degraded | 10 cm | 3.0° | the 4.9 cm run had 12 teleports |
| not ready | 1 m | 90° | before the gyro bias is measured |

`z`, roll and pitch carry `1e6` — this is a planar estimate, and claiming
certainty about a dimension nothing measures would be a lie to whatever consumes
it.

**Twist** comes from the sensors that measure velocity directly — forward speed
from the wheels, yaw rate from the gyro — rather than by differencing the
filtered pose, which would feed the filter's own lag back into control.

---

## 11. What Phase 1 does **not** do

- ~~No runtime fused pose~~ — **done, see §10.**
- ~~No teleport guard on the published pose~~ — **done**: the guarded estimate
  owns the TF and `/vo/odom` stays raw for comparison.
- **No footprint.** nav2 needs the chassis outline to plan clearances. Nothing
  has measured the rectangle or the camera's forward overhang.
- **The fused pose has never been driven by nav2.** It publishes; nothing has
  consumed it yet.

### Constraints nav2 will inherit

- **Watch landmarks, not the speedometer.** The old "25 cm/s limit" was a
  mis-attribution: 42 cm/s with 162 landmarks gave zero teleports, while 84 cm/s
  with 17 gave twelve. `/fusion/status` publishes the count and the fusion drops
  cuVSLAM below 30. See TODO §3
- **Pivots cost tracking** — 2.0 rad/s swings the camera at 34 cm/s. Prefer plans
  that turn and drive forward over plans that reverse or spin in place
- **The D555 drops out** — three distinct failure modes, see `TODO.md` §7. A robot
  needing a human to reseat a cable is not autonomous

Open faults are tracked in `TODO.md`.
