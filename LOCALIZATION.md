# Localization — holding x, y, θ through turns

**The goal (the owner's words, 2026-09-23):** from its current x, y, go to
x + x1, y + y1, and when asked to go back to x, y it must land in the exact
position and at the exact heading θ. In a known room or an unknown one, and
with no map saved across a power-off.

That needs the pose to stay right. It was right driving straight and wrong on
turns. This document is what was found about why, what was built, and what is
still open. Work of 2026-09-22 → 09-24; the chronological notes, with every
number, are docs/archive/TODO.md §42 and §43.

---

## Where it stands (2026-09-24)

| | state |
|---|---|
| heading through turns | **good**: ±1.6° per 90° with pure-`wz` turns, ≤0.56° per 45° in held-left mode |
| the rover's turns | **predictable now**: held-left turns pivot about one point, spread 1.5 cm |
| the pose's view of a turn | misses ~3–5 cm of the slide per turn (held-left), which slam corrects |
| LiDAR mount angle | **measured**: +88.60°, by driving |
| LiDAR timestamps | **fixed**: were ~82 ms early, now −2 ms |
| `./rover drive` exact moves | **1.4 cm / +0.4°** (slam) and **1.9 cm / −2.2°** (walls) for a +90° in place, after the timestamp fix |
| exact return to a marked spot | **not yet tested on the floor** |
| which heading is true to 1–2° | **not known** — needs the tile test (below) |

---

## 1. The pose, as built now

```
 wheels ─┐
 gyro   ─┼─► fusion_node ──► odom -> base_link       dead reckoning: smooth, drifts
 cuVSLAM┘                    (gyro heading, cuVSLAM direction, wheel distance)

 LiDAR /scan ─► slam_toolbox ──► map -> odom          the correction: what the walls
                                 /map                 say dead reckoning got wrong

                map -> base_link = the corrected pose, and what ./rover drive steers on
```

**One owner per transform.** `odom -> base_link` belongs to `fusion_node`;
`map -> odom` belongs to slam_toolbox. cuVSLAM also publishes `map -> odom`
when its loop closure is on (`slam:=true`, `./rover pose`'s default), so
`./rover up` now starts pose with `SLAM=false`, and `./rover slam` refuses to
start if cuVSLAM already owns the frame. nav2 and the Pi 5 brain still plan in
`odom`; see §6.

---

## 2. What was found, in order of how much it mattered

### 2.1 The rover cannot turn about its centre

Commanded to pivot, the left side cannot reverse against the tyres' sideways
scrub. The right side then swings the rover around the planted left tyres.

| measurement | value |
|---|---|
| pivot speed reached (wz 0.4–2.5 rad/s swept) | ~7–11% of command, never above 5% of commanded yaw rate |
| straight driving, same wheels | over 100% of command (0.051 m/s for 0.050) |
| real slide per 90° "in place", by the walls | **27.8–32.8 cm** (four turns) |
| where it pivots (pure `wz`) | 7 of 8 turns about **(4, 21) cm** from centre, ±1.5 cm — the outer edge of the left tyres |
| the 8th turn | pivoted near the **centre**: the left side got going that time |

That last row is the real problem: a turn that usually does one thing and
sometimes another cannot be planned around.

**Ruled out:** the battery (full, ~12 V, and the firmware's PI drives a lagging
side to full duty within ~2–3 s, so full voltage is the best case); the floor
(tile is low-friction, and slipping wheels would make the encoders read *high*,
not low); the firmware (the source's mixing is correct and nothing overrides
the default gains). What is left is **torque against scrub**: rough arithmetic
puts a pivot at ~3 kg·cm per wheel, near a 200 RPM GB37's stall torque. The
**lift test** (§6) is what settles whether the left side's deficit is the
motor/gearbox or its driver and wiring.

### 2.2 The pose sees most of the slide, not all of it

Graded against the walls (`./rover pivot`), odometry reports the slide but
over-reads it by ~12%: **6–8 cm missed per 90° pure-`wz` turn**, 3–5 cm in
held-left mode. That is the part that is genuinely a localization error; the
rest of the slide is real motion the pose knows about. slam_toolbox corrects
it against the walls.

### 2.3 The LiDAR stamped every scan ~82 ms early

slam_toolbox looks the pose up at each scan's timestamp. `sllidar_node` reads
the clock *before* blocking for the next full revolution, so the stamp led the
measurement by most of a scan period. During a turn every scan was matched
against a pose ~0.6° away, in the direction of the turn, and the error added
up: two +90° turns the same way left slam with an **8° heading correction**
that odometry and direct wall registration both contradicted, and
`./rover drive`, steering to slam's heading, over-turned by ~10°.

`./rover lidar --lag` measured it: **+68 / +94 / +85 ms** over three runs,
same sign every time. Fixed in the driver
(`lidar/patches/0001-scan-time-offset.patch`, `LIDAR_TIME_OFFSET=0.082`);
re-measured at **−2 ms**. Results from before this fix that involve turning
are suspect.

### 2.4 The LiDAR's mount angle was unknown

The C1's case does not mark its zero beam. Two values set from descriptions of
the RViz picture (0°, then −90°) both had the **sign inverted**.
`./rover lidar --calibrate` measures it by driving: the whole room slides in
the laser frame and the direction it slides is the angle. **+1.5463 rad
(+88.60°)**, five legs, spread 0.76°, ICP residuals 0.5–0.7 cm, and the
LiDAR's and odometry's distances for the same move agreeing to 1%. The mount is
genuinely 1.4° off a clean quarter turn.

### 2.5 Silent faults, each with every gate green

| fault | effect | fix |
|---|---|---|
| `./rover pose` killed every static TF | the LiDAR's frame vanished on every full bring-up; RViz drew nothing, slam could never have worked | narrow `kill_match`; `scan_check.py` asserts the frame |
| `scan_check.py` ignored the TF | its front/left/back/right were the LiDAR's, not the rover's | resolves through the live TF |
| two owners of `map -> odom` | cuVSLAM and slam_toolbox would have fought | ownership guard; `up` runs `SLAM=false` |
| `SLAM=false ./rover pose` did nothing | expanded inside the container, where it was never set | expanded on the host |
| "Wheels are live" with motor power off | the gate reads telemetry, not motion | `./rover wheels --nudge` |
| a calibration tool held the raw depth stream | took the D555 offline; cuVSLAM and the gyro (the camera's IMU) went with it | drops the subscription after one frame |

---

## 3. Making turns predictable: the left side is HELD

The firmware mixes `wL = vx − wz·0.17`, `wR = vx + wz·0.17`, and switches a
side **off** when its target is under 0.01 m/s. The ways to turn, as measured:

| mode | command | result |
|---|---|---|
| pure `wz` | `vx = 0` | left asked to reverse at full speed; usually stalls (pivot at the left tyres), sometimes does not (pivot at the centre) — a coin flip |
| pure `wz` at **5.0 rad/s** (the brain's turns) | `vx = 0` | worse: 31–68 cm per turn, pivot spread 36 cm, one stall at 67° of 90° (2026-09-24) |
| left **off** | `vx = wz·0.17` | left free-rolls behind the right: pivot **75 cm** out, ~60 cm of slide per 45° |
| left **held** | `vx = wz·0.17 − sign(wz)·0.02` | left PI holds ~0.02 m/s: it can neither free-roll nor reverse properly |

Held-left, four 45° turns: slide **21.0–21.6 cm**, pivot spread **1.5 cm**,
heading error **≤0.56°**. The pivot depends on direction:

| | 45° | 90° |
|---|---|---|
| left turns | (3.1, 27.7) cm | (4.7, 24.9) cm |
| right turns | (0.3, 27.1) cm | (−0.7, 37.6) cm |

`PIVOT_ANCHOR=hold` is the default in `./rover drive`. It also stops the left
motor sitting at full duty against a speed it cannot reach.

---

## 4. `./rover drive` — plan around the slide instead of fighting it

`phase3/nodes/pivot_goto.py`. Every move is **rotate → straight → rotate →
straight**, about the measured pivot for each direction. Rotating θ about a
pivot P moves the centre by `(I − R(θ))·P`; the straight legs are sized to
cancel that exactly. With a second straight leg every split of the heading
change has an exact plan, and it takes the one that turns least, because
turning is where the error comes from. It re-plans from the LiDAR-corrected
pose after each pass, and grades the end pose a second time by registering the
start and end scans directly, with no slam involved.

```
./rover drive 0 0 90 --rel       turn 90° left without the centre moving
./rover drive 0.4 0.3 0 --rel    go to x + 0.4, y + 0.3, same heading
./rover drive --mark home        remember this pose (map frame)
./rover drive --to home          come back to it
```

**Safety**, before every segment: a map frame, teleop in AUTO, legs ≤ 1.5 m,
the LiDAR showing 8 cm clear of the exact area the planned turn sweeps and of
the straight lane. The LiDAR sees one plane at 21 cm and nothing below it. A
human watches.

### Results so far

| run | planner | before timestamp fix? | slam says | walls say |
|---|---|---|---|---|
| +90° in place | pure-wz pivot model, 1 leg | yes | 1.0 cm, +0.8° | 1.2 cm, −1.6° |
| −90° in place | pure-wz model | yes | 8.4 cm after 4 passes (pass 1: 35 cm — the coin flip) | 8.6 cm |
| +90° in place | held-left, 1 leg | yes | 4.4 cm, +4.1° | 2.8 cm, −6.6° |
| +90° in place | held-left, 2 legs | yes | 0.3 cm, −0.9° | 3.4 cm, +9.8° |
| **+90° in place** | **held-left, 2 legs** | **no** | **1.4 cm, +0.4°** | **1.9 cm, −2.2°** |

Before the timestamp fix the two judges disagreed on heading by ~10°; after,
by ~2.6°. Which of them is right to 1–2° is what the tile test settles.

---

## 5. Tools added

| command | what it does | moves? |
|---|---|---|
| `./rover lidar` | C1 → `/scan`, `base_link → laser`; asserts the frame resolves | no |
| `./rover lidar --calibrate` | measures `LIDAR_YAW` by driving out and back | **yes**, 0.20 m legs |
| `./rover lidar --watch` | live: where the nearest object renders on the rover | no |
| `./rover lidar --yaw` | one-shot yaw from a target held at the nose | no |
| `./rover lidar --lag` | measures the scan timestamp error | **yes**, short turns |
| `./rover slam` / `--check` | slam_toolbox → `map -> odom`; prints the correction | no |
| `./rover pivot [deg …]` | grades turns against the walls: real slide, pose error, pivot point | **yes**, spins |
| `./rover drive …` | exact short moves, see §4 | **yes** |
| `./rover wheels --nudge` | proves the motors are powered, not just the link | **yes**, ~7 cm |
| `./rover view --restart` | RViz re-reads its config (it only does at startup) | no |

RViz (`phase2/rviz/rover_live.rviz`): Fixed Frame is now `map`, so the room
holds still and the rover moves through it; two track lines, green
(LiDAR-corrected) over red (raw odometry), whose gap is the drift; the rover
drawn at its described size with the nav2 footprint outline and the LiDAR puck.

---

## 6. Open, in the order worth doing

1. **Tile test — which heading is true.** Square the rover to a grout line,
   tape an X under its centre, run `./rover drive 0 0 90 --rel`, check it is
   square to the crossing line and over the X. Settles slam vs walls to 1–2°.
2. **Go-and-return test — the goal itself.** `--mark home`, drive away with
   turns, `--to home`, measure from the X with a tape.
3. **Lift test.** Wheels off the floor, command a pivot. Full speed in the air
   → the left side loses to floor load (motor, gearbox). Still weak → its
   driver or wiring.
4. **Tape measurements, then a URDF** — planned in full in
   [ROVER_BUILD_PLAN.md](ROVER_BUILD_PLAN.md), with CAD, power (voltage
   telemetry, sag) and weight distribution. Geometry lives in four places that
   disagree: the firmware's `WHEEL_BASE_M` is 0.34 m, the RViz model's wheel
   centres are 0.385 m apart, and the camera (x 0.17) and LiDAR (x 0.135)
   positions are from a description. Measure from the midpoint of the four
   wheels: track, wheelbase, and the camera's and LiDAR's x and height. Then
   one URDF feeds the TFs, the RViz model and nav2's footprint.
5. ~~**Grade the brain's turns.**~~ **Done 2026-09-24:** `PIVOT_WZ=5.0 ./rover
   pivot`, six turns. The 2026-08-22 "5.0 pivots cleanly" does not hold: slides
   of **31–68 cm** per turn, pivot points spread **36 cm** (the first two sat on
   the *right* tyres, 0.4 cm apart; the next four went anywhere), pose error
   0.8–8.9 cm, and **one turn stalled at 67° of 90°**, which a timed turn would
   report as done. Heading stayed good (≤0.8°). The brain's turns should move
   to held-left, closed-loop on the gyro, or to `./rover drive`.
6. **nav2 and the brain in `map`.** The correction is used by `./rover drive`
   but not yet by nav2 or the brain. That is a driven change across both repos
   (phase2/config/SLAM.md) and should wait for 1–2.
7. **Give the brain `./rover drive`.** It is a script on the Jetson; the brain
   cannot call it yet.

## 7. Stage A baseline, 2026-09-24 (partial): graded against LiDAR truth

The first runs of the new harness (`phase1/harness/`, SENSOR_FUSION_PLAN.md
§5), on the new geometry (robot_state_publisher; vo_node reading the camera
mount from TF). Truth is the LiDAR scan at each stop matched against the
start. Every checkpoint below passed the residual and two-guess gates.
Errors: cm / degrees at the last checkpoint.

| run | what really happened (truth) | fused | vo | wheels | wheels+gyro |
|---|---|---|---|---|---|
| `023642_still`, 34 s parked | nothing moved | 0.2 / **0.53** | 0.2 / 0.00 | 0.0 / 0.00 | 0.0 / 0.12 |
| `023804_straight`, 0.5 m out and back | out **50.3 cm**, 1.1 cm sideways; back to −0.5 / +0.6 cm, heading −0.93° | 2.1 / 0.22 (3.1 at the far end) | 2.0 / 1.17 | 0.2 / 0.25 (2.2 at the far end) | 0.3 / 0.40 |
| `023845_pivot90`, stopped after 2 of 8 turns | after +90: **slid to (−18.9, −27.6) cm**; after +180: (+17.3, −49.5) cm | **19.3 / 4.94** | 16.8 / 0.80 | 11.4 / 37.8 | 16.8 / 1.51 |

What it says:

- **The wheels over-read distance by ~1.4%.** They said 51.0 cm where the
  truth measured 50.3 cm, in the direction the 85 mm (firmware) vs 83 mm
  (tape) difference predicts (2.4%). That is one 0.5 m run, not the taped 1 m
  drive, but it points the same way.
- **The fused heading drifts while parked:** 0.53° in 34 s, where the gyro
  plus wheels held 0.12° and VO held 0.00°. Stationary drift is exactly
  what the zero-velocity lock (SENSOR_FUSION_PLAN.md §3.1) is for.
- **In the pivot, the fused heading was worse than the gyro alone:** 4.94°
  against 1.51° after 180°. The wheels' own heading was off by 37.8°.
- **Position through a pivot is poor for every estimator** (11-19 cm after
  two turns), because the rover slides ~33 cm per 90°. That is the case the
  LiDAR odometry (stage C) and the closed-loop pivot (stage E) exist for.

### The pivot run stalled, and the traces say why

- **Actual rate ~0.06 rad/s (3°/s) for a commanded 1.5 rad/s.** One 90° took
  30 s, the next 55 s. nav2.yaml records ~0.21 rad/s actual for 1.0
  commanded, so the rover turned roughly three times slower than before.
- **In a left (CCW) turn the RIGHT side barely drove:** left −5685 / −3978
  ticks (front/rear), right +105 / +54. The rover pivoted about its right
  side. That is the 33 cm slide.
- **Then the right side ran BACKWARDS while commanded forward.** From about
  84 s, and through all of the third turn, right went −22 → −2794 ticks with
  wz +1.5 commanded. There were also stretches where the left wheels turned
  and the gyro did not: pure slip.
- The third turn stalled at +38.7° (gyro). Driving stopped there: the motors
  were sitting near stall for minutes.

Leading suspect: a **low battery**, which build plan §0 predicts would take
the pivot's torque margin to zero. Nothing measures the voltage yet (the
INA226, build plan §4). The right side reversing under a forward command is a
separate question for the firmware's PI loop, or for the wiring on that side.

**Still to record, after a charge:** the rest of `pivot90`, `pivot360`,
`square`, `small`.

## 8. Stage A baseline, 2026-09-25: complete, and what it found

Charged pack, motors powered, new geometry. `logs/bags/SUMMARY.md` has every
run; all checkpoints passed the truth gates (26 usable of 26). Errors are
cm / degrees at the last checkpoint, with the worst in brackets.

| run | fused (today's fusion.py) | vo | wheels | wheels+gyro |
|---|---|---|---|---|
| `pivot90`, 6 of 8 turns | 13.5 (16.2) / 2.53 | 13.4 (16.5) / 0.43 | 17.7 / 86.4 | 14.6 / 1.73 |
| `pivot360`, +360 then −360 | 3.8 / 1.61 | 3.7 / 1.18 | 4.6 / 51.7 | 2.4 / 2.39 |
| `turn`, one +90 | 13.3 / 0.17 | 13.6 / 0.28 | 6.0 / 18.5 | 5.8 / 0.16 |
| `square`, 4 × (0.4 m, +90) | 5.0 (19.4) / 0.16 | 4.8 (19.6) / 4.28 | 65.5 / 85.5 | 16.8 / 3.98 |

`small` was not recorded: by then the rover was boxed in (3 cm clear ahead,
1 cm to the right).

### The finding: the camera's lever arm had the wrong sign

After every turn, fused and VO were ~13 cm off even when the rover had barely
moved: in `turn`, the truth moved 6 cm and fused was off by 13.3 cm, while
wheels+gyro was off by only 5.8. That is a lever-arm error. The recorded VO
was re-projected with candidate camera offsets and fitted to the LiDAR truth,
26 checkpoints over 7 runs:

| camera0_link (x, y) | mean | max |
|---|---|---|
| as run, (0.173, −0.0475) | 8.0 cm | 19.6 cm |
| the old (0.173, 0) | 4.4 cm | 10.1 cm |
| flipped, (0.173, +0.0475) | 2.1 cm | 8.1 cm |
| **fit (0.1623, +0.0419) ± 3 mm** | **2.0 cm** | **5.7 cm** |

Leave-one-run-out, fitting on the other six each time, every turning run
improves: held-out max 16.8 → 6.8, 16.5 → 3.7, 13.6 → 1.3, 19.6 → 4.8 cm.

- **The left imager is on the rover's LEFT.** The D555 driver's TF puts infra2
  at +0.095 y from infra1, which read as "infra1 on the right". That sign is
  wrong for this rig, and 2026-09-24 trusted it.
- **This also explains TODO 43's "pose misses ~7 cm per turn".** The old y = 0
  is 4.75 cm off, and 0.0475 × √2 = 6.7 cm per 90°. It was never the fusion;
  it was the camera's lever arm.
- The optical centre sits 1.1 cm behind the front glass.
- The fit is now in `description/params.yaml` (`vo_lever_x/y`, `infra2_side
  −1`) and live: vo_node reads it from TF.
- **The fused rows above were recorded with the wrong lever arm.** They are the
  bar for "the old system", not for fusion.py itself; re-record the turning
  runs to see fusion.py with the right geometry.

### Also measured

- ~~The gyro under-reads by about 2%.~~ **Withdrawn 2026-09-26:** over 21 turns
  graded against LiDAR truth (3 × `pivot360` each way plus every earlier turn
  over 45°), the gyro scale is **1.0010**, per turn ±0.7%, 1.1° rms. The "2%"
  came from one turn (2.6% off; the run's other turns agreed within 0.3%)
  and a trace read mid-motion. Turns do overshoot 1-3°, but because the
  runner integrates the gyro on arrival time and the rover coasts after the
  stop. That belongs to motion control (M4), not the gyro.
- **The pivot slide depends on the battery and varies.** On the charged pack
  it was about 6 cm per 90° in `pivot90`, but 22 cm for the first turn of the
  square, against 33 cm on 2026-09-24's weak pack. The weak-pack pivot ran at
  3°/s; charged, 3-4 s per 90°.
- **One turn froze.** The seventh pivot got a −1.5 command, but the encoders did
  not change at all for 8 s. The link and firmware were healthy (loop 507 Hz,
  agent connected), and the motors answered a nudge right after. This fits
  the BTS7960s' over-current or thermal protection tripping after six hard
  pivots near stall. Unproven without current sensing (the INA226, build
  plan §4).
- **Wheel distance is ±1-3% with no fixed sign:** 50.3 true for 51.0 read on
  2026-09-24, and 42.1 true for 41.0 read in the square. That is slip and
  start/stop, not only the 83 vs 85 mm question.
- **Something on the rover's rear-right corner now reaches the laser plane**,
  at base_link (−0.176, −0.14). It blocked every turn until the harness,
  pivot_goto and pivot_test got a self-filter. It also shadows part of the
  LiDAR's view behind and to the right. Identify it, and add it to
  `params.yaml` as a component.

## 9. Calibrated baseline, 2026-09-26 (after M1)

All with the M1 calibration live: lever arm, camera roll and yaw. Errors are
cm / degrees at the last checkpoint.

| run | fused | vo | wheels | wheels+gyro |
|---|---|---|---|---|
| `straight` 1.0 m out (the return leg was refused: the start was 17 cm from a wall) | **0.5 / 0.02** | 0.6 / 0.01 | 3.9 / 0.80 | 4.4 / 0.30 |
| `pivot360` × 3 (+360, −360) | **1.6 / 0.08**, 3.1 / 0.82, 0.5 / 0.53 | 1.2 / 0.63, 2.4 / 0.16, 0.2 / 0.27 | 1.8-7.0 / 34-37 | 1.2-4.3 / 0.4-3.5 |
| `small` (±10°, ±5°, ±5 cm) | **0.4 / 0.71** | 0.4 / 0.33 | 0.5 / 1.35 | 0.5 / 0.28 |
| `turn` +90 | 3.2 / 0.01 | 3.0 / 0.12 | 7.1 / 18.5 | 6.9 / 0.58 |
| `square` 4 × 0.4 m | 2.4 (3.9) / 2.48 | **0.9** (3.6) / 0.06 | 37.3 / 70.4 | 1.6 (8.3) / 1.39 |
| `pivot90` (8 turns) | 6.0 / **12.96** | 7.3 / 8.23 | 30.3 / 54.4 | 15.2 / 1.95 |

Against §8 (before M1), full turns went from 3.8 cm / 1.6° to 0.5-3 cm / under 1°.

### Two findings from `pivot90`

- **The fusion took on a VO heading glitch.** Mid-run, VO's landmarks
  dropped to 36 during the spins and its heading jumped 7.6° between two
  stops. fusion.py trusts VO's heading whenever the rover is still, so it
  absorbed the jump and ended **13° off**, while the raw gyro with the wheels
  stayed within **2°**. This is exactly the hard-switch weakness M3 replaces:
  weight VO by its confidence, and gate a sudden jump.
- **The harness runner lost count of rotation.** It integrated the gyro from an
  index into a buffer it trims every ~20 s. After a trim, ~10 s of rotation
  went uncounted while the motors ran, and one "+90" really went ~112°. Fixed:
  it now integrates in the callback on the IMU's own stamps. Truth was never
  affected: every graded number above is still valid.

### The drive froze again after long near-stall pivots

In a re-run of `pivot90`, turn 3 crawled for ~12 s (one rear wheel nearly
stopped), then turn 4 froze completely: all four encoders were still for 8 s
under a full +1.5 command, while telemetry kept arriving. A nudge right
afterwards drove normally. Same pattern as 2026-09-25 turn 7. The firmware has
no stall cutoff; its only gates are the agent connection (up throughout) and a
500 ms /cmd_vel watchdog. So it is one of:

- **the command link stuttering over Wi-Fi**: the watchdog zeroes the motors
  while telemetry flows fine the other way. That would also produce the
  crawling; or
- **the BTS7960s cutting out** on over-current after long near-stall, made
  likelier by a pack drained by the session's pivots.

Nothing measures the pack or the motor current yet (INA226, build plan §4),
and nothing reports "watchdog fired" (an M4 firmware item). Pivot tests
stopped here to spare the motors.

## 10. M2: LiDAR odometry, offline on the recorded runs (2026-09-26)

`phase1/nodes/lidar_odom.py`, with no ROS inside it. Every scan is de-skewed
with the gyro, then matched point-to-line (Huber) against a submap of the last
10 keyframes (keyframe every 10 cm or 10°, 3 cm voxels). Each fit carries a
residual, an inlier count and a degeneracy eigenvalue; a failed fit falls back
to the prediction and is flagged. Graded as the `lidar` row in
`./rover grade`.

**De-skew, decided by grading** (13 turning runs, 43 checkpoints; position
mean / max, heading mean / max):

| de-skew | position | heading |
|---|---|---|
| off | 0.61 / 1.57 cm | 1.32 / 5.31° |
| beams forward in time | 0.87-1.04 / 2.3-2.8 cm | 2.7-3.2 / 12-14° |
| **beams backward in time** (the C1 spins clockwise) | **0.27 / 0.88 cm** | **0.17 / 0.39°** |

**Over all 17 runs** (`logs/bags/SUMMARY.md`), LiDAR odometry's worst
checkpoint is **0.9 cm and 0.39°**. On the same runs fused reaches 19 cm /
13° and VO 17 cm / 8°. It does not use the camera, so the pre-calibration
runs are as good as the rest (2026-09-24 `pivot90`: lidar 0.4 cm, fused
19.3 cm). No fit failed. It takes 5.9 ms per scan (p95 11 ms) on the Orin
with the full stack running, against a 100 ms budget.

**What this does and does not prove.** The truth also comes from the LiDAR,
so a shared LiDAR error (its mount yaw, a range scale) would cancel out of
both. The spin and square runs are a real test: by the checkpoint the
odometry had chained dozens of keyframes and dropped the early ones. `small`
and short `straight` runs are close to self-comparison, because the submap
still holds the reference scan. **The independent check is the owner's
tape-marked `return` run**, and a long `zigzag`, both still to record.

**Next in M2:** run it live as a node (`/lidar/odom` with covariance),
record it, and compare live against offline; add a `zigzag` scenario; then
the owner's `return` test with tape marks. After that, M3 fuses it.

### The owner's `return` test: the independent check (2026-09-26)

`20260926-004818_return`. The owner put tape marks at three corners, moved the
rover by hand (about 120-145 s into the run), and set it back on the marks.

| | error at the end |
|---|---|
| LiDAR truth: where it really ended vs its start | **0.9 cm, 0.1°**, consistent with it sitting back on the tape |
| **LiDAR odometry, live** | **0.3 cm / 0.24°** from the truth, so within ~1 cm / 0.3° of the marks |
| LiDAR odometry, offline | 0.0 cm / 0.14° |
| fused | 20.5 cm / 0.68°, 3 pose jumps |
| VO | 8.9 cm / 6.0°, 2 jumps |
| wheels+gyro | 24.9 cm / 6.9° |
| wheels | 41.7 cm / 4.9° |

The tape and the LiDAR truth agree, which checks the truth method from outside.
Being moved by hand is the "picked up and carried" case: the wheels saw
motion they did not cause, VO and the fusion jumped, and the LiDAR odometry
tracked it through. **M2's acceptance (≤ 2 cm, ≤ 0.5°) holds on every recorded
run and on the independent return test.** Still open in M2: a motor-driven
`zigzag`, which waits for the power work (build plan §4.5).

## 11. M3: fusion2, offline on 19 recorded runs (2026-09-26)

`phase1/nodes/fusion2.py`, with no ROS inside it. One EKF over [x, y, θ, vx,
vy, gyro bias]. The gyro predicts at 200 Hz. Corrections, each gated
(Mahalanobis) and weighted by its own evidence:

- **LiDAR odometry:** an absolute pose through a re-anchored offset. A late
  fix (it arrives ~0.13 s after its stamp) is carried forward by motion only.
- **VO, adaptive:** velocity while the LiDAR is healthy, an absolute pose when
  it is not. Noise scales with landmarks, and jumps fail the gate.
- **Wheels:** forward speed and "no sideways speed", with noise that grows
  with the turn rate and with slip evidence (front vs rear; wheels vs gyro).
- **Certain stillness:** encoders unchanged and nothing commanded pins the
  velocity and re-measures the gyro bias.

Graded with the LiDAR fixes fed at their live delay (worst checkpoint per
run: the worst over all runs, and the mean of the per-run worsts):

| | worst | mean of worst |
|---|---|---|
| fused (today's fusion.py) | 20.5 cm / 12.96° | 6.4 cm / 1.68° |
| LiDAR odometry alone | 0.9 cm / 0.39° | 0.3 cm / 0.18° |
| **fused2** | **0.8 cm / 0.45°** | **0.3 cm / 0.19°** |
| fused2 without the LiDAR | 19.6 cm / 4.21° | 6.2 cm / 1.43° |
| VO alone | 19.6 cm / 8.38° | 5.6 cm / 1.76° |

With the LiDAR, fused2 keeps its accuracy. Without it (corridors, outdoors),
it beats today's fusion everywhere, most in heading: worst 4.2° against 13°.
The `pivot90` VO glitch is rejected (0.36° against fused's 12.96°).

Three design mistakes the grades caught on the way, recorded because each
looked reasonable:

1. **Rejecting healthy LiDAR fixes that failed the gate**, and re-anchoring
   after five in a row, locked drift in: 29.5 cm on `pivot90`. A skid-steer
   pivot slides faster than the velocity states follow, so the filter, not
   the LiDAR, was wrong. Now a healthy fix that fails the gate widens our
   uncertainty and is taken. Only a physically impossible jump (> 30 cm or
   > 10° in one scan) counts as a LiDAR fault.
2. **Carrying late fixes forward with the corrected state's history** fed our
   own corrections back into the next measurement. Parked, it wandered
   10-20 cm. Now it uses a motion-only dead-reckoned track.
3. **VO as an absolute pose all the time** fought the healthy LiDAR (1.8 cm
   worst). As velocity only, it wasted VO's position tracking when the LiDAR
   was absent. Adaptive takes the better of each.

**Not yet shown:** a corridor where the LiDAR degenerates, and slip / stuck
/ lifted flags on a run built to trigger them. Next: run fusion2 live beside
fusion.py (`/fused2/odom`, no TF), record, compare live against offline; then
decide with the owner when it takes over `odom → base_link`.

### Live: the owner's second `return` test, fusion2 running (2026-09-26)

`20260926-013329_return`. Moved by hand and turned. The owner reported the
wheels skidding under the hand at times. Put back on the tape marks within
0.1-0.2 cm / 0.04° (LiDAR truth).

| | final (worst during the run) |
|---|---|
| **fusion2, live** | **0.1 cm / 0.32°** (0.2 cm / 0.38°) |
| fusion2, offline on the same bag | 0.2 cm / 0.29° |
| LiDAR odometry, live | 0.1 cm / 0.30° |
| fused.py | 1.8 cm / 0.74° (**9.7 cm**) |
| VO | 4.5 cm / 0.15° |
| wheels+gyro | 24.5 cm / 5.17° (31.6 cm) |

Live matches offline. The skid showed up in fusion2's slip score (max 0.74),
so the wheels were down-weighted and the fused pose held 0.1 cm while the
wheels were 24 cm off. Over the run: LiDAR 4900 accepted / 0 rejected; VO
2105 / 4, the 4 being VO jumps; wheels 9390 / 0, at slip-scaled weight.
**M3's acceptance holds offline over 19 runs and live on an independent
return test.** Not yet exercised: a LiDAR-degenerate corridor, and the
stuck / lifted flags.

### The switch (2026-09-26): fusion2 owns odom → base_link

At the owner's call after the live return test, `./rover fused` now runs
fusion2 on `/odom` with the TF (FUSION=2, the default). `fusion.py` runs
beside it on `/odom_legacy` without TF as the fallback; `FUSION=1 ./rover
fused` swaps them back. SLAM, nvblox and nav2 were restarted on the new odom.
Checked live: one process of each, `odom → base_link` on `/tf`, `/odom` at
20 Hz. fusion2 takes 10.1 LiDAR fixes/s, 3.9 VO/s (64 VO glitches rejected),
the wheels and the still-lock; its own sd is 0.56 cm / 0.1°.

The first switch attempt started a fusion2 process that never joined the ROS
graph: no `/odom`, no TF, no status, though the process ran. It was reverted
to FUSION=1 at once, and the retry came up cleanly. The cause is not known.
**The fused layer's gate must be read after the node is up**: it checks
`/odom` at 15 Hz, which that failure would have failed. Watch for it on
future starts. The ROS CLI tools also dropped `/fusion2` from `node list` and
missed TF frames in short samples; use a 6 s+ sample or a Python probe before
concluding anything is missing.

nav2 has not yet driven on fusion2: the first drive is short and watched, on
a charged pack (build plan §4.5).

## 12. M4: the goal executor, proven in simulation (2026-09-26)

`phase3/nodes/goal_exec.py` (no ROS), `goal_exec_node.py` (live), `./rover goto
X Y [DEG]`, and the VLM brain can send the same goals on `/goal_exec/goal`.

**How it moves: approach along the goal line.** The goal (x, y, θ) defines a
line. The rover goes to a pre-goal 25 cm behind the goal on that line (turn →
straight → turn, both turns' slides pre-compensated), then drives straight
along the line to the goal, holding θ and steering out cross-track error. If it
ends off the line, it backs up along the line and approaches again, with no
turn in place. The last motion is always a straight drive, so no final turn can
slide it off.

**The slide is learned, not assumed.** Every turn measures where the rover
really pivoted: P = (I − R(a))⁻¹ d, per direction, smoothed. P is saved to
`/logs/goal_exec_pivot.json` after every goal, so a session starts with what the
last one learned.

**Safety before and during every primitive:**
- a turn: the outline + 5 cm swept about the learned P, against the scan
- a straight: the outline + 5 cm swept along the leg
- no progress for 6 s: stop
- pose unsure (LiDAR unhealthy, sd > 3 cm, fusion status silent): pause, and
  give up after 5 s

**Simulator** (`phase3/tools/sim_goal_exec.py`). The rover's quirks all at once:
turns about an off-centre pivot, turns at 30-45% of the command with a lag and
a scrub deadband, the firmware's 0.01 m/s side cutoff, fusion2-level pose noise,
and a room scanned by 720 beams. 5 seeds × 3 chassis conditions × 12 random
goals:

| chassis | reached | mean error | worst |
|---|---|---|---|
| charged (P near centre) | 60/60 | 1.1-1.2 cm / 0.3-0.4° | 1.7 cm / 0.89° |
| weak pack (P at the left tyres, ~30-50 cm slide per turn) | 59/60 + 1 correct refusal (5 cm margin to a wall) | 1.1-1.2 cm / 0.3-0.5° | 1.7 cm / 1.06° |
| asymmetric L ≠ R | 58/60; 2 near-misses at 1.9-2.0 cm | 1.0-1.3 cm / 0.3-0.6° | 2.0 cm / 1.33° |

It refuses a turn with a box 3 cm beside the rover, and a leg with a box in the
path. Learned P matches the true pivot to ~1 cm.

**What the simulator caught in the first designs:**
1. Turn → drive → turn toward each correction made two ~180° turns, each
   sliding ~6 cm, to fix 5 cm. It never converged; replaced by the line
   approach.
2. The forward/reverse choice flip-flopped inside the aim iteration.
3. The cross-track correction was sign-flipped when reversing, twice.
4. The approach was too fast for the drive's lag and coasted past.
5. Unlearned on a weak pack, the first goals ran out of tries; fixed with more
   tries and the saved prior.

**Live, so far** (no motion): a goal at the current pose reports `reached 0.0 cm`;
a 2.5 m goal is refused as nav2's job. **Next: short live goals, watched, on a
charged pack.**

### M4 live, first goals on the rover (2026-09-26)

Sent with `./rover goto` (the brain's LLM was down; the same topic it will use).
Every goal was recorded by the harness, with the goal sent only after the
recorder was running. The first attempt ran each goal *after* its recording
ended, because `run.py`'s output was block-buffered when piped; `./rover
record` now runs it with `python3 -u`.

| goal | result (goal_exec's pose) | LiDAR truth |
|---|---|---|
| 1. back 40 cm | **reached 0.7 cm / 0.0°**, 9.8 s: back to the pre-goal, 25 cm straight in | moved −40.8 / −0.1 cm, +0.06°: **0.8 cm from the goal**; fused within 0.1 cm of the truth |
| 2. turn 90° left in place | **stalled**: the first turn landed at +90.0° but slid 8.8 cm; a +20° correction turn then stopped moving (stuck 12.4° short for 16 s) | to grade |
| 3. back-right 58 cm, face −90° | **failed**, a limit cycle: each line approach ended 6-7 cm off the line with ±4° heading, so it backed up and tried again, 8 times | to grade |
| 4. return to the start pose (78 cm, +17° away) | **reached 1.2 cm / 0.5°**, 21.8 s, after 8 small corrections | to grade |

**What the rover showed that the simulator did not:**

- **Small turns in place stall.** The turn slows to WZ_MIN = 0.6 rad/s
  commanded near its target. On the real rover that is below what the scrub
  lets through (0.6 × ~30-45% efficiency), so a small correction turn never
  finishes. The harness turned reliably at a constant 1.5. Fix: keep the
  command at or above ~1.2 and stop early on the measured rate, instead of
  slowing down.
- **Steering while driving corrects less than modelled.** A 25 cm approach
  did not remove 6-7 cm of cross-track error on the real floor. The simulator
  assumed an arc follows 85% of the command. Fix: measure the real arc
  response, then a longer runway or stronger cross-track gain.
- The simulator must get both measured responses before the next live round.

### The measured response (`./rover response`, 2026-09-26, charged pack, this floor)

| motion | commanded | actual | share |
|---|---|---|---|
| pivot in place | 0.6 / 0.8 / 1.0 / 1.2 / 1.5 rad/s | 0.10-0.15 / 0.20-0.25 / 0.31-0.34 / 0.42-0.45 / 0.58 | 17-39%, rising with the command: **ω ≈ 0.52·cmd − 0.21** |
| pivot slide | per 10-60° turned | 1-4 cm | (P near the centre on a charged pack) |
| arc at 0.10 m/s | wz 0.15 / 0.30 / 0.50 | 0.023-0.029 / 0.059-0.066 / 0.116-0.127 | **15-25%**; the speed tracks (0.09-0.11 m/s) |
| creep straight | 0.02 / 0.035 / 0.05 m/s | moves every time | straight driving is fine even very slow |

The simulator had assumed arcs follow 85% of the command. The real 20% is why
a line approach could not steer out 6-7 cm of cross-track error (live goal 3).
The pivot curve is why a correction turn slowed to 0.6 rad/s stalled (goal 2).

**The drive froze again** during the first full run, after ~20 s of pivoting
(ten 2 s pivots). The −1.5 pivot, the arcs and the creep after it moved little
or not at all, and a nudge right afterwards drove normally. It is the same
pattern as §8 and §9: driver protection or the pack under sustained stall
current (ROVER_BUILD_PLAN.md §4.5). The arc and creep rows above are from a
separate run on a rested drive.

### M4 live, round 2: retuned on the measured response (2026-09-26)

After `./rover response`, the simulator was given the measured model and the
executor retuned: turn floor 1.0, early stop on the measured rate, steering
gains ~4× in command units, 35 cm runway. Simulator: 104/108 reached, mean
0.9-1.2 cm. Then on the rover, each goal recorded:

| goal | goal_exec | LiDAR truth |
|---|---|---|
| forward 30 cm | reached 0.9 cm / 0.3° | after it finished, the rover crept ~11 cm over ~12 s with no command running (the owner reported wires under the wheels during the run) |
| **turn 90° in place** (stalled in round 1) | reached 0.2 cm / 0.7° | **0.8 cm / 0.96°** |
| forward-left (0.3, 0.4), face +90° | reached 1.5 cm / 0.6° | **1.25 cm / 0.8°** |
| return to the start pose | reached 0.9 cm / 0.9° | **the whole four-goal loop closes within 1.4 cm / 0.8°** |

fused (fusion2) agreed with the LiDAR truth to 0.1-0.4 cm in every run; the
wheels alone were off by up to 59 cm / 74° over the same moves. Both round 1
failure modes, the stalling small turn and the line-approach limit cycle, are
gone.

**Not yet shown:** a drive freeze mid-goal (the executor's stall stop is the
guard), goals on other floors, and approaching an object.

## 13. M5: nav2 on the fused pose, and `./rover navto` (2026-09-26)

The first nav2 drives since fusion2 took over `/odom`. `./rover navto X Y DEG
--exact` splits the work: nav2 plans and drives the route (costmaps: nvblox +
LiDAR, 5 cm padding) to the **pre-goal**, 35 cm behind the target on its
heading line, and goal_exec does the straight final approach. nav2's own goal
checker is 10 cm / 14° by design; the exact part is goal_exec's.

| run | nav2 | goal_exec | LiDAR truth |
|---|---|---|---|
| nav2 only: 0.5 m fwd, 0.2 m left | succeeded in 13.5 s, 8.6 cm from its goal (inside its 10 cm) | - | - |
| return to start, `--exact` (wires under the rover) | reached ~14 cm in 20 s, then held 10-14 cm for ~80 s before succeeding at 9.3 cm | **refused**: obstacle 18 cm into its 30 cm leg | recording has no moving checkpoints; the stall's cause is **not established** (suspect: the wires) |
| return to start, `--exact`, wires cleared | succeeded in 15.8 s, 4.9 cm from the pre-goal | **reached 1.3 cm / 0.5°** | fused within **0.1 cm / 0.1°** of truth, so ~1.3 cm real; wheels alone 13.4 cm / 18° |

So nav2 drives correctly on fusion2 (TF, costmaps, LiDAR layer), and the route
+ exact-finish split works. **Not yet shown:** gaps (fits vs refuses), a route
around an obstacle, approaching an object, a repeat of the 80 s near-goal stall.
