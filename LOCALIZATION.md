# Localization — holding x, y, θ through turns

**The goal (the owner's words, 2026-09-23):** from its current x, y, go to
x + x1, y + y1, and when asked to go back to x, y it must land in the exact
position and at the exact heading θ. In a known room or an unknown one, and
with no map saved across a power-off.

That needs the pose to stay right. It was right driving straight and wrong on
turns. This document is what was found about why, what was built, and what is
still open. Work of 2026-09-22 → 09-24; the chronological notes, with every
number, are TODO.md §42 and §43.

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
