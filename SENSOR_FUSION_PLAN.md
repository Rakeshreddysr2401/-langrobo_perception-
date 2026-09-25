# Sensor fusion plan — every sensor used for what it is good at

Written 2026-09-24 at the owner's request: *"don't blindly ignore wheel data,
don't blindly follow the existing code; where and how to use each sensor
effectively, which techniques, and a plan that will work."* Targets: home and
office now, a parking assistant later, surfaces and wheel types that change
(omni wheels are possible). Edge cases: ROVER_BUILD_PLAN.md §9.6.

---

## 1. What we have today, judged honestly

`fusion.py` is a hand-tuned complementary filter. It was validated with care,
and it has real wins (4.9 cm after 12 cuVSLAM teleports, where either input
alone was 55-80 cm off). But its structure limits it:

| today | problem |
|---|---|
| the position step is **cuVSLAM's direction × a wheel/VO scale ratio** | position depends on the camera's direction; when the camera is weak the fusion drops to dead reckoning |
| **the LiDAR is not an input**; slam_toolbox corrects `map → odom` only after **10 cm or 0.1 rad** of motion | the best indoor sensor corrects in coarse, late steps, and least of all during a pivot, which is exactly when the pose slips (~7 cm per turn, TODO §43) |
| hard switches: `turning_now`, `enc_ok`, `YAW_CORRECT_MAX_RATE` | a sensor is either fully trusted or fully ignored; it jumps at the threshold; nothing downstream knows how sure the pose is |
| the wheels are averaged per side, **front and rear ticks summed** | throws away the rover's best slip signal: the two wheels on one side must turn the same, and when they do not, something slipped |
| skid-steer turning uses one symmetric `effective_track` 0.5216 | the left side scrubs more (1.60× vs 1.29×); a symmetric model cannot represent that |
| the scan is taken as a snapshot | at 10 Hz the C1 rotates 0.1 s per scan: at ω rad/s the rover turns ω·0.1 rad *during* one scan (5.7° at 1 rad/s), smearing it exactly while turning |
| no covariance on `/odom` | nav2 and the brain cannot tell a sure pose from a guess |

So: keep what was learned (the measurements, the failure cases, the tests),
change the structure.

---

## 2. The principle

> **Each sensor reports what it measured *and how sure it is right now*.
> The fusion weighs them by that. No sensor is switched off by a rule;
> it is down-weighted by its own evidence, and rejected only by a statistical gate.**

"How sure" is a covariance, computed by the sensor's own node from its own
evidence (slip, landmarks, scan geometry, stillness). That turns "ignore the
wheels while turning" into "the wheels report a large yaw-rate uncertainty
while their four encoders disagree", which is correct in every case, not
just the ones someone thought of.

---

## 3. Each sensor: where it helps, and the technique that gets the most from it

### 3.1 Wheels (4 encoders, 20 Hz): a velocity sensor with a slip detector built in

They are not a position truth. They are a **body-velocity measurement** that
is excellent when the tyres grip and wrong when they do not, and the rover
can usually *tell which*.

| technique | what it gives | why it fits here |
|---|---|---|
| **Skid-steer ICR kinematics** (Mandow et al., 2007): three parameters, the left and right instantaneous centres of rotation `y_L, y_R` and `x_ICR`, instead of one "effective track" | correct vx and ω from wheel speeds, **asymmetric**, so the weak left side is modelled, not averaged away | 0.5216 is the symmetric special case; the measured left/right difference says we need the general one |
| **Online calibration of those parameters** against the LiDAR and gyro, only while they are healthy and the tyres are not slipping | parameters that follow the surface (tile, carpet, road) and the wheels (worn, swapped) | "calibrate once with a tape" does not survive a new floor |
| **Per-wheel slip evidence**: front vs rear on the same side (bolted to one chassis and one driver, so without slip they must agree); wheel ω vs gyro ω; wheel v vs LiDAR v | a slip score per side, 0 to 1, every message | straight: front/rear agree to 1.00-1.03×; pivot: 1.29-1.60× — the signal is already in the data we collect |
| **Covariance from slip**, not a switch: σ grows with the slip score and with |ω| | wheels still help mid-turn when they grip, and fade smoothly when they do not | the owner's point: do not ignore them blindly |
| **Zero-velocity lock** (ZUPT): all four encoders still and no command → "stationary" with tiny covariance | freezes drift while parked; the moment to re-measure gyro bias | wheels are the *only* sensor that knows this for certain when people walk past (LiDAR and camera see their motion) |
| **Along-corridor distance**: where the LiDAR is degenerate (3.2), the wheels carry that one axis | distance down a plain corridor | exactly where LiDAR scan matching fails |
| **Stuck detection**: wheels moving, LiDAR + gyro + camera say still | stop pushing, report "stuck" | a disagreement is information, not noise |
| **Command vs response** (and motor current once the INA226 is in, §4 of the build plan) | stall and low-battery detection | the pivot works near stall (§0) |

For **omni/mecanum wheels** the kinematic model changes (`drive: skid | mecanum`
in `description/params.yaml`) and lateral velocity becomes observable; the
slip score and online calibration work the same way.

### 3.2 LiDAR (RPLidar C1, 10 Hz, 720 beams): the position anchor indoors

| technique | what it gives |
|---|---|
| **De-skew each scan** with the gyro (200 Hz): place every beam at the pose the rover had *when that beam fired* | clean scans while turning; the time-stamp fix (377eed6) was step one, this is step two |
| **Scan-to-submap matching every scan** (point-to-line ICP, Censi 2008), not only scan-to-previous-scan | LiDAR odometry at 10 Hz with little drift, independent of the wheels and the floor. slam_toolbox stays as the **global** layer (loop closure, `map → odom`) |
| **Robust cost** (Huber/Cauchy) and **dynamic-point rejection** (points that stop matching between submap updates) | people and moving chairs do not drag the pose |
| **Degeneracy from the ICP Hessian** (Zhang, Kaess, Singh 2016): its eigenvectors say which directions the scan constrains | in a corridor the along-axis covariance becomes huge, so the fusion takes that axis from the wheels and camera and the rest from the LiDAR |
| **Tilt gate**: roll/pitch from the accelerometer; above ~3° the laser plane sweeps the floor, so drop or down-weight that scan | no phantom walls on rugs, thresholds, ramps |
| **Health**: number of matched points, residual, fraction in range | "nothing within 12 m" (open car park) is reported, not guessed |

### 3.3 Gyro (D555 IMU, 200 Hz): the heading backbone

| technique | what it gives |
|---|---|
| **Bias re-estimated at every zero-velocity lock** (wheels say still) | already done; now triggered by the certain signal (3.1) |
| **Scale-factor calibration** against LiDAR-measured 360° spins | the −0.2 to −0.9% turn error is consistent with a scale error; the LiDAR can measure it automatically |
| **High-rate propagation**: the filter predicts with the gyro, and every other sensor corrects | smooth heading between 10 Hz scans, and the de-skew input |

### 3.4 Camera (D555): position where the LiDAR is weak, obstacles everywhere

| technique | what it gives |
|---|---|
| **cuVSLAM covariance from landmark count and tracking state** | trusted in texture-rich scenes, faded in blank ones, gated on teleports (the existing chi-style checks become a Mahalanobis gate) |
| **Test cuVSLAM's visual-inertial mode** (IMU fed in) | may fix its rotation weakness (+11% on a 90°). Open question, decided by the harness (§5), not assumed |
| **Outdoors / open space**: VO is the primary position source there (the C1 is rated 40k lux, has 12 m range) | the parking-assistant case |
| **Depth for obstacles** (nvblox), separate from localization; **tilt the camera down** to shrink the near blind zone | ROVER_BUILD_PLAN.md §9.6 |

### 3.5 Things no current sensor covers
Cliffs and low, close objects (under the 25 cm laser plane, inside ~40 cm of
the nose): downward ToF sensors and a firmware bumper (§9.6). Not a fusion
problem; listed so it is not forgotten.

---

## 4. The fusion engine: decided by measurement, not by habit

Two candidates, both fed by the same per-sensor nodes from §3:

| | **robot_localization EKF** (installed) | **extend `fusion.py`** |
|---|---|---|
| weighting | by the covariance on each message — exactly §2 | hand logic unless rewritten as a filter |
| outlier gate | Mahalanobis per input (`*_rejection_threshold`) | custom checks |
| lagged data | `smooth_lagged_data` + history | none |
| our special knowledge | lives in the sensor nodes (slip, ICR, degeneracy), not in the filter | mixed into the filter |
| risk | generic tuning; must be graded | a proven baseline, but structurally limited (§1) |

**Plan:** build the sensor nodes first (they are needed either way), then run
**both** engines on the **same recorded data**, graded by the same harness.
Keep `fusion.py` as the baseline until the new one beats it on every
scenario. If both fall short on delayed LiDAR data, the next step is a small
factor graph (the LiDAR's lag is exactly what graphs handle well); that is
only built if the numbers ask for it.

Layers after the change:

```
sensors ─► per-sensor nodes (measurement + covariance + health)
            wheels_odom  lidar_odom  gyro(+ZUPT)  vo (+cov)
                     │
                     ▼
            local fusion  ──► odom → base_link, /odom with covariance   (smooth, never jumps)
                     │
            slam_toolbox  ──► map → odom                                (global, corrects drift)
```

---

## 5. The test harness: how we know it works

Nothing here is decided by argument. It is decided by recorded runs.

1. **Record once, replay many.** `ros2 bag record` of the raw inputs
   (`/scan`, `/gyro/base`, `/wheel_ticks`, `/wheel_state`, `/vo/odom`,
   `/vo/status`, `/cmd_vel`, `/tf_static`). Every fusion variant is then
   compared offline on the same data.
2. **Ground truth**:
   - tape: start and end marks (the X on the floor), measured to ±5 mm;
   - continuous: an **offline** full-batch slam_toolbox solve with loop
     closures over the whole bag, which is better than anything online.
     Used only as truth, never as an input to the variant being graded.
3. **The scenario suite** (from §9.6): straight 2 m and back; 90° and 360°
   pivots each way; square; plain corridor; people walking past a parked
   rover; rug edge / threshold; dark room; glass door; rover lifted and set
   down; wheels spinning while held.
4. **Metrics**: end-point error (cm), heading error (°), pivot-centre drift
   (cm), pose jumps (count), time to flag slip / stuck / kidnap (s), CPU on
   the Orin.
5. **Acceptance** (home/office):
   - end point ≤ 3 cm on a 2 m out-and-back; ≤ 5 cm after a 4 × 90° square
   - heading ≤ 1° after 360°
   - pivot centre held to ≤ 2 cm (with the closed-loop pivot, stage E)
   - never worse than the current `fusion.py` on any scenario
   - no pose jump > 2 cm in `odom`
   - stuck and kidnap flagged within 1 s

---

## 6. The plan, in stages

| stage | what | done when |
|---|---|---|
| **A. Harness** | bag recording command, scenario checklist, grading script (metrics above), baseline numbers for today's `fusion.py` | the current fusion has a score on every scenario — **tools built and truth validated 2026-09-24 (`phase1/harness/`); baseline session pending** |
| **B. Wheel node** | ICR skid-steer kinematics, per-wheel slip score, covariance from slip and ω, zero-velocity lock, stuck flag; ICR params fitted from bags, then online | wheel-only odometry beats today's wheel numbers in turns; slip flagged in the "wheels spinning" run |
| **C. LiDAR odometry** | gyro de-skew, scan-to-submap PL-ICP, robust kernel, Hessian degeneracy → covariance, tilt gate | LiDAR-only odometry graded; corridor run shows the degeneracy flag |
| **D. Fusion** | robot_localization with B + C + gyro + VO(cov); A/B against `fusion.py` on the bags; keep the winner | meets §5 acceptance on the recorded suite, then live |
| **E. Closed-loop pivot** | hold base_link's x,y from the fused pose while turning | centre ≤ 2 cm over 360°, both ways |
| **F. Calibrate** | `./rover calibrate`: drives a pattern, fits ICR + gyro scale + per-side minimum duty per surface, stored by name | re-run on carpet recovers the tile numbers' accuracy |
| **G. Outdoors** | shade vs sun, VO as primary in open space, GPS if the parking case continues | measured, then decide on hardware |

Stages A-D need nothing bought and no hardware change. A comes first because
without it every later "improvement" is an opinion.

---

## 7. What stays, what changes

- **Stays:** the gyro as the heading backbone; teleport protection (becomes a
  Mahalanobis gate); tilt from the accelerometer; slam_toolbox as the global
  layer, with fresh maps at every start; every measurement in LOCALIZATION.md
  and TODO §13/§43, which become the harness's first test data.
- **Changes:** hard switches → covariances; per-side sums → per-wheel slip
  evidence; symmetric track → ICR model; the LiDAR becomes a first-class,
  10 Hz input; `/odom` carries covariance.
- **Stays until beaten:** `fusion.py` runs as the baseline until the
  replacement wins on the recorded suite.

---

## 8. Movement roadmap: what joins with what, in order (2026-09-25)

Movement first, the arm later. Updated with the baseline (LOCALIZATION.md §8)
and the measured gaps (LOCALIZATION_GAPS.md). Four layers, each built on the
one below.

### The joins: each pairing gives something neither sensor has alone

| join | what it gives |
|---|---|
| gyro + LiDAR | de-skewed scans while turning; the gyro's scale checked against the walls (1.001 over 21 turns: fine); heading that is fast (gyro) *and* absolute (LiDAR) |
| LiDAR + wheels | slip and stuck detection (wheels say moving, LiDAR says not); the asymmetric ICR skid-steer model fitted per surface; distance along a corridor where the LiDAR is blind to it |
| LiDAR + camera | one extrinsic for the camera, measured (G3); depth time offset measured against the scan (G8); VO carries position where the LiDAR is degenerate; the costmaps take the union of both |
| wheels + IMU | zero-velocity lock: certain stillness, and gyro bias re-measured every stop |
| battery voltage + motors (INA226) | pivot torque margin known before a turn; speed scheduled to the pack |
| everything + the harness | every change graded against LiDAR truth before it goes live |

### The layers, in build order

| phase | what | acceptance (graded by the harness) |
|---|---|---|
| **M1 Calibrate** | camera ↔ LiDAR from 4-6 poses (one camera yaw everywhere); gyro scale from 3 × `pivot360` each way; the item on the rover resolved | depth-to-LiDAR residual ≤ 1 cm and ≤ 0.3°; gyro ≤ 0.3° per 360° |
| **M2 LiDAR odometry** | scan-to-submap PL-ICP at 10 Hz, gyro de-skew, Huber kernel, degeneracy → covariance; offline on the bags first, then live — **offline done 2026-09-26: worst 0.9 cm / 0.39° over 17 runs (LOCALIZATION.md §10); live node next** | ≤ 2 cm and ≤ 0.5° on every scenario, including a new random `zigzag`; no drift while parked |
| **M3 Fusion** | gyro predicts; LiDAR, VO (covariance from landmarks) and wheels (ICR model, slip score, zero-velocity lock) correct, weighted by covariance; robot_localization EKF vs fusion.py decided by the harness; `/odom` with covariance; health flags (degenerate, slip, stuck, kidnap). With odom no longer drifting, nvblox's objects stay where they were seen | never worse than LiDAR-only; corridor, lift and slip runs flagged correctly |
| **M4 Motion control** | primitives closed on the fused pose instead of timed: centre-holding pivot, straight with heading hold, exact short moves. Firmware: anti-windup on the per-side PI (the right side ran backwards under a forward command), stamped ticks, INA226 current and voltage | pivot centre ±2 cm over 360°; 1 m straight ±1 cm; 5 cm / 5° moves ±5 mm / ±0.5° |
| **M5 Safe navigation** | nav2 on M3's pose: rotation shim for in-place turns, **collision monitor** on the raw LiDAR and depth as a last line independent of the planner, speed scaled by pose confidence, reversing only into space already seen; downward ToF cliff sensors and a bumper | the critical-path suite: doorways, tight corners, clutter, people |

### Kept cheap now so the arm fits later

- **Keep the laser plane clear.** Anything on the rover above 22 cm blinds the
  LiDAR: the arm stows below it, or the LiDAR moves above the arm.
- **The arm goes into `description/`**, so its links are in TF and can be masked
  from the scan and the depth.
- **M4's centre-holding pivot and exact short moves are the base's half of a pick.**
  The base gets within ±2 cm; a wrist camera and the arm's reach do the last centimetre.
- **Stop-and-grasp** relies on M3's zero-velocity lock: a base that is certainly still.
