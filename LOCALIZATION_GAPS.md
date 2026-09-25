# Localization gaps — measured on the running rover, 2026-09-25

**The owner's requirement:** correct x, y, θ at all times, through any
zig-zag. What the camera sees at (x1, y1) from pose (8, 8, 90°) must still be
at (x1, y1) when the rover comes back. Safe motion in tight paths comes after
this.

Every gap below is measured, not argued. Tools: the stage A harness
(`phase1/harness/`), a stamp-age probe, and a camera-vs-LiDAR match.

---

## What the requirement actually needs

"Always correct x, y, θ" **and** "depth objects land in the same place" need
three things to hold at once:

1. **An odometry frame that does not drift.** Everything that remembers
   obstacles (nvblox, both costmaps) stores them in `odom`.
2. **Sensors that agree on geometry.** The camera and the LiDAR have to put
   the same wall in the same place: extrinsics.
3. **Sensors that agree on time.** Each measurement has to be placed at the
   pose the rover had *when it was taken*: stamps and latency.

Today none of the three is fully true.

---

## The gaps, most damaging first

| # | gap | evidence (measured) | effect on "object at (x1, y1)" |
|---|---|---|---|
| **G1** | **Obstacles are stored in a drifting frame.** nvblox and both costmaps integrate in `odom`, which is dead reckoning. slam_toolbox corrects `map → odom`, but only after 10 cm or 0.1 rad of travel (`slam_toolbox.yaml`), and nothing already stored moves with the correction. | Baseline (LOCALIZATION.md §8): 5-19 cm of odom error after turning sequences, before the lever-arm fix | Every cm of odom drift is a cm of error in every remembered obstacle. **The root gap.** |
| **G2** | **The LiDAR is not an odometry input.** It only corrects through SLAM, coarsely and late. | fusion.py takes VO, gyro, wheels; no scan input | The best indoor sensor does not hold the pose between SLAM updates |
| **G3** | **Camera and LiDAR disagree on the camera's yaw.** The URDF says 0°, VO applies 2.06° (measured in August, older mount), and the LiDAR says **+1.09°**. | Depth at 22-28 cm matched to the scan: dx +1.4, dy −1.5 cm, **dyaw +1.09°**, residual 1.2 cm | 1.1° puts an object at 3 m **~6 cm** sideways. Two different yaws for one camera means VO and depth disagree with each other too. |
| **G4** | **Scans are not de-skewed.** A C1 scan takes 0.1 s, so turning at ω smears it by ω·0.1 rad. | 5.7° across one scan at 1 rad/s | Scan matching is weakest exactly during turns |
| ~~G5~~ | ~~The gyro under-reads by ~2%.~~ **Withdrawn 2026-09-26.** Measured over 21 turns against LiDAR truth, the gyro scale is **1.0010** (per turn ±0.7%, 1.1° rms). The "2%" was one turn's disagreement plus a mid-motion trace read. | M1 gyro-scale runs, `pivot360` × 3 each way | None. Turns do overshoot 1-3°, but from the runner integrating on arrival time and the rover coasting after the stop: an M4 motion-control item |
| **G6** | **Wheel ticks carry no timestamp.** They are stamped on arrival, after the ESP32 → agent → network path. | `/wheel_ticks` is a bare `Quaternion`; 20 Hz, with 326 ms worst gap seen | Unknown delay and jitter on every wheel velocity; worst during starts and stops |
| **G7** | **VO drops about 1 frame in 5.** | IR in at 26 Hz, `/vo/odom` out at 21.5 Hz | Less VO in fast turns, where it is already weakest |
| **G8** | **Depth time offset: unmeasured.** Depth arrives 35 ms after its stamp (p95 55 ms). Whether the stamp is the exposure time is not known. | stamp-age probe | If the stamp is late by Δt, turning at ω misplaces objects by ω·Δt: 30 ms at 1 rad/s is 1.7°, **9 cm at 3 m** |
| **G9** | **The Pi 5 clock is ≥ 30 ms ahead of the Jetson's**, and there is no chrony status. | `date` on both, back to back | Matters for anything stamped on the Pi (the brain's commands, maybe the micro-ROS agent) |
| **G10** | **No uncertainty on `/odom`.** | message covariance is unset | nav2 and the brain cannot tell a sure pose from a guess |
| **G11** | **Something on the rover blocks a LiDAR sector** (front-right now, rear-right earlier), at base_link (+0.19, −0.12). | stays fixed in base_link when the rover moves | A blind wedge, and a constant point pulling every scan match toward "no motion" |

| **G12** | **The gyro lives inside the camera.** `/gyro/base` is the D555's IMU, delivered over the same PoE/DDS link as depth and IR. The D555 goes "half-alive" (colour streams, depth and IR stop, `timeout waiting for reply`), seen again 2026-09-26. | fusion2 publishes the pose from the gyro callback, so if the IMU stream stops, **the pose stops**, and today the LiDAR odometry's de-skew also needs the gyro | One fragile link carries three of the four motion sensors (IR for VO, depth, gyro). An independent IMU on the ESP32 (or the Jetson) removes the single point of failure; until then, fusion2 must keep publishing on LiDAR + wheels if the gyro goes silent |

Measured and fine: `/scan` arrives 19 ms after its (corrected) stamp; depth,
gyro and `/odom` rates are nominal; the camera lever arm is fixed (8.0 → 2.0 cm
mean error, LOCALIZATION.md §8); the floor tilt is in the TF.

---

## Are we using the right inputs? And can the LiDAR do more?

| job | today | should be |
|---|---|---|
| heading, fast | gyro (scale checked against the LiDAR: 1.001, 2026-09-26) | gyro, bias re-measured at every stop |
| heading, absolute | VO when not turning; SLAM, coarsely | **LiDAR scan-to-map, every scan** |
| position | VO direction × wheel scale; SLAM, coarsely | **LiDAR scan-to-map, every scan**; VO where the LiDAR is degenerate; wheels as fallback and slip detector |
| where depth puts objects | TF with the camera yaw at 0°, stored in drifting `odom` | a **LiDAR-calibrated** camera extrinsic, stored in a frame that does not drift |
| calibration | tape, and one-off scripts | **the LiDAR as the ruler** for everything: gyro scale, wheel ICR, camera extrinsic, depth time offset |

**Yes, the LiDAR can do far more than it does.** It is the one sensor here
that measures absolute position against the room at 10 Hz with ~1 cm
precision (hundreds of ±30 mm points per match), independent of the floor,
the wheels and lighting. Used as an odometry source (SENSOR_FUSION_PLAN.md
stage C), `odom` stops drifting in any room with structure. That fixes G1, the
root gap: once `odom` holds, nvblox's objects stay where they were seen, and
the (8, 8, 90°) → (x1, y1) consistency follows.

---

## Order of work

| step | fixes | how | needs the rover to move? |
|---|---|---|---|
| 1. **Joint camera ↔ LiDAR calibration** | G3 | depth slice vs scan, from 4-6 poses; write the camera yaw (and an x, y cross-check) into params.yaml; VO takes its yaw from TF instead of its own 2.06° | a few repositions |
| 2. **Gyro scale** | G5 | 3 × `pivot360` each way, graded; apply the scale in gyro_node | yes, spins |
| 3. **LiDAR odometry** (stage C) | G1, G2, G4 | gyro-de-skewed scan-to-submap PL-ICP at 10 Hz, with covariance, into fusion; `odom` stops drifting | recorded bags first, then live |
| 4. **Depth time offset** | G8 | rotate slowly past a wall; fit the depth stamp offset against the LiDAR (like `./rover lidar --lag`) | yes, turns |
| 5. **Wheel ticks stamped at the source** | G6 | firmware publishes ticks with a header from micro-ROS time sync | a flash |
| 6. **Clock sync** | G9 | chrony between the Pi 5 and the Jetson, with the Jetson as server | no |
| 7. **Covariance on /odom** | G10 | comes with stage D | — |
| 8. **The item on the rover** | G11 | remove it, lower it below 22 cm, or add it to params.yaml | owner |

Steps 1 and 3 do the most for the requirement. Step 3 is the foundation, and
step 1 makes the camera agree with it.
