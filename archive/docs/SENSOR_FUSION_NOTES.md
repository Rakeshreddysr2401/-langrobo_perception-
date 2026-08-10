# How the robot knows where it is — sensor issues, in plain language

*Written 2026-07-18 for later study. No prior robotics knowledge assumed. This
explains why the robot kept "getting lost" during turns, and the fix we put in.*

---

## The one-sentence version

The robot figures out where it is by watching the world move past its camera.
That works great when driving straight, but the camera **can't tell it's turning**
on our shiny floor — so we added the camera's **motion sensor (gyroscope)** to
handle turning, and a small program (an "EKF") to blend the two.

---

## 1. Two ways a robot can measure its own motion

- **By the wheels** ("how many times did the wheel spin?"). As of 2026-08 the rover
  HAS wheel encoders (Rhino GB37 + firmware v2 — see
  `orin-nav-stack/firmware/HARDWARE.md`), so wheel odometry is now available as a
  fusion source. It stays *secondary* because wheels *skid*, which lies, while the
  camera measures real motion.
- **By the camera** ("how did the view shift?") — called *visual odometry*. This
  is what we use. Its big advantage: skidding doesn't fool it, because it measures
  the *real* movement of the world, not wheel spin.

## 2. The problem we hit: the camera is blind to turning

The floor is **glossy marble**. When the robot spins in place, the camera sees a
smooth, reflective, low-detail surface sweep past too fast to track. Result, measured
live: we spun the robot a **full 360°**, and the camera only reported **48°**. For
*driving straight* the camera is fine (we calibrated it to ~±5%); for *turning* it's
almost useless.

This matters because the navigation brain (Nav2) steers using this heading. If the
robot thinks it turned 48° when it really turned 360°, it will spin wildly trying to
"finish" a turn it already overshot. Straight-line driving works; **turns break.**

## 3. The fix: use the gyroscope for turning

The depth camera (D555) contains an **IMU** — a little motion-sensor chip with a
**gyroscope** that measures how fast the robot is rotating, 200 times a second. A
gyro is *excellent* at rotation (that's literally its job) and terrible at position.
The camera is the opposite. So the plan writes itself:

> **Position from the camera. Turning from the gyro.**

We proved the gyro works: in our test tool (`scripts/drive_test.py rotate`), turning
90° with the gyro was accurate where the camera was hopeless.

## 4. Three gotchas we found along the way (the "issues")

**(a) You can't get position from the gyro.** A gyro measures rotation, and an
accelerometer measures shaking — to get *position* from an accelerometer you have to
add it up twice, and tiny errors blow up to meters within seconds. So the gyro is
*only* for turning. Position still comes from the camera.

**(b) The camera's clock and its IMU's clock don't line up.** The IMU's timestamps
run ~130–250 ms behind the image timestamps. Our first attempt fed the gyro *directly*
into the camera-tracking program, which demands the two line up tightly — it choked
and **stopped tracking entirely**. Lesson: don't tightly couple them. The EKF (below)
is "loosely coupled" — it accepts each sensor on its own clock, so the mismatch is fine.

**(c) The gyro reads a bit low because the camera is heavy and tilts the mount.**
The camera hangs off the front, so its mount sits at a slight angle — which means the
gyro's "straight up" axis isn't perfectly vertical, so it reads about **28% low** on
turns. We patch this two ways: a calibration number in the test tool (`ROT_CAL=1.30`),
and — more importantly — the map itself corrects long-term heading (see §6). The
*clean* fix, for later, is to measure the real mount angle and put it in the robot's
config so no patch is needed.

## 5. What an "EKF" is and what we added

An **EKF** (Extended Kalman Filter) is a standard program that **blends several
noisy sensors into one best-guess** of where you are and which way you face. We use
the `robot_localization` package's EKF. We told it:

- Take **position (x, y)** from the camera (`/odom`) — trust it for moving.
- Take **turning (yaw rate)** from the gyro — trust it for facing.
- Ignore the parts each sensor is bad at.

It outputs one clean pose that the navigation brain reads. Concretely:

```
BEFORE:  camera-tracking ──(where + facing, facing is blind)──► Nav2
AFTER:   camera-tracking ──(where)──►┐
                                      EKF ──(where + facing, gyro facing)──► Nav2
         camera's gyro ────(facing)─►┘
```

Files: `config/ekf.yaml` (the settings + why), and the launch wiring in
`scripts/run_rtabmap.sh` (the camera-tracker now hands the transform to the EKF).

**The gotcha that made it work (2026-07-18):** at first the EKF *ignored the gyro
completely* — heading stayed frozen at 0. The cause: the IMU reports its spin in
the **camera's tilted "optical" frame**, where a real *turn* shows up on a
sideways axis, and the filter wasn't rotating it into the robot's frame. Fix: a
tiny relay, `scripts/imu_to_base.py`, that **rotates the gyro into the robot
frame** (so "turning" lands on the right axis) and **re-stamps it to the host
clock** (removing the ~200ms lag) before the EKF sees it, on `/imu/base`.
Verified live: heading now tracks real turns to ~±10%.

## 6. Why small gyro errors don't ruin navigation

On top of moment-to-moment tracking, **RTAB-Map** builds a *map* and recognizes
places it has seen before ("loop closure"). When it recognizes a spot, it snaps the
robot's position/heading back to truth. So: the gyro gives good *short-term* turning,
and the map fixes any slow *long-term* drift. Together that's plenty for reliable
navigation, even with the 28% gyro quirk.

## 7. What's done, what's left

- ✅ EKF installed and fusing; transform chain healthy (`map→odom→base_link` intact).
- ✅ **Turning tracks** — verified live: real ~88–90° turns read ~79–87° in the EKF
  heading (was frozen at 0 before the `/imu/base` relay fix).
- ⏳ Re-test a Nav2 goal that requires a turn (the real payoff).
- 🔧 Optional: the ~8° left/right gap is the front camera weight (CoG off the turn
  axis) — reduce by counterweighting/centering the camera.

## 8. If it ever misbehaves — how to undo this

Everything is in git. To go back to camera-only (no EKF): in `scripts/run_rtabmap.sh`
set the camera tracker back to `publish_tf:=true` and remove the EKF launch block,
then re-run `scripts/run_localization.sh`. The map and navigation are untouched by
this change — only *who reports the heading* changed.
