# Task 03 — fusion: surviving the moments cuVSLAM cannot see

**Needs:** task 02 passed (the pose is metrically honest), **and the Pi 5's
micro-ROS agent running** — see the prerequisite below.
**Moves the robot:** no — hand push, plus a pivot on the spot.
**Command:** `./rover.sh l3`

---

## Prerequisite: fix the wheels FIRST — and it is not in this repo

`/wheel_state` reads a rock-steady 1.00 Hz, which per `TODO.md §2` means the
ESP32 is **not connected at all** (the firmware publishes unconditionally at
20 Hz once connected; 1.00 Hz is the `WAITING_AGENT` ping). The likely cause is
`TODO.md §11`: neither the micro-ROS agent nor the wheel-odom relay has a systemd
unit on the Pi 5, so a reboot leaves the ESP32 with nothing to connect to.

**Do that check before starting this task**, on the Pi 5, in `pi5_ros2_ws` — not
here. **Do not reflash the ESP32**; that was the 2026-08-08 fix and it is done.

Why it is worth crossing a machine boundary for: wheel `vx` is the **only
independent contradiction of cuVSLAM in the whole stack**. The gyro cannot
contradict it — it measures rotation, not travel. So without wheels, nothing in
the system can tell you cuVSLAM has frozen while still reporting
`slam_pose_ok: true` (`FACTS.md §3`), which is precisely the failure mode task 04
is about to walk into.

If it genuinely cannot be fixed today, the task still runs on two sensors and the
gate below still means something — but write down that you ran it degraded, and
expect task 04 to be harder to diagnose.

---

## Learn first

Read [`03-sensor-fusion-ekf.md`](../knowledge/03-sensor-fusion-ekf.md) then [`01-frames-and-tf.md`](../knowledge/01-frames-and-tf.md) — what a Kalman filter is actually doing, how to read the boolean grid in `ekf.yaml`, and why only ONE node may publish `odom -> base_link`.

Not to memorise. Just so the words in this task mean something.

## What you are proving

That the pose stays sane through the moments vision fails: fast pivots, blank
walls, a hand briefly over the lens.

## What to understand first

Three sensors, three different kinds of wrong:

| Sensor | Gives | Fails when |
|---|---|---|
| cuVSLAM | absolute x, y, yaw | fast pivots, blank walls, low texture |
| Gyro | yaw **rate**, ~74 Hz | drifts if integrated alone |
| Encoders | forward speed | wheels slip |

An EKF fuses them by tracking not just a best estimate but **how uncertain it
is** about each part. Each sensor's contribution is weighted by its trust. So the
gyro carries you through a pivot cuVSLAM cannot handle, and cuVSLAM stops the
gyro drifting away over minutes.

Three deliberate choices in `config/ekf.yaml`, each of which is a lesson:

**The accelerometer is not fused at all.** Tempting — it measures acceleration,
integrate twice for position. In practice a MEMS accel diverges by hundreds of
metres: gravity is ~1000× the signal you want, and any tilt error leaks gravity
into horizontal acceleration. A slow ground robot gains nothing and inherits all
the drift.

**Wheels give speed only, never heading.** In a pivot the wheels turn opposite
ways and slip; the gyro is simply better at rotation.

**`two_d_mode: true`** pins z, roll and pitch to zero. This is not cosmetic. Run
visual-only and cuVSLAM reported **z = −0.272 m** — the rover believed it had
sunk 27 cm through the floor. nvblox slices obstacles in a *fixed* height band,
so a sinking pose drags that band through the floor and maps floor as wall.

## Do this

```bash
./rover.sh l3
```

Note what it does first: it **restarts cuVSLAM with `publish_odom_tf:=false`**.
Until now cuVSLAM published `odom → base_link`; from here the EKF owns it. Two
nodes publishing the same transform fight, and RViz shows the pose flickering
between two answers.

## Gate

```
[ ] /odometry/filtered publishes at >= 10 Hz
[ ] /imu/base publishes at >= 50 Hz
[ ] /odometry/filtered z is EXACTLY 0.0
[ ] pivot the rover 90 deg on the spot by hand -> heading follows, does not jump
[ ] cover the lens for ~2 s -> the pose does not explode; it recovers
[ ] rotate a FULL 360 deg slowly by hand, CLOCKWISE -> heading returns to its
    start within 5 deg, and x,y have not wandered more than ~0.10 m
[ ] repeat the full 360 deg ANTICLOCKWISE -> same bounds
[ ] write down the SIGN of both residuals — see below, this is the real result
```

The z check is the one people skip. **If z is not 0.000, the EKF is not running**
and you are about to map with the pose that smeared the map last time.

### Why the 360° check is here, and why it matters more than it looks

It would be easy to treat this as pedantry. It is not.

The camera sees **87° forwards** and there is no working pan/tilt head. So in
task 08 — autonomous exploration — the rover fills in its surroundings by
**stopping and spinning in place**, sweeping that cone around a full circle,
before choosing where to go next. Rotation is therefore not incidental to the
robot's autonomy; it *is* the mechanism.

And rotation is precisely what visual odometry handles worst. In a spin,
features leave the frame within a few frames and there is little parallax to
work with, so pure vision degrades exactly when you need it. That is what the
gyro is there for.

So this check is really asking: **does the gyro genuinely carry the pose through
a rotation?** If the heading is off by 30° after one turn, task 08 cannot work —
the rover will spin, believe it is facing somewhere it is not, and write the
whole scan into the map at the wrong angle.

Find that out here, by hand, at zero speed. Not in task 08 with the robot
driving itself.

### Where 5° comes from, and why you spin BOTH ways

**The 5°** is derived from the map, not from the sensor. nvblox voxels are 5 cm,
so at 3 m range one degree of heading error displaces a wall by about 5 cm —
roughly one voxel. 5° is therefore ~26 cm of smear at typical room range, which is
already at the edge of task 04's "lines, not blobs". Anything looser and the spin
in task 08 would write its scan into the map crooked enough to make the map worse
the more the rover explores.

**Both directions** is the part that turns a number into a diagnosis. Two turns,
two residuals, and it is the *sign* that tells you which problem you have:

| Clockwise | Anticlockwise | What it means |
|---|---|---|
| +4° | −4° | **Scale bias** — the gyro or the yaw scaling is off by a constant. Correctable in one number, and task 08 is still viable. |
| +4° | +4° | **True drift** — error accumulates regardless of direction. This does not cancel; ten spins in one exploration run is 40°, and task 08 as designed is in trouble (`TODO.md §9`). |
| ±1° | ±1° | Fine. Record it in `FACTS.md` and move on. |

Run each turn slowly and record both numbers even when they pass. This is the
first measurement of rotation this rig has ever had, so the numbers are worth more
than the pass/fail.

## If it fails

| Symptom | Meaning |
|---|---|
| `/odometry/filtered` missing | EKF did not start — `./rover.sh logs ekf` |
| z drifts from 0.000 | you are on raw cuVSLAM, not the EKF |
| yaw ignores a real pivot | the gyro is not reaching the EKF; check `/imu/base` |
| pose jumps when TF is fine | two publishers of `odom → base_link` |

⚠️ `/wheel_odom` is currently **dead** (`TODO.md §2`), so the EKF is running on
two sensors, not three. The gate above is still meaningful, but the independent
speed cross-check is missing — which matters for task 04.

## Commit

```bash
git add -A && git commit -m "task 03: EKF 20 Hz, gyro 74 Hz, z=0.000, survives 2 s blind"
```

**Next:** [`04-mapping.md`](04-mapping.md)

---

## What I learned doing it

*Fill this in while you still remember being confused — that is the valuable
part, and it evaporates within a day.*

**What surprised me**

>

**What I got wrong first**

>

**Numbers I measured**

>

**Codebase things worth remembering** — a file, a parameter, a line that turned
out to matter more than it looked

>

**Still don't understand**

>

> Housekeeping when you finish: a measured number belongs in `FACTS.md`, a
> broken or unverified thing belongs in `TODO.md`, and anything you learned about
> the *concept* rather than about today should be promoted into
> [`knowledge/`](../knowledge/).
