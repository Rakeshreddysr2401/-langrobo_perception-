# Task 03 — fusion: surviving the moments cuVSLAM cannot see

**Needs:** task 02 passed (the pose is metrically honest).
**Moves the robot:** no — hand push, plus a pivot on the spot.
**Command:** `./rover.sh l3`

---

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
```

The z check is the one people skip. **If z is not 0.000, the EKF is not running**
and you are about to map with the pose that smeared the map last time.

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
