# Sensor fusion, and why this rover has no EKF

**Used by:** Phase 1 (pose).
**Code:** `phase1/nodes/fusion.py`, `phase1/nodes/gyro_node.py`  ·  **What we built instead of an EKF:** [ARCHITECTURE.md](../ARCHITECTURE.md) §5

---

## Why fuse at all

Three sensors, three different ways of being wrong:

| Sensor | Gives | Good at | Fails when |
|---|---|---|---|
| cuVSLAM | absolute x, y, yaw | long-term accuracy, no drift when it closes loops | fast pivots, blank walls, low texture |
| Gyro | yaw **rate**, ~74 Hz | fast rotation, never loses track | drifts if integrated alone |
| Encoders | forward speed | knowing you moved at all | wheels slip |

Notice they fail in **different situations**. That is the whole point. The gyro
is excellent exactly where cuVSLAM is weakest (rotation), and cuVSLAM stops the
gyro drifting away over minutes.

Fusing is not averaging. It is combining estimates weighted by how much you trust
each one *right now*.

---

## What a Kalman filter actually does

Two steps, repeating forever:

**Predict.** Using the motion model, guess where the robot is now based on where
it was. Uncertainty **grows** — you are extrapolating.

**Update.** A measurement arrives. Combine prediction and measurement, weighted
by their relative uncertainty. Uncertainty **shrinks** — you learned something.

The key idea is that the filter tracks not just a best guess but **how confident
it is**, as a covariance matrix. A measurement it trusts pulls the estimate hard;
one it does not barely moves it.

The "Extended" part just means the maths is linearised so it works with rotations,
which are not linear.

**The practical consequence:** covariances matter as much as measurements. A
sensor that reports over-confident covariance will dominate and drag the estimate
wrong, and nothing will look broken.

---

## Reading `phase1/nodes/fusion.py`

The config is a grid of booleans. Each sensor's `_config` is 15 flags in this
order:

```
x      y      z
roll   pitch  yaw
vx     vy     vz
vroll  vpitch vyaw
ax     ay     az
```

`true` means "take this from this sensor". Ours:

| Source | Topic | Taken |
|---|---|---|
| `odom0` | `/odom` (cuVSLAM) | **x, y, yaw** |
| `odom1` | `/wheel_odom` (encoders) | **vx** only (index 6) |
| `imu0` | `/imu/base` (gyro) | **vyaw** only (index 11) |

### Three deliberate choices, each a lesson

**The accelerometer is not fused at all.** Tempting — it measures acceleration,
integrate twice for position. In practice a MEMS accelerometer diverges by
hundreds of metres. Gravity is ~1000× the signal you want, so a tiny tilt error
leaks a huge false horizontal acceleration, and you integrate that error twice.
A slow ground robot gains nothing and inherits all the drift.

**Wheels give speed, never heading.** `odom1_config` takes vx and nothing else.
In a pivot the wheels turn opposite ways and slip, so wheel-derived heading is
actively misleading. The gyro is simply better at rotation.

**`two_d_mode: true`** pins z, roll and pitch to zero. Not cosmetic:

> Running visual-only, cuVSLAM reported **z = −0.272 m** — the rover believed it
> had sunk 27 cm through the floor. nvblox slices obstacles in a **fixed** height
> band, so a sinking pose drags that band down through the floor and maps floor
> as wall.

`z = 0.000` on `/odometry/filtered` is therefore a **health check**, not a
detail. If z is not exactly zero, the EKF is not running and you are about to
build a smeared map.

### `world_frame: odom`

The EKF fuses in the **continuous** frame and publishes `odom → base_link`. It
does not try to own `map` — cuVSLAM does that, via loop closure. See
[`01-frames-and-tf.md`](01-frames-and-tf.md).

---

## Why `imu_to_base.py` exists

The D555's gyro reports rotation **in the camera's frame**. The EKF needs it in
`base_link`. The node rotates it and re-stamps it.

Without it the EKF output exactly **0 yaw-rate during real turns** — the gyro was
silently ignored, because a message in the wrong frame is not an error, it is
just wrong.

---

## Debugging checklist

| Symptom | Meaning |
|---|---|
| `/odometry/filtered` missing | EKF did not start — `./rover logs fused` |
| z drifts from 0.000 | you are on raw cuVSLAM, not the EKF |
| yaw ignores a real pivot | the gyro is not arriving, or is in the wrong frame |
| output jitters at rest | a sensor's covariance is over-confident |
| output lags reality | `sensor_timeout` (0.2 s) or a slow input |

---

## Current gap

`/wheel_odom` is **dead** — see `TODO.md §2`. So the EKF runs on two sensors, not
three, and has **no independent check on cuVSLAM's speed**. Nothing contradicts
cuVSLAM when it under-reads.

That matters for `TODO.md §1`: the smeared map is suspected to be a pose error in
motion, and the wheels are precisely the sensor that would prove it.

---

**See also:** [`02-visual-odometry.md`](02-visual-odometry.md),
[`01-frames-and-tf.md`](01-frames-and-tf.md).
