# How the robot moves and knows how far it went — in plain language

*Written 2026-07-18 for later study. No robotics background needed. Companion to
NAVIGATION_PIPELINE.md (the technical pipeline) and SENSOR_FUSION_NOTES.md (the
sensor fix). This one answers: "how does it actually measure distance and turns
now, instead of just running the motor for N seconds?"*

---

## The old way: eyes closed

Early on, to move the robot we'd basically say **"run the motor for 10 seconds"**
— like telling someone to *walk forward for 10 seconds with their eyes shut*. You
have no idea how far they really went: could be 2 m, could be 0 if a wheel slipped
or they hit a wall. That's called **open-loop**: you give a command and *hope*. It
was only ever used as a rough hardware test, never for real navigation.

## The new way: eyes open, watching a pencil trace

Now the robot works like a **pencil that always knows where its tip is and which
way it is pointing**, tracing its path as it moves. It keeps a live estimate of
its own position (x, y) and its heading (facing direction), and it **watches that
estimate until the tip reaches the target, then stops.** That's **closed-loop** —
feedback, not a timer.

- `straight 60` is **not** "drive for N seconds." It watches the measured distance
  climb (0 → 20 → 50 cm), slows down near the end, and stops at 60.
- `rotate 90` watches the measured angle climb and stops at 90°.

Because it stops on *measurement*, going faster or slower doesn't change where it
ends up — the feedback corrects for it.

---

## How the "pencil tip" is tracked — two sensors, two jobs

Each sensor does the one thing it's good at:

| What we measure | Which sensor | Plain version |
|---|---|---|
| **Distance** (forward / back) | **the camera** (visual odometry) | it watches the world slide past and measures how far it moved — like counting how much the paper slid under the pencil |
| **Heading** (turning) | **the IMU's gyroscope** | it senses how fast the robot is spinning and adds it up to know which way the pencil now points |

**Is the IMU used?** Yes — the **gyroscope** half of it, for *turning*. This is the
piece we got working on 2026-07-18 (see SENSOR_FUSION_NOTES.md). The other half,
the **accelerometer**, we deliberately do **not** use for distance: getting
distance from it means adding acceleration up twice, and tiny errors blow up to
meters within seconds. So **distance comes from the camera, turning from the gyro.**

Why not measure distance from the wheels? No wheel encoders, and wheels **skid** —
which lies. The camera measures the *real* movement of the robot through the world,
so skidding doesn't fool it.

---

## Are we "drawing"? Literally, yes — two drawings

1. **The path** — the pencil-trace of where the robot has been. Its *length* comes
   from the camera, its *turns* from the gyro. (Formal name: *odometry*.)
2. **The map** — RTAB-Map builds an actual picture of the room from the camera, and
   when it **recognizes a place it has seen before**, it snaps the pencil back onto
   the true spot if the trace has drifted. (Formal name: *loop closure* — like
   checking your drawing against a photo and correcting it.)

So there really is a drawing being made, and it self-corrects.

---

## The whole stack, one line each

- **Camera visual odometry** → how far it moved (the line's length)
- **Gyro (IMU)** → which way it's facing (the line's turns) — fixed 2026-07-18
- **EKF** (`robot_localization`) → blends those two into one clean pencil-tip pose
- **RTAB-Map** → draws the room map and corrects long-term drift
- **Nav2** → reads the pencil tip and steers the motors along a path to the goal,
  re-checking ~20 times a second

## Where you see each part
- The calibration tool `scripts/drive_test.py` is the clearest example of the
  feedback loop: `straight/back <cm>` watches the camera distance; `rotate <deg>`
  watches the gyro angle; each stops at the target (never a timer).
- Real autonomous driving (Nav2) uses the same pencil-tip pose to follow a planned
  route — see NAVIGATION_PIPELINE.md.

## Honest limits (today)
- Distance is accurate to ~±5%, turns to ~±10% — the leftover wobble is the shiny
  floor (camera) and the front camera-weight tilt (gyro), not a fault. The map's
  loop closure cleans up long-term drift.
- The rover is currently tethered by power wires, so big in-place spins can tangle
  it — keep turns modest until it's untethered.
