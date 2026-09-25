# Visual odometry and SLAM (cuVSLAM)

**Used by:** Phase 1 (pose).
> **Measured on this rig:** cuVSLAM can fail in a way this document does not
> cover — it can **diverge silently**, reporting healthy landmark counts and a
> steady rate while its pose is metres wrong and frozen. Feature tracking being
> fine is not the same as the pose being fine. See
> [PHASE1 §3](../PHASE1.md) and [TODO §3](../TODO.md), which also corrects the
> "speed limit" this file's drift discussion would imply: teleports track
> **texture**, not velocity.

**Code:** `phase1/nodes/vo_node.py`  ·  **What we measured:** [PHASE1.md](../PHASE1.md) §3, §5

---

## Odometry vs SLAM

**Odometry** answers *"how far have I moved since I started?"* by adding up small
motions. It is smooth and it drifts, because every small error is kept forever.

**SLAM** — Simultaneous Localization And Mapping — also builds a map of
landmarks, so when you return somewhere you **recognise it** and can correct the
accumulated drift. That correction is called **loop closure**.

cuVSLAM does both. It publishes odometry continuously on `/odom`, and it
maintains keyframes (up to 300, see `max_map_size`) so it can close loops. The
correction lands in the `map → odom` transform — see
[`01-frames-and-tf.md`](01-frames-and-tf.md).

---

## How stereo visual odometry actually works

Four steps, and each has a failure mode worth knowing:

**1. Find features.** Corners and distinctive patches in the left IR image. → A
blank white wall has no features. Nothing to track.

**2. Match left to right.** The same feature appears slightly further left in the
right image; that horizontal offset is **disparity**, and disparity gives depth
by triangulation. → Repetitive patterns (tiles, radiators) match wrongly.

**3. Track over time.** Follow those features into the next frame. How they moved
tells you how the camera moved. → If the camera moves too far between frames the
features leave the view and tracking breaks.

**4. Solve.** Find the camera motion that best explains all the feature movement.

### Why rotation is the hard case

In a **translation**, features slide across the image gradually and nearby things
shift more than distant things — that parallax is rich information.

In a **rotation**, everything sweeps across the image at the same rate, there is
almost no parallax, and features leave the frame within a few frames.

So visual odometry is at its weakest exactly when you spin. This is not a cuVSLAM
flaw, it is geometry. It is also why:

- the gyro is fused (task 03) — it measures rotation directly
- task 03's gate includes a full 360° hand rotation
- task 08 spins **slowly** at each waypoint

---

## It uses IR, not colour

| Stream | Who uses it |
|---|---|
| `infra1` + `infra2` (stereo IR) | **cuVSLAM** |
| `depth` | **nvblox** |
| `color` | nobody, in this repo |

cuVSLAM does its own triangulation from the raw stereo pair. It does not want the
camera's depth image.

---

## The trap: the IR emitter

The D555 has an infrared projector that paints a dot pattern on the scene so
stereo matching works on blank surfaces. Excellent for depth. **Catastrophic for
visual odometry here.**

The projector is **bolted to the camera**, so the pattern is repainted from the
camera's own viewpoint every frame. On a low-texture floor the dots stay in
almost the same place in the image *even as the rig moves* — so the tracker sees
features that are not moving, and concludes the robot is not moving.

Measured 2026-08-09:

| emitter | real push | `/odom` said | error |
|---|---|---|---|
| ON | 100 cm | **26 cm** | ~4× under |
| OFF | 100 cm | **97 cm** | 3% |

Hence `depth_module.emitter_enabled:=0` in `rover`. Passive-stereo depth stays
metrically correct either way (140 cm wall reads 1.40 m), it is just noisier on
textureless surfaces. **Localization beats pretty depth.**

---

## The trap that matters most: silent failure

cuVSLAM **does not report that it has failed.**

- Starve the camera below ~10 Hz and cuVSLAM **freezes and never recovers** —
  while still publishing `slam_pose_ok: true`.
- Under-read translation 4× and it reports success throughout.

There is no exception, no error topic, no log line. The stack keeps navigating on
a pose that stopped updating.

This is why:

- `./rover status` measures **rates**, not whether a topic exists
- `/vo/status` reports landmarks at all — it cross-checks cuVSLAM against
  `/cmd_vel` and `/wheel_odom` and publishes `/odom/health`
- task 02's gate is a **tape measure**, not "is it publishing"

> A pose that is confidently wrong is more dangerous than one that is missing.
> A missing pose stops the robot. A wrong one drives it somewhere.

---

## SLAM can diverge, and the frame split is what saves you

Loop closure's correction goes in `map -> odom`, never into `odom -> base_link`.
The usual reason given is REP-105's: `odom` must be continuous, and a closure is
a jump. There is a second reason, and 2026-08-22 demonstrated it:

```
odom -> base_link   [-0.828, 0.876,  0.000]     healthy, z exactly 0
map  -> odom        [-10.5,  -22.2, 87.685]     87 metres UP
correction_m        91.045
```

The SLAM optimiser had settled into a pose 87 m above the floor and wanted to
"correct" the world by 91 m — for a rover that had driven about four. Odometry
was untouched and perfectly fine the whole time. **The garbage stayed in the
frame nothing critical consumes**, so nav2 and nvblox never saw it.

Two things worth carrying forward:

- **`planar_constraints = True` did not prevent this.** It was set specifically
  to make a vertical divergence impossible, and it happened anyway. Do not treat
  that flag as a guarantee.
- **Gate the correction like you gate the pose.** A loop-closure correction is
  accumulated drift made visible: over a room it is centimetres to a metre or
  two, never tens of metres and never vertical. `vo_node` now discards any
  correction beyond 0.30 m of z or 5 m of magnitude rather than publishing it.

A gate stops the damage; it does not make closure work. See `TODO.md §16`.

---

## Reading `cuvslam_node.py`

| Parameter | Default | Why it is there |
|---|---|---|
| `publish_odom_tf` | `true` | set **false** at task 03 so the EKF owns `odom → base_link` |
| `planar_constraints` | `true` | floor rover — it cannot fly, so do not solve for it |
| `max_map_size` | 300 | keyframes kept for loop closure |
| `map_dir` | `/maps/current` | where saved maps live (the `/maps` volume) |
| `left_ns` / `right_ns` | `infra1` / `infra2` | the stereo pair |

Publishes `/odom`, `/visual_slam/tracking/odometry` (same data), `/slam/status`,
and the `map → odom` transform.

---

## Debugging checklist

| Symptom | First thing to check |
|---|---|
| `/odom` at 0 Hz, camera fine | cuVSLAM **frozen** — restart the layer |
| distance reads far too short | emitter on? low-texture floor? |
| pose jumps around | two publishers on one TF edge |
| heading wrong after a turn | rotation — the gyro should be carrying it |
| works then dies after minutes | camera rate sagging. Check `status`, check load. |

---

**See also:** [`01-frames-and-tf.md`](01-frames-and-tf.md),
[`03-sensor-fusion-ekf.md`](03-sensor-fusion-ekf.md).
