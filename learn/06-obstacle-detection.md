# 06 — Obstacle detection: what is in my way?

> **STUB.** Written in full when you get here.

**Needs:** issue 05 passed (you have a real saved map).
**Motors move:** no — you walk obstacles in front of a stationary rover.

---

## Goal

Understand every layer that decides "the rover must not go there", and see each
one separately in RViz.

## What you will learn

There are **three independent** obstacle systems, and they are easy to confuse:

| Layer | What it is | Reacts in |
|---|---|---|
| **nvblox ESDF → costmap** | The planner's map of where it can't go | ~0.1 s |
| **collision_monitor** | nav2's emergency brake, on a live depth cloud | ~0.1 s |
| **safety_guard** | 3 forward bumper layers, last line of defence | immediate |

Also: what **inflation** is — why the costmap marks a margin *around* obstacles
rather than just the obstacle itself, and how that radius relates to the rover's
physical width.

## Known state going in

- `safety_guard` is built and verified live (3 forward layers).
- `collision_monitor`'s obstacle source is `/perception/depth_points` from
  `nodes/depth_to_cloud.py` at 10 Hz.
  > ⚠️ If that source goes **stale**, the monitor fail-safes to "stop due to
  > invalid source" and holds the rover at zero. A rover that won't move is often
  > this, not a broken motor.
- **The main task carried into this issue:** lower `esdf_slice_min_height` from
  the **0.12** workaround toward **~0.06**.
  - It was raised purely to dodge the 4 cm camera-height error, at the cost of
    no longer seeing obstacles shorter than 12 cm.
  - That error is fixed now (floor reads 0.000 ±0.004), so the band can come down.
  - **Validation:** after lowering it, check the costmap has **no floor plateau** —
    `ros2 topic echo /local_costmap/costmap --once`; a wide band of nonzero cost
    with no lethal cells *is* the floor being mapped as an obstacle. Revert to
    0.12 if it comes back.

## Likely work

Walk a box, a chair leg, and something short (a shoe) in front of the camera and
watch each layer respond. Then lower the slice band and re-validate.

## Gate

*To be written when we get here.* Roughly: obstacles appear in the costmap at the
right place and size, no floor plateau after lowering the band, and a short
object (under 12 cm) is now detected where previously it was invisible.
