# 08 — Path planning: why did it pick *that* path?

> **STUB.** Written in full when you get here.

**Needs:** issue 07 passed (the rover reliably reaches clicked goals).
**Motors move:** **YES.**

---

## Goal

Stop treating the planner as a black box. Change one parameter at a time, watch
the path change in RViz, and be able to explain why.

## What you will learn

- Why the planned path hugs walls or swings wide — and which parameter controls
  it (**inflation radius** and **cost scaling factor**, mostly)
- The difference between the **global plan** (whole route, recomputed
  occasionally) and the **local plan** (next couple of seconds, recomputed
  constantly)
- What MPPI's **critics** are: separate scoring functions for path-following,
  obstacle clearance, goal alignment, preferring forward motion. Tuning is mostly
  adjusting how loudly each one argues.
- Why **goal tolerance** matters, and why too tight a tolerance makes the rover
  dither forever near the goal instead of declaring arrival

## Known state going in

- Everything is in `config/nav2.yaml` (396 lines) — planner, MPPI, critics,
  costmaps, speed caps.
- Remember the edit loop: the repo is mounted read-only over the baked image, so
  **edit the YAML on the Jetson → restart the stack → it takes effect. No
  rebuild.**

## Likely work

A controlled experiment per parameter. Same start, same goal, same obstacle,
change one number, screenshot the path. Build up a small table of
*parameter → visible effect*, which is worth far more than any tuning guide
because it is measured on *your* rover, in *your* room.

Good candidates:

| Parameter | Expected effect |
|---|---|
| `inflation_radius` | How wide a berth the path gives obstacles |
| `cost_scaling_factor` | How sharply cost falls off with distance |
| MPPI `PathAlignCritic` weight | How tightly it hugs the global plan |
| MPPI `ObstaclesCritic` weight | How aggressively it avoids |
| `xy_goal_tolerance` | How close is "arrived" |

## Gate

*To be written when we get here.* Roughly: you can predict, before running it,
how a given parameter change will alter the path — and be right.

---

## After this

You have finished the plan in `PRD.md`: the rover builds a map as you drive,
keeps it, and drives to any goal you click, and you understand every stage.

Sensible next steps, in rough order:

1. **Place memory** — name a spot, come back to it in a later session
2. **"Go near the bottle"** — the original goal, now on a foundation you can see
3. **Autonomous exploration** — frontier-seeking, maps a room unattended
4. **Pan/tilt head** — fixes the ~87° forward-cone blindness (needs firmware work)

The full long-term picture is in
[`../orin-nav-stack/SYSTEM_INTEGRATION.md`](../orin-nav-stack/SYSTEM_INTEGRATION.md).
