# Task 08 — it maps the room by itself

> **STUB.** Written in full when you get here. **This is the stage-2 goal.**

**Needs:** tasks 01–07 all passed. No exceptions — see "Why this is last".
**Moves the robot:** YES, and **without a human deciding where.**

---

## Goal

Switch it on in an unmapped room. It drives around, works out where it has not
looked, goes and looks, and stops when there is nothing left. You touch nothing.

## The idea: frontier exploration

A frontier is the **boundary between mapped-free and unknown** — the edge of
what you know, reachable from where you are. The whole algorithm is four lines:

```
1. find all frontier cells in the occupancy grid
2. cluster them, discard clusters smaller than the robot
3. pick one (nearest is the usual choice) and send yourself a nav2 goal
4. arrive -> scan -> recompute -> repeat.  no frontiers left = done
```

The elegance is that it needs no new perception. The map already distinguishes
free / occupied / unknown, and step 3 is the goal you were clicking by hand in
task 07. **Autonomy here is a decision layer on top of stage 1, not new sensing.**

## The hard part: an 87° robot doing a 360° job

Standard frontier exploration assumes a 360° lidar. This rover sees a forward
cone. Drive down a corridor and everything beside you stays unknown, so
frontiers appear next to places you have physically already driven past, and the
rover can loop forever without ever "finishing".

**The chosen answer is to spin.** At each waypoint:

```
drive to frontier
  -> stop
  -> slow 360 deg spin in place, sweeping the cone around a full circle
  -> the map now has a complete disc around this point
  -> recompute frontiers
  -> repeat
```

Slow and inelegant, and it is the honest way for a forward-sensing robot.

> ⚠️ **The spin is the risky part, not the driving.** Visual odometry is at its
> worst in rotation — features leave the frame fast and there is little parallax.
> This is why task 03's gate now includes a full 360° hand rotation. If heading
> is off by 30° after one turn, the rover writes an entire scan into the map at
> the wrong angle and the map gets worse the more it explores.
>
> Spin **slowly**. Rotation speed is the main tuning knob in this task.

## Likely work

1. A `frontier_explorer` node: subscribe the occupancy grid, find and cluster
   frontiers, publish the chosen one as a `NavigateToPose` goal.
2. A scan behaviour: rotate in place at a tuned rate, with the pose watched
   throughout — abort the spin if `/odom/health` reports the pose is untrusted.
3. A termination rule, and an honest answer to "what if a frontier is
   unreachable?" (the usual failure: it retries the same impossible goal
   forever — blacklist goals that fail twice).
4. A hard stop: a wall-clock limit and a big red manual stop, because this is
   the first task where nobody is deciding where it goes.

## Safety — read before running this once

This is the first time the robot moves with **no human choosing the
destination**. Before it ever runs:

- `TODO.md §3` **must be resolved.** The collision monitor's obstacle source
  publishes at ~0.5 Hz against a 2.5 s timeout, and a stale source is *ignored*,
  not treated as danger. An autonomously exploring robot with no working
  collision monitor is genuinely dangerous, and `safety_guard.py` would be the
  only thing between it and the furniture.
- The rover is blind to its sides and back. It can turn into something it has
  never seen.
- Tether length, clear floor, hand on the power, `./rover.sh stop` ready.
- First run in **one small room with the door shut**, not the whole flat.

## Why this is last

Every part of this is stage 1 plus a decision. If the pose is wrong, an
autonomous robot does not fail politely — it gets lost faster, with more
confidence, and writes the mistake into the map permanently.

Specifically, task 08 cannot work unless:

| Needs | From |
|---|---|
| a metrically honest pose | task 02 |
| heading that survives a full spin | task 03 |
| walls that are lines, not blobs | task 04 |
| a costmap that does not call the floor an obstacle | task 06 |
| nav2 reliably reaching a clicked goal | task 07 |

## Gate

*To be written when we get here, from real measurements.* Roughly:

```
[ ] switched on in an unmapped room, produces a complete map
[ ] no human input from start to finish
[ ] no collision
[ ] it decides it is FINISHED, rather than being stopped
[ ] the resulting map still passes task 04's occ/free < 0.3
```

That last line is the one that matters. A map made while exploring must be as
good as one made while you pushed it by hand — otherwise autonomy cost you
quality, and it is not worth having.
