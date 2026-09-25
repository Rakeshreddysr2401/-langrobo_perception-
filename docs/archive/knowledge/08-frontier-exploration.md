# Frontier exploration

**Used by:** Phase 2 and beyond (exploration).

---

## The idea

A **frontier** is the boundary between **known-free** and **unknown** space. It is
where the map ends *and you could go and see more*.

The whole algorithm:

```
1. scan the occupancy grid for free cells adjacent to unknown cells
2. cluster them; discard clusters smaller than the robot
3. pick one (nearest reachable is the usual choice)
4. send yourself a nav2 goal to it
5. arrive, look, recompute
6. no frontiers left -> the space is fully mapped, stop
```

What makes it elegant: **it needs no new perception.** The map already
distinguishes free / occupied / unknown, and step 4 is exactly the goal you were
clicking by hand in task 07.

> Autonomy here is a **decision layer** on top of a working stack — not new
> sensing. That is precisely why every layer beneath it has to be verified first.
> A robot that explores on a bad pose does not fail politely; it gets lost faster
> and writes the mistake permanently into the map.

---

## Free / occupied / **unknown**

Three states, and the third is the one people forget.

| Value | Meaning |
|---|---|
| `0` | free — observed, nothing there |
| `100` | occupied — observed, something there |
| `-1` | **unknown — never observed** |

Unknown is not "empty". It is "no information". Frontier exploration is entirely
built on the difference, and the reason a robot can decide *where to go next* is
that it can tell "I looked and it was clear" from "I never looked".

---

## The hard part here: an 87° robot doing a 360° job

Standard frontier exploration assumes a 360° lidar, so wherever the robot goes,
everything around it becomes known.

This rover sees a forward cone. Drive down a corridor and everything **beside**
you stays unknown — so frontiers appear right next to places you have physically
already driven past, and the rover can loop forever without ever finishing.

### The chosen answer: spin at each waypoint

```
drive to frontier
  -> stop
  -> slow 360 deg spin, sweeping the cone around a full circle
  -> the map now has a complete disc around this point
  -> recompute frontiers
  -> repeat
```

Slow and inelegant, and it is the honest way for a forward-sensing robot.

### Why the spin is the risky part

Rotation is what visual odometry handles **worst** — features leave the frame
fast and there is almost no parallax. See
[`02-visual-odometry.md`](02-visual-odometry.md).

So the spin, which is meant to *improve* the map, is also the moment the pose is
most likely to be wrong. And a bad heading during a scan does not produce a small
error: **the whole sweep gets written into the map at the wrong angle.**

Mitigations:

- spin **slowly** — rotation rate is the main tuning knob
- the gyro should carry the pose through it (task 03)
- watch `/odom/health` during the spin and abort if trust drops
- task 03's gate now includes a full 360° hand rotation, specifically to find
  this out early at zero speed

---

## Design decisions this task will force

**Which frontier?** Nearest is simplest and works well. Largest gets more map per
trip but wanders. Some implementations weight by both.

**When is a frontier not worth it?** Clusters smaller than the robot are noise —
usually sensor speckle at the edge of the cone.

**What about unreachable frontiers?** The classic failure: a frontier behind
glass, or beyond a doorway too narrow. The rover retries the same impossible goal
forever. Standard fix: **blacklist a goal after it fails twice.**

**When is it done?** No frontiers larger than the threshold remain. In practice
also needs a wall-clock limit, because "done" can be surprisingly hard to reach.

**How does it not get stuck?** Note this cannot use nav2's usual blind recoveries
— Spin and BackUp are deliberately absent
([`07-planners-and-controllers.md`](07-planners-and-controllers.md)). The answer
must be blacklist-and-choose-another.

---

## Safety, because nothing human is choosing the destination

This is the first behaviour where **no person decides where the robot goes**.

- `TODO.md §3` must be resolved first: `/perception/depth_points` must be proven
  to hold ≥ 5 Hz **while driving**, against the monitor's 1.5 s `source_timeout`.
  A source that goes stale does not make the rover blind — the monitor is
  fail-safe and holds it at zero — but an exploration run that freezes at a
  random frontier and cannot say why is its own kind of dangerous, and here
  nobody is watching the goal it was heading for.
- The rover is blind to its sides and back. It can turn into something it has
  never seen.
- First run in **one small room with the door shut**.
- A wall-clock limit and a manual stop are part of the feature, not extras.

---

**See also:** [`04-tsdf-esdf-voxels.md`](04-tsdf-esdf-voxels.md) (where
free/occupied/unknown comes from), [`07-planners-and-controllers.md`](07-planners-and-controllers.md)
(how the goal is executed).
