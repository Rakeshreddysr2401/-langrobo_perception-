# Task 05 — a map that survives being switched off

> **STUB.** Written in full when you get here — the gate needs numbers we do not
> have yet, and a made-up gate is worse than no gate.

**Needs:** task 04 passed (walls are lines).
**Moves the robot:** no.

## Goal

Save the map, restart everything, and have the rover **find itself on it**.

## What you will learn

- Why the map currently lives in `odom`, and what that costs
- What **relocalization** is: recognising a place you have seen and snapping the
  pose to it
- Why `map` jumps and `odom` does not, and why a costmap that jumps under a
  moving robot is dangerous

## Known state going in

- `config/nvblox.yaml` sets `global_frame: "odom"`, and both nav2 costmaps use
  `odom` too. `bt_navigator` uses `map`.
- **Consequence:** the map is built in the drifting frame. It is never corrected
  by loop closure, and it does not line up across sessions.
- This is NVIDIA's reference wiring and it is deliberate — a costmap in `map`
  teleports under the controller at every loop closure.
- cuVSLAM already has the machinery: `/slam/save_map`, `/slam/localize`, and
  `map_dir` pointing at the `/maps` volume. It has not been exercised.

## The real question this task settles

Whether to move nvblox to the `map` frame. That trades "the costmap never jumps"
for "the map persists and gets corrected". Both are defensible; make the choice
with a driven map in front of you, not on paper.

## Gate

*To be written from real measurements.* Roughly: save a map, restart the stack,
and have the rover localize on it within some bound you will have measured by
then.

## Learn first

Read [`01-frames-and-tf.md`](../knowledge/01-frames-and-tf.md) then [`04-tsdf-esdf-voxels.md`](../knowledge/04-tsdf-esdf-voxels.md) — why `map` jumps and `odom` does not, and what that costs a map built in `odom`.

Not to memorise. Just so the words in this task mean something.

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
