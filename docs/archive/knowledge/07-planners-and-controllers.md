# Planners, controllers and behaviour trees

**Used by:** Phases 3 and 4 (navigation).
> **Measured on this rig (Phase 3):** this rover **cannot turn in place** — see
> [TODO §14](../TODO.md). nav2 assumes rotation is free, so almost every choice
> in our config is a workaround: RPP runs `use_rotate_to_heading: false`,
> `allow_reversing` is off, the yaw tolerance is loose, and the **Spin recovery
> is removed**. Removing the Spin *behaviour* is not enough — nav2's default
> behaviour trees hard-require a `spin` action server, and **both** trees must be
> overridden or `bt_navigator` refuses to load at all. Every such setting is
> marked `NO-PIVOT` so they can be reverted together. See [PHASE3](../PHASE3.md).

**Code:** `phase3/config/nav2.yaml`, `phase3/bt/navigate_to_pose.xml`

---

## Two different questions

Navigation splits into two problems that need completely different algorithms.

**Global planning** — *"which way round the sofa?"* Runs over the whole known
map, cares about topology, does not care about wheel dynamics. Recomputed
occasionally.

**Local control** — *"what velocity should I send in the next 50 ms?"* Cares
about how the robot actually accelerates and turns, and about obstacles that
appeared a second ago. Runs continuously.

A global planner that tried to model dynamics would be far too slow. A controller
that tried to plan the whole route would be far too myopic. So: two layers.

---

## Global: `NavfnPlanner`

Dijkstra/A* over the global costmap. Finds the cheapest path from here to the
goal, publishes it on `/plan` — the green line in RViz.

Cheapest, not shortest: it prefers routes through low-cost cells, which is why
paths sit in the middle of corridors rather than scraping walls. See
[`06-costmaps-and-inflation.md`](06-costmaps-and-inflation.md).

---

## Local: MPPI

**Model Predictive Path Integral** control. The idea is unusual and worth
understanding, because it explains the robot's behaviour.

Every cycle, MPPI:

1. **samples** hundreds of random velocity sequences for the next second or so
2. **rolls each one forward** through a model of how the robot moves, producing a
   candidate trajectory
3. **scores** each: how close to the global path, how far from obstacles, how
   fast, how smooth
4. **combines** them, weighted by score, into one command

It does not follow the plan exactly. It finds a *dynamically achievable* motion
that mostly agrees with the plan — which is why the robot rounds corners the
planner drew as sharp, and can dodge something that was not on the map.

**Limits (`phase3/config/nav2.yaml`):**

```yaml
vx_max: 0.30   # m/s
wz_max: 1.0    # rad/s
```

Then `velocity_smoother` caps acceleration (0.25 / 1.0), because a step change in
commanded velocity is a step change in current draw.

**Debugging insight:** if MPPI crawls, it is because every sampled trajectory
scores badly — usually because the costmap says the robot is already in a
high-cost region. That is what the floor-as-obstacle failure looked like:
0.082 m/s against a 0.30 limit.

---

## Behaviour trees: the decision layer

`bt_navigator` runs a **behaviour tree** — a structured way to express "try this;
if it fails, try that; if that fails, give up."

Nodes are composed:

- **Sequence** — run children in order, fail if any fails
- **Fallback** — try children in order until one succeeds
- **Decorator** — modify a child (retry N times, run at X Hz)

The classic nav2 tree is roughly: *follow the path; if that fails, try recovery
behaviours; if those fail, abort.*

### Ours recovers by turning first, not by reversing

Standard nav2 recovery is: clear the costmap, spin in place, back up, try again.

This page used to claim our tree omitted both Spin and BackUp because they are
blind moves. That was half wrong and the wrong half mattered. The tree never had
Spin — removed under TODO 14, when the rover genuinely could not pivot — but it
always had **two** BackUps, so the only recovery it could actually perform was
the blind one. On a rover that sees 87° forwards, nothing below 10 cm and
nothing at all behind it, reversing is the move with no sensor behind it.

TODO 14 was fixed on 2026-08-22 (65 °/s, measured). TODO 40 rebuilt the recovery
round-robin around that:

    clear costmaps → Spin +90° → BackUp 0.30 m → Spin −90° → Wait 5 s

Spin first, because turning is the recovery this rover can *watch itself do* —
it swings the camera onto new ground, which frees the rover and simultaneously
gives nvblox something to observe. An unobserved cell under the robot is one of
the standard reasons NavFn returns no path at all, so the turn often fixes the
planner as a side effect.

**The trap, if you ever re-tune this:** `behavior_server`'s `max_rotational_vel`
/ `min_rotational_vel` / `rotational_acc_lim` are read unnamespaced by Spin, and
stock nav2 values (0.6 / 0.2 / 1.0) sit *entirely inside* this chassis's
0.8 rad/s scrub breakaway. Spin would command a rotation, the wheels would sit,
and the behaviour would time out — reproducing TODO 14's symptom exactly and
"confirming" a fault that no longer exists. See `nav2.yaml`.

**This becomes interesting in task 08.** Autonomous exploration needs *some*
answer to being stuck. The answer will not be a blind recovery; it will be
blacklisting the unreachable frontier and choosing another.

---

## The full velocity chain

```
controller_server   ──> /cmd_vel_nav
velocity_smoother   ──> /cmd_vel_smoothed     caps acceleration
collision_monitor   ──> /cmd_vel_shim         runtime veto on obstacles
safety_guard        ──> /cmd_vel              final gate
                          └──> ESP32 ──> wheels
```

**One path, no bypass.** Every stage can only slow or stop the robot, never speed
it up. If you ever find two things publishing `/cmd_vel`, that is a bug — the
last writer wins and the safety chain is bypassed.

⚠️ `collision_monitor`'s obstacle source is effectively dead (`TODO.md §3`), so
`safety_guard.py` is currently the only real protection.

---

## Goals and frames

`bt_navigator` uses `global_frame: map`, while both costmaps use `odom`. So a
goal clicked in RViz arrives in `map` and is transformed through cuVSLAM's
`map → odom`.

That is why a goal **stays put** when a loop closure corrects the pose: it was
specified in the frame that gets corrected. See
[`01-frames-and-tf.md`](01-frames-and-tf.md).

---

## Debugging checklist

| Symptom | Meaning |
|---|---|
| no `/plan` at all | the goal is unreachable in the global costmap, or in a bad frame |
| plan exists, robot does not move | MPPI scoring everything badly, or the velocity chain is vetoing |
| crawls far below `vx_max` | high-cost region — suspect the floor being mapped |
| "Failed to make progress" | `SimpleProgressChecker` gave up; usually the crawl above |
| robot moves without a goal | something else is publishing `/cmd_vel` |
| reaches roughly the goal then stops short | `SimpleGoalChecker` tolerances |

---

**See also:** [`06-costmaps-and-inflation.md`](06-costmaps-and-inflation.md),
[`08-frontier-exploration.md`](08-frontier-exploration.md).
