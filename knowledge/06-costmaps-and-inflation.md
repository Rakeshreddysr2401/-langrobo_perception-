# Costmaps and inflation

**Used by:** tasks 06, 07, 08.
**Code:** `config/nav2.yaml` (`local_costmap`, `global_costmap`)

---

## What a costmap is

A 2D grid where each cell holds a **cost** from 0 to 254, meaning "how bad would
it be to have the robot's centre here?"

Three bands matter:

| Cost | Name | Meaning |
|---|---|---|
| 0 | free | go anywhere |
| 1–252 | inflated | passable, but the planner will avoid it if it can |
| **253** | inscribed | the robot's body would touch an obstacle |
| **254** | lethal | there is an obstacle here |

The planner does not search for the *shortest* path — it searches for the
**cheapest**, which is why it naturally hugs the middle of corridors rather than
scraping walls.

## Inflation: why obstacles look bigger than they are

Planners treat the robot as a **point**. That is a huge simplification, and it
only works if you grow every obstacle by the robot's radius first.

That is inflation: around each lethal cell, cost decays outward. Within the
robot's inscribed radius the cost is 253 — the body would collide. Beyond that it
falls off gradually, so paths *prefer* clearance without *requiring* it.

**The signature of a wrong robot radius:** too small and it clips corners; too
large and it refuses to fit through doorways that are physically fine.

---

## Two costmaps, two jobs

| | `global_costmap` | `local_costmap` |
|---|---|---|
| Covers | the whole known map | a small rolling window around the robot |
| Used by | `planner_server` — the whole route | `controller_server` — the next second |
| Updated | slowly | fast |

The global one answers "which way around the sofa?"; the local one answers "what
is immediately in front of me right now?".

Both of ours use:

```yaml
global_frame: odom
resolution: 0.05
plugins: NvbloxCostmapLayer, InflationLayer
```

### Where the obstacles come from

`NvbloxCostmapLayer` reads nvblox's 2D slice directly — see
[`04-tsdf-esdf-voxels.md`](04-tsdf-esdf-voxels.md). So the chain is:

```
depth image + pose -> TSDF -> ESDF -> 2D slice -> costmap layer -> inflation -> planner
```

Every link inherits the errors of the one before it. A pose error becomes a
smeared wall, becomes a fat obstacle, becomes a costmap that refuses to plan.

### Both costmaps are in `odom`, not `map`

Deliberate. A costmap in `map` **teleports under the robot** at every loop
closure, and a controller mid-manoeuvre cannot cope with obstacles jumping.

The cost is that the map is built in the drifting frame, so it is never corrected
and does not persist across sessions. See [`01-frames-and-tf.md`](01-frames-and-tf.md)
and task 05.

---

## The failure pattern worth memorising

> **A wide band of nonzero cost with NO lethal cells anywhere is the floor being
> mapped as an obstacle.**

Real cells produce a lethal core with inflation around it. The floor produces
inflation with nothing at its centre, because the "obstacle" is a thin sheet of
noise rather than a solid thing.

What it did here, end to end:

1. camera TF said 0.200 when the truth was 0.163
2. floor deprojected to +0.037 instead of 0.000, into the 0.05 slice band
3. nvblox mapped the floor
4. costmap grew an inflation plateau across the full ±1 m width from 0.25 m ahead
5. MPPI crawled at **0.082 m/s** against `vx_max` 0.30
6. the collision monitor's SlowZone cut it to **0.033 m/s**
7. the rover moved **0.9 cm in 30 s**; nav2 aborted on "Failed to make progress"

Nothing in that chain reported an error. Every layer did exactly what it was told.

---

## The collision monitor is a separate thing

Do not confuse it with the costmap. The costmap informs *planning*; the collision
monitor is a **runtime veto** sitting in the velocity chain:

```
controller -> /cmd_vel_nav -> smoother -> /cmd_vel_smoothed
   -> collision_monitor -> /cmd_vel_shim -> safety_guard -> /cmd_vel
```

It watches `/perception/depth_points` and can slow or stop the robot regardless
of what the plan says.

⚠️ **Ours is effectively unprotected.** That source publishes at ~0.5 Hz with
gaps to 7.3 s, against a `source_timeout` of 2.5 s. And when a source is stale
the monitor **ignores it** rather than treating it as danger — so the protection
silently disappears rather than failing loudly. `TODO.md §3`, and a hard blocker
for task 08.

---

## Inspecting a costmap

```bash
ros2 topic echo /global_costmap/costmap --once | head -20
```

In RViz, add a Map display on `/global_costmap/costmap` with colour scheme
`costmap`. Look for:

- lethal cells where real obstacles are
- inflation around them, not floating free
- **zero cost on open floor** — if the floor is grey, go back to task 06

---

**See also:** [`04-tsdf-esdf-voxels.md`](04-tsdf-esdf-voxels.md),
[`07-planners-and-controllers.md`](07-planners-and-controllers.md).
