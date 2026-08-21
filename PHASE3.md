# Phase 3 — Autonomous navigation

**Goal: click a point on the map, and the rover plans a route and drives there,
going round obstacles.**

**Status: running, not yet driven to a goal.** Every server is up and both
costmaps carry real obstacle data. What has not happened is a goal being set and
the rover reaching it — see §6.

---

## 1. What nav2 needed, and where each piece came from

| requirement | supplied by |
|---|---|
| a pose | `fusion_node` — `/odom` + TF `odom → base_link` (Phase 1b) |
| obstacles | nvblox ESDF slice via `NvbloxCostmapLayer` (Phase 2b) |
| a footprint | derived from Phase 1's tape measurements — §3 |
| a way to drive | `/cmd_vel` → ESP32, proven end to end (Phase 1) |

Nothing new was measured for Phase 3. It consumes what the earlier phases
produced, which is the point of having built them in order.

---

## 2. The pieces

| node | job | choice made |
|---|---|---|
| `planner_server` | route from A to B | **NavFn** with A*, `allow_unknown: false` |
| `controller_server` | follow the route | **Regulated Pure Pursuit** |
| `behavior_server` | recoveries when stuck | **BackUp and Wait only** — no Spin |
| `bt_navigator` | orchestrates the above | custom behaviour trees, §4 |
| `velocity_smoother` | soften velocity steps | between controller and wheels |

**`allow_unknown: false`** matters: 79% of the map is still unknown, and planning
through it would drive the rover blind into space it has never seen.

**Regulated Pure Pursuit over DWB** because RPP follows a path by aiming at a
point ahead and *arcing* toward it, which is what this rover can actually do.
DWB samples candidate velocity pairs including rotations the rover cannot
execute.

**The velocity smoother is not cosmetic.** A step change in commanded velocity
goes straight to the PID, and on a skid-steer that is how you get wheel slip —
which corrupts the very odometry nav2 is steering by.

---

## 3. The footprint

Derived from Phase 1's measurements rather than guessed. From the rover centre:

| | value | from |
|---|---|---|
| front | **+0.180 m** | camera front face: 17.0 cm + ~1 cm deep |
| rear | **−0.170 m** | rear wheel centre + wheel radius: 12.5 + 4.25 |
| side | **±0.190 m** | wheel centre 17.0 + ~2 cm tyre half-width |

**35 cm long × 38 cm wide.** The tyre half-width is the one assumption; verify
with a tape before trusting the rover through a tight gap.

Confirmed live on `/global_costmap/published_footprint`:
`(0.187, 0.161), (0.191, −0.239), (−0.179, −0.242), (−0.183, 0.158)` — the
rectangle, rotated by the rover's small yaw.

Inflation radius **0.35 m**. Half the footprint diagonal is ~0.13 m; the rest is
margin for a pose good to ~7 cm and a rover that can only steer by arcs.

---

## 4. Everything shaped by the rover not turning in place

**This rover cannot pivot.** Measured 2026-08-21, [TODO](TODO.md) §14: a pivot
command drives instead of turning — the two sides never counter-rotate.

nav2 assumes rotation in place is free. Almost every adaptation in
`phase3/config/nav2.yaml` exists because of this, and each is marked `NO-PIVOT`
so they can be found and reverted together once §14 is fixed:

| setting | value | without it |
|---|---|---|
| `use_rotate_to_heading` | **false** | RPP stops and rotates to face the path before moving. The rover would sit still commanding a turn that never happens until the progress checker timed out |
| Spin recovery | **removed** | nav2's *first* recovery is the one manoeuvre this rover cannot do — it would turn a recoverable situation into a stuck one |
| `allow_reversing` | false | cuVSLAM under-reads reverse by ~6% |
| `yaw_goal_tolerance` | 0.5 rad (29°) | a final heading cannot be corrected in place, so demanding better guarantees failure |

**Expect goals needing tight turns, or goals behind the rover, to fail.** That is
the mechanical fault showing through, not a tuning problem.

---

## 5. Three things that fail silently

Each cost real time and none produced an obvious error.

**Removing the Spin *behaviour* is not enough.** nav2's default behaviour trees
**hard-require a `spin` action server**, so `bt_navigator` refused to load at all:

```
"spin" action server not available after waiting for 1.00s
Exception when loading BT -> Failed to bring up all requested nodes
```

And **both** trees need overriding — `navigate_to_pose` *and*
`navigate_through_poses`. The second is loaded at startup even when nothing ever
calls it, so fixing only the first still fails the whole bringup.
`phase3/bt/` holds both, with Spin replaced by a longer BackUp.

**Costmaps only publish deltas.** They are TRANSIENT_LOCAL, but by default send
one full map on a size change and increments after. A subscriber that connects
later — RViz, or a health check — sees **nothing** while the costmap runs
perfectly. `always_send_full_costmap: true` fixes it.

**`Invalid frame ID "odom"` on startup is a race, not a fault.** The costmaps come
up before `fusion_node`'s TF is flowing and recover on their own.

---

## 6. Measured, and what is untested

Running:

```
global costmap  0.8 Hz   240×240 @ 5 cm   lethal 1770  inflated 165
local  costmap  1.7 Hz    80×80  @ 5 cm   lethal 1430  inflated 365
footprint       published, correct shape
all servers     activated
```

Those lethal cells are the room's real obstacles, flowing from nvblox through
`NvbloxCostmapLayer` into nav2.

**Not yet done:**

- **No goal has been set and reached.** The stack is up; the behaviour is
  unknown.
- **No tuning against real driving.** Lookahead, speed and inflation are chosen
  from Phase 1 measurements, not from watching the rover follow a path.
- **Recoveries never exercised.** BackUp and Wait are configured and untried.
- **Frame is `odom`.** Navigation works within one session. Recognising a room
  tomorrow needs Phase 2c/2d.

---

## 7. Commands

```bash
./rover camera && ./rover pose && ./rover fused && ./rover map
./rover nav
```

Then set a goal — in RViz use the **2D Goal Pose** tool, or:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: odom}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}'
```

Watch `/plan` in RViz for the route it computed. `./rover logs nav` for the log.

### First goals to try

Ahead of the rover, a metre or two, with clear space. Then work outward. Given
§4, avoid goals that need a tight turn or sit behind the rover until the pivot
fault is fixed.
