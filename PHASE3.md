# Phase 3 — Autonomous navigation

**Goal: click a point on the map, and the rover plans a route and drives there,
going round obstacles.**

**Status: it drives to goals.** 2026-08-23: goals of 1.00 m and 1.20 m planned
and driven, finishing 4.9 cm and 3.6 cm from target against a 5 cm tolerance, in
6 s and 17 s. A 1 m run was tape-checked — odometry read 95.8 cm, the floor said
93–95.

**What is still untested is the part that matters: nothing has yet been driven
around a deliberate obstacle.** The costmap stops it in theory. See §6.

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
| `planner_server` | route from A to B | **NavFn** with A*, `allow_unknown: true` — §2a |
| `controller_server` | follow the route | **Regulated Pure Pursuit** |
| `behavior_server` | recoveries when stuck | **BackUp and Wait only** — no Spin |
| `bt_navigator` | orchestrates the above | custom behaviour trees, §4 |
| `velocity_smoother` | soften velocity steps | between controller and wheels |

### 2a. `allow_unknown` had to become true, and it is a real trade

It was `false`, on the reasoning that planning through unmapped space drives the
rover blind into ground it has never seen. That reasoning is sound and the
setting was still wrong, for a reason that only appears on hardware:

**a forward-facing camera can never see the floor the rover is standing on.** The
rover's own cell is unknown until something observes it from elsewhere, and NavFn
cannot start a wavefront from an unknown cell. So immediately after a fresh
start, every goal failed in every direction with free floor plainly visible 20 cm
ahead. It could not plan until it moved and could not move until it planned.

What `true` costs: the planner will route through unmapped floor, and unknown is
not the same as empty. What still protects it: the local costmap and the
controller's own collision check run during execution, and nav2 replans as new
ground is observed. Those guard the *driving*. Nothing guards the blind spots —
which is why an autonomous run still needs a human watching.

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

## 4. Everything shaped by how this chassis actually turns

The rover **can** pivot — that was fixed on 2026-08-22 and verified on the floor
at 93% and 96% duty counter-rotating, 65 °/s ([TODO](TODO.md) §14). Every
`NO-PIVOT` adaptation has been reverted. But turning is still the thing that
shapes this file, for three measured reasons.

### There is a deadband: below 0.8 rad/s commanded, nothing moves

Swept 2026-08-23 by commanding `wz` and reading back `/odom`:

| commanded | measured | |
|---|---|---|
| 0.15 | 0.00 | nothing |
| 0.30 | 0.01 | nothing |
| 0.50 | 0.03 | nothing |
| **0.80** | 0.11 | breakaway |
| 1.00 | 0.21 | |
| 2.00 | 0.59 | |
| 3.00 | 0.86 | |

Two faults in one table. Below ~0.8 rad/s the wheels never break skid-steer
scrub and simply sit. Above it, commanded `wz` still runs about **3.4× optimistic**,
because the firmware converts `wz` to wheel speeds using the physical 0.34 m
track while the rover actually rotates about an effective 0.52 m (§13), and
saturates on top.

**This produced a deadlock.** RPP clamps its rotate-to-heading output to
`curr_speed ± max_angular_accel × dt`, and `curr_speed` comes from odometry. From
a standstill the first step was `1.5 × 0.1 = 0.15 rad/s` — inside the deadband —
so the rover never moved, `curr_speed` stayed 0, and the clamp never rose. It
cost 35 s of a 53 s run commanding 0.15 and going nowhere. `max_angular_accel` is
now 10.0, making the first step 1.0 rad/s, clear of breakaway on the first tick.

### A pivot drags the body 10–15 cm

Measured during a final rotate-to-heading with `vx` commanded at zero: the
position wandered over an **11 × 15 cm patch**. Skid steer scrubs all four tyres
sideways and the body goes with them.

`xy_goal_tolerance` was 5 cm — *smaller than that disturbance*, so the two goal
conditions could never hold at once. Every rotation that fixed the heading pushed
the position out; every correction that fixed the position swung the heading. A
1 m goal was reached in 7 s and then circled for 80 more. It is now 10 cm, larger
than the disturbance, so `stateful: true` latches the position check once and the
controller settles the heading without re-opening it.

**The honest consequence: on this chassis you can have position precision OR
heading precision, not both to a few centimetres.**

### Turning is not movement, and the progress checker did not know

`SimpleProgressChecker` demanded 0.5 m of travel within 15 s. With
`allow_reversing: false` a goal behind the rover must be *faced* first, and at
0.21 rad/s a 146° turn takes 12.1 s — nearly the whole allowance spent standing
still, legitimately, before a wheel had carried it anywhere. The goal was aborted
mid-turn, a backup recovery ran, the tree exhausted itself and the rover froze
for 18 s having travelled 2.2 cm. Now **0.25 m within 40 s**.

| setting | value | why |
|---|---|---|
| `use_rotate_to_heading` | **true** | pivots work; without it heading can only be bought with forward motion |
| `rotate_to_heading_angular_vel` | 1.5 | ~0.42 rad/s actual. 2.0 was tried and overshot — RPP predicts its arc from the twist it *commands*, so a faster command overshoots faster |
| `max_angular_accel` | 10.0 | first step clears the 0.8 deadband |
| `xy_goal_tolerance` | 0.10 | larger than the 11 × 15 cm pivot drag |
| `movement_time_allowance` | 40.0 | a slow turn is not a stall |
| `allow_reversing` | false | cuVSLAM under-reads reverse by ~6% (§4) |

---

## 5. Four things that fail silently

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

**Two layers both computing a distance falloff, merged by max.** This one cost
two sessions and a wrong conclusion. Every goal failed with `NO_VALID_PATH` while
the map was demonstrably good, and cells with 0.47 m of clearance read INSCRIBED
while cells with 0.45 m read FREE — separated by a sharp axis-aligned line rather
than a distance contour.

`libnvblox_nav2.so` links `CostmapLayer::updateWithMax`, which takes the per-cell
**maximum** of layer and master grid: it can raise a cost and never lower one. In
gradient mode the nvblox layer writes a graduated cost to every cell within
`max_obstacle_distance` of an obstacle, so those costs accumulated and never
released. A better map produced no better costmap.

`convert_to_binary_costmap: true` fixes it — the layer marks LETHAL only where
the ESDF distance is ≤ 0, FREE elsewhere, and nothing accumulates. The safety
margin was never that layer's job: `inflation_layer` owns it. **Two layers both
computing a falloff was double-counting, and the one that merges by max could not
clear itself.**

The lesson generalises: two wrong hypotheses came from inferring the cost formula
from its outputs. The answer came from `strings` on the plugin `.so` and reading
which nav2 merge function it links against. **When a layer misbehaves, look at
how it MERGES before theorising about what it computes.**

---

## 6. Measured, and what is untested

**Goals actually driven**, 2026-08-23:

| goal | result | time |
|---|---|---|
| 1.00 m ahead | 4.9 cm from target | 6 s |
| 1.20 m ahead | 3.6 cm from target | 17 s |
| 1.00 m ahead (tape-checked) | odometry 95.8 cm, floor 93–95 cm | — |

The costmap agrees with the ESDF it is built from, checked cell by cell:
clearances of 0.45–0.54 m read FREE, 0.10–0.15 m read INSCRIBED, −0.10 m reads
INSCRIBED. Real obstacles block; open floor does not.

**Not yet done, in the order it matters:**

- 🔴 **Nothing has been driven around a deliberate obstacle.** Every goal so far
  has been over clear floor. This is the single most important untested thing:
  the costmap stops the rover *in theory*.
- 🔴 **cuVSLAM diverges silently** ([TODO](TODO.md) §21) — four times. `./rover
  status` now detects it and the tracker rebuilds itself, but the cause is
  unknown, and it was diverged *during* two of the goal runs above.
- 🟠 **Recoveries half-exercised.** BackUp has fired, several times, always on a
  goal that could not be reached. Wait has never run.
- 🟠 **Frame is `odom`.** Navigation works within one session. Recognising a room
  tomorrow needs Phase 2c/2d, and loop closure has never usefully fired (§16).
- 🟡 **Path curvature is tuned for clearance, not directness.** `inflation_radius`
  0.45 and `cost_scaling_factor` 2.0 push routes toward open floor, and a 1 m
  goal bulged 9.5–29.5 cm off the straight line. Those are the same knob: "keep
  away from obstacles" and "go straight" pull opposite ways.

---

## 7. Commands

```bash
./rover camera && ./rover pose && ./rover fused && ./rover map
./rover nav
```

Then set a goal. In RViz use the **2D Goal Pose** tool, or — better, because it
checks first and explains itself:

```bash
docker exec -it rover bash -lc 'source /opt/ros/jazzy/setup.bash; \
  export ROS_DOMAIN_ID=0; python3 -u /logs/goto.py 2.0 1.5 90'
```

`goto.py x y [theta] [--rel]`. θ in degrees, absolute against the map (0 = +x,
90 = +y) or a turn from the current heading with `--rel`.

It refuses to drive when the goal is unreachable — naming the nearest point that
*is* — when the route touches a blocked cell, or when anything else owns
`/cmd_vel`. Each of those was learned the expensive way:

| refusal | what happened without it |
|---|---|
| goal unreachable | 52 s driving at a goal inside an obstacle, then giving up 42 cm short |
| `/cmd_vel` contended | teleop in MANUAL streams zeros at 10 Hz that cancel nav2's commands; the rover stalls and it looks like a controller fault |

### Picking a goal that exists

Guessing coordinates off a map produced three consecutive refusals on goals
inside obstacles. Ask the costmap instead:

```bash
docker exec -it rover bash -lc 'source /opt/ros/jazzy/setup.bash; \
  export ROS_DOMAIN_ID=0; python3 -u /logs/where.py'
```

It lists the reachable points around the rover and the turn each one needs.

### Before any autonomous motion

- The teleop must be in **AUTO**. Tapping **MANUAL** cancels the goal — that is
  the stop button.
- Check `./rover status` says `ok — tracking, not dead reckoning`. Two of the
  goal runs above happened while cuVSLAM was diverged and nothing said so.
