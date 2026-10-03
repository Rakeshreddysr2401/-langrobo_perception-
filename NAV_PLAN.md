# NAV_PLAN — paths the rover's body actually fits, and a rover that gets itself out

Owner's goal (2026-09-27): *"always take the best path, one that fits it —
a long-term solution, managing itself, production grade."*

## Why tuning stopped working

The same failure was hit three times on 2026-09-27 and "fixed" by a
parameter each time (padding 5 → 2 cm, inflation 0.45 → 0.80,
failure_tolerance 0.3 → 2.0). It came back the same evening, measured from the
Pi 5 with the rover parked where an RViz goal had stalled:

| what | measured |
|---|---|
| lidar, nearest per side | 0.32-0.43 m both sides and behind; 2 m open ahead |
| nvblox lethal cells inside the padded footprint | **2** — the rover "is inside an obstacle" |
| nav2 log | RPP `collision ahead` every 0.1 s → Spin `Collision Ahead` in 0.1 s → BackUp same → Wait 5 s → again |
| nav2 | segfaulted (exit -11) once that session; core dump lost to apport |

The obvious way out (creep forward, then turn left) is one no part of the
current stack can produce:

1. **NavFn plans for a point**, not a 36 × 38 cm box. Its paths ignore heading,
   so a path can begin with a turn the body has no room to make.
2. **RPP pivots first** whenever the path heading is > 34° off. A pivot sweeps
   the 0.29 m circumscribed circle; with walls at 0.26-0.27 m it is refused,
   and RPP has no second idea.
3. **The recoveries are blind**: Spin (refused, same reason), BackUp (into the
   side the camera cannot see), Wait. None of them looks at where the space is.
4. **Nothing clears what is under the rover.** The camera cannot see within
   ~30 cm of the nose, and nvblox's layer has no footprint clearing, so a mark
   left on the way in stays inside the body and every check starts "in
   collision".

This is a design gap, not a tuning gap. The plan replaces the parts, one
measured step at a time, and keeps what works (fusion2, turn_shaper,
goal_exec/reach for the exact finish).

## The target stack

| role | now | target | why |
|---|---|---|---|
| planner | NavFn (point) | **Smac State Lattice**, diff-drive primitives at 2.5 cm, in-place turns allowed | plans in (x, y, θ) with the real footprint; "forward then turn" is a native move |
| controller | RPP | **MPPI** (DiffDrive), footprint cost critic | samples ~1000 short forward/turn trajectories a tick against the real outline and picks one that fits |
| under the body | never cleared | **footprint clearing for nvblox** | the rover cannot be inside an obstacle; a mark there is stale by definition |
| recovery | Spin → BackUp → Spin → Wait | **escape toward measured free space** (lidar 360°), then Spin, BackUp only if the lidar sees the rear clear | stops recovering into walls and into the blind side |
| stuck | silent loop | **stuck report** → `[SYSTEM]` turn to the Pi 5 brain ("I'm boxed in; the way out is ahead-left") | CLAUDE.md rule 6 mechanism; the owner hears it instead of watching a loop |
| crashes | restarted, cause unknown | **core dumps kept + backtrace** on every nav2 death | segfaults cannot be production-grade until one has a stack trace |

All four plugins are already in the image (`libnav2_smac_planner_lattice.so`,
`libmppi_controller.so`, `nav2_rotation_shim_controller`, graceful). No image
change is needed.

## Phases — each ends with a number, on the floor, with the owner watching

**N0 — Baseline (measure before changing anything).**
A fixed course of ~8 RViz-style goals in the house, including today's pocket,
a doorway, a goal behind the rover, and a long open run.
`phase3/tools/nav_course.py` sends each goal and records: time to goal,
recoveries, `collision ahead` count, closest body-to-obstacle clearance
(footprint vs lidar), final error, nav2 CPU. One CSV per run, graded like
`./rover grade`.
*Done:* baseline numbers for today's stack, committed.

**N1 — Stop the false "I am inside something", and catch the crashes.**
- Clear nvblox inside the footprint: prefer `/nvblox_node/shapes_to_clear`
  (the topic exists; check the message type) at ~2 Hz with the unpadded box;
  otherwise a small costmap layer that clears the footprint in the nvblox
  layer's grid.
- nav2 core dumps to a file (`core_pattern` → a path; `ulimit -c` is already
  unlimited), plus a gdb backtrace script in `nav2_supervise.sh`.
*Done:* the pocket from today plans and drives out with no recovery; one
crash, when it happens, leaves a backtrace.

**N2 — A planner that knows the body.**
Smac State Lattice. Generate diff-drive primitives at **2.5 cm** (only 5 cm
ships), with Nav2's own `generate_motion_primitives.py` (pure Python, runs on
the Pi 5 or laptop; output JSON goes in `phase3/config/`). Footprint collision
checking, in-place rotation primitives on, reverse penalised (blind rear),
`allow_unknown` kept for the unseen start cell.
Check the planning time on a 12 m, 2.5 cm costmap. If it's over ~0.5 s, the
global costmap goes back to 5 cm (the local stays at 2.5).
*Done:* on the N0 course every planned path passes an offline footprint sweep
(no pose overlaps lethal), and the pocket's path starts forward.

**N3 — A controller that tries alternatives.**
MPPI, DiffDrive model; critics: footprint cost (`consider_footprint: true`),
path align/follow, goal angle, prefer-forward. Its `vx`/`wz` limits are REAL
rates (turn_shaper stays between the smoother and the wheels and still lifts
the 0.8 rad/s deadband).
**CPU is the risk:** the Jetson is at load ~7.5 on 6 cores (OPEN_ISSUES #1).
Budget ≤ 1 core. Start at batch 1000, 40 steps, 20 Hz. If it cannot hold 20 Hz,
fix OPEN_ISSUES #1 first or fall back to RotationShim + RPP.
*Done:* on the N0 course, time-to-goal and recoveries both clearly better than
baseline (target: zero recoveries in open space, ≤ 1 in the pocket), and 20 Hz
held.

**N4 — Recoveries that look, and a rover that says so.**
- New BT order: clear local → **escape** (a small behaviour: pick the
  lidar sector with the most clearance, drive ≤ 20 cm toward it by goal_exec)
  → Spin → BackUp *only* if the lidar sees ≥ 40 cm behind → Wait.
- Stuck detector (no progress N s, or recovery ≥ 2): publish a reason on
  `/nav/stuck`; the Pi 5 injects it as a `[SYSTEM]` turn so Mitra says what is
  wrong and what it will try.
- `fleet.sh check` gains: nav2 restarts this session, last stuck reason.
*Done:* boxed in on purpose (3 sides), it gets out or says why, every time.

**N5 — Stay good: regression.**
The N0 course is the release gate for every nav2.yaml change: re-run and
compare CSVs before committing. Optional: the same config on rover_sim for
planner/controller regressions without the robot (its map frame and
TwistStamped differ — the frame is the one to watch).
*Done:* the release gate is written down in OPERATIONS.md and has been used.

## Production-grade means (the exit bar)

- 20/20 course goals reached, ≥ 3 cm real clearance throughout.
- Zero recoveries in open floor; ≤ 1 in pockets; never a blind BackUp.
- No nav2 crash in 1 h of mixed goals, and any crash leaves a backtrace.
- Jetson CPU with nav running leaves ≥ 20% idle.
- Every stuck state is spoken, with its reason.

## Order and dependencies

N0 → N1 (cheap, removes today's failure) → N2 → N3 (needs CPU headroom:
OPEN_ISSUES #1 may have to move up) → N4 → N5. Each phase is one commit with
its CSV. nav2.yaml keeps its convention: every changed value says why, when,
and what was measured.

## Status — 2026-09-27, 23:10 (live since 23:15; see the session sections below)

| step | state |
|---|---|
| N0 planning baseline | **done** — `plan_check.py` on NavFn from the pocket: **3/16** goals planned, **1 path with the body inside lethal** (−2.5 cm), every path's first 20 cm turns 51°. `logs/plan_check.csv` |
| N1 hard margin | nav2 padding, goal_exec/reach MARGIN, wall_margin_view: 2 → **1 cm**; clearance now soft (lattice `cost_penalty`, MPPI `CostCritic`) |
| N1 crash traces | libbackward preloaded into every nav2 node; `nav2_supervise.sh` saves each trace to `logs/nav_crashes/` |
| N2 planner | **Smac State Lattice**, `config/lattice_diff_2p5cm_r0p4.json` (104 primitives, generated with nav2 jazzy's generator) |
| N3 controller | **MPPI** DiffDrive, 1000 × 40 @ 20 Hz, footprint CostCritic, forward only |
| reach_node | knows MPPI's and the lattice's failure messages |
| N0 drive course, N4 escape + spoken stuck report, N5 gate | not started — need the rover driving with the owner watching |

To go live: `./rover nav` (restarts nav2 + reach + goal_exec; moves nothing), then
`python3 /opt/rover3/tools/plan_check.py --csv /logs/plan_check.csv` from the same
spot and compare with the NavFn row above. Roll back (NOT `git checkout`: that
would also drop today's earlier uncommitted nav2.yaml work):
`tar xzf ~/rover_backups/pre-navplan-20260927-2247.tgz ./phase3/config/nav2.yaml ./phase3/launch ./phase3/nodes && ./rover nav`
(the backup holds today's uncommitted work).

## Next phases (owner, 2026-09-28) — in this order

**N6 — Finish the drive-test fixes (2026-09-27 night).**
- *Footprint-clearing layer* on the GLOBAL costmap (small C++ costmap plugin,
  built inside the image): cells under the rover's own outline are cleared
  after nvblox + LiDAR, before inflation. The rover is physically there, so a
  mark there is stale by definition. Removes "Start occupied" by
  construction (seen 6 times that night; the escape only recovers from it).
- *Goal snapping* in reach: a goal on or against furniture moves to the
  nearest pose where the outline fits (same heading), and says so. Every
  "no path" on the long goals that night was a goal the costmap marked solid.
- *Map honesty*: stale nvblox marks after the rover is carried or people
  move (OPEN_ISSUES #5) — `ghost_check.py` against what the owner sees;
  `./rover map` clears nvblox's memory meanwhile.
*Done:* long goals across open floor (≥ 2 m) reach first time; no "Start
occupied" in a session.

**N7 — Speed governor: fast where open, slow where tight.**
A node between the velocity smoother and turn_shaper that caps vx every tick by:
clearance ahead (LiDAR + depth), path curvature, SEEN distance ahead (never
faster than it can stop inside what the camera has observed), and fusion2's
pose health. vx_max rises toward ~0.5 m/s in open floor (wheels top ~0.86).
*Done:* the N0 course faster than MPPI at 0.25 flat, never faster near
obstacles, stopping distance always inside sensor range.

**N8 — Build and go: planning into unknown floor.**
Already true: allow_unknown, nvblox maps while driving, replans at 1 Hz. Add:
look before committing (face the camera onto unseen floor at a turn — it sees
87°), a small price on unknown vs known-free cells, speed tied to seen
distance (N7), and for goals in unmapped space: drive to the mapped edge,
look, replan, repeat — instead of "no path".
*Done:* a goal in a room it has not seen is reached step by step.

**N9 — Motor effort (ESP32 firmware).**
Breakaway feedforward at pivot start (the 0.8 rad/s scrub deadband), gentler
straight-line ramps (less slip → better pose), battery-voltage compensation
(the turn slide moves with the battery, OPEN_ISSUES #12).
*Done:* in-place turns start without the deadband; the ~30 cm per 90° pivot
slide shrinks. Measured with `./rover record` / `./rover grade`.

## Drive test 2026-09-27 night — results (Lattice + MPPI + escape)

| goal | result |
|---|---|
| 0.8 m out of the pocket | reached 0.6 cm, 14 s (RPP/NavFn: 8 attempts, 341 s, failed) |
| 1.6 m through clutter | reached 1.2 cm, 47 s, 51 MPPI "Optimizer fail" ticks |
| 1.24 m, tight | reached 0.8 cm, 79 s, escape + retries |
| 2.5-3.7 m across the room | no path: goal marked solid / no exit in the costmap |

Fixed during it: reach started mid-restart never got /odom and dropped goals
silently (now fails after 3 s); failure classifier reports the root cause;
escape (straight only, closed on fresh points); local padding 0.02 vs global
0.01 so MPPI never parks where the planner refuses to start; clearance
weights (CostCritic 6.0, PathAlign 8.0, lattice cost_penalty 3.0). Controller
held 20 Hz throughout; no nav2 crash after the change.

## Session 2026-09-28 (after midnight) — N6 built and drive-tested

### What was built
| piece | where | what it does |
|---|---|---|
| **footprint_clear** costmap layer (C++) | `phase3/plugins/` (`build.sh`, built inside the image like `lidar/build.sh`) | GLOBAL costmap only, after nvblox + LiDAR, before inflation: cells under the rover's own outline are FREE. Swept ±11.25° (the lattice checks its start at the nearest of 16 headings) + 2 cm. The local costmap MPPI checks against keeps every mark. |
| **goal snapping** | `reach_node.py` `fit()` | A goal whose outline would touch something moves to the nearest pose (same heading, ≤ 50 cm) with 3 cm of real clearance, using the planner's own costmap; reach reports `phase: snap`. Unseen floor is left alone (N8). |
| **escape, straight only** | `reach_node.py` `escape()` / `drive_straight()` | Closed-loop 5 cm/s straight move, fresh LiDAR + depth + local costmap every 50 ms, stops if the gap shrinks. No longer uses goal_exec (its docking turned 30° first). Points INSIDE the body are ignored (floor bumps already driven over). |
| **reach cancels on shutdown** | `reach_node.py` main | SIGTERM (pkill, `./rover nav`) cancels its nav2 goal and any goal_exec move, then stops. Tested 5/5. Before: a restart mid-goal left nav2 driving for 20 s. |
| `.gitignore` | repo root | plugin build/install/log ignored; src tracked |

### What improved (measured on the floor)
| | before (session 1) | now |
|---|---|---|
| "Start occupied" | 6× per session; goals died in the pocket | **0** after the heading-sweep fix; the pocket start now gets a real answer from the planner (208 no path, not 205 start occupied) |
| 2.6 m through a gap between two obstacles | — | **reached 0.8 cm, first attempt, 39 s, no retries** |
| click touching an obstacle | "no path" × 8 | **snapped 46 cm / 8 cm** to a pose that fits, then drove |
| escape on floor noise | backed up 5 cm twice for nothing | ignores points inside the body; reads the true 33 cm |
| nav2 crashes | ~5 in two days, no trace | **0** since the change (libbackward ready if one happens) |

### What we learned
- **The main remaining blocker is the depth camera marking floor as obstacles.**
  Map probe (goal 2 area): **12** LiDAR obstacle cells vs **270 camera-only**
  cells, in a room where the owner says the rover can pass. `ghost_check.py`:
  73% of lethal cells ahead were camera-only; a fresh `./rover map` removed
  ~60% of them (stale memory), the rest came back (really seen, or floor
  noise at range). Two real obstacles confirmed by the owner (left and right,
  ~0.5 m, with a passable gap between).
- `./rover camera --floor` needs clear flat floor in front; from the pocket it
  gave a weak fit (16% on plane) — not yet measured.
- nav2 bringup can hang on a lost lifecycle service reply under load
  (controller configured, manager never heard): one more `./rover nav` fixed it.
- After ANY restart of reach, check it has a pose (it once never got /odom).

### TODO — next session, in order
1. **Camera tilt**: face the rover at 1.5-2 m of clear bare floor and run
   `./rover camera --floor`. Drifted > ~0.5° → re-save (the knock when it was
   lifted is a candidate). This is the cheapest fix for phantom floor marks.
2. **nvblox floor band vs range**: if the tilt is fine, make the obstacle
   floor range-dependent the way `depth_obstacles.py` is (2 cm < 1 m,
   3.5 cm to 1.6 m, 5 cm beyond) or cut `max_integration_distance` where the
   floor stops measuring clean; re-run the 12-vs-270 probe and `ghost_check.py`.
3. **"Look again before giving up"** in reach: on "no path" / no snap, face the
   camera at the blocked area so nvblox clears what it can see past (no timed
   forgetting — the repo's rule).
4. Re-drive the session's goals: pocket → far side (goal 2), through the gap,
   click-on-obstacle. Exit bar: long goals reach first time, 0 Start occupied.
5. Then **N7 speed governor**, **N8 build-and-go**, **N9 motor effort**.

## Session 2026-10-03 (night) — "go near the door" in a full room: why it failed

Owner: the floor now has many objects; the rover takes bad ways, tries blocked
areas when an open way exists, curves poorly. Wants settings fitted to the
real objects' measurements. Rover cancelled by `/reach/cancel` after this was read.

**What happened (logs: pixel_to_goal, reach, goal_exec):**

| step | measured |
|---|---|
| VLM box for "door" | `[86, 0, 314, 325]` (left third of a 896 x 504 frame, top to mid) |
| pixel_to_goal | `depth 0.80 m ... 86 points above the floor, nearest slab` → the goal was the **nearest thing in the box**, i.e. the furniture in front, not the door |
| reach snap | `the goal as clicked touches something; nearest pose that fits`, moved 10 cm, gap 3.2 cm |
| attempts | 8 attempts in 270 s: goal_exec "turn stuck", "obstacle 0.33 m into the leg", "gap too tight 14.9 cm (needs 42)"; nav2 "no path: planner found none" ×5; escapes of 5-10 cm |
| meanwhile | a fresh frame showed the door on the right with **open floor straight to it** |

**Causes, most damaging first:**

1. **Target choice (decision, not nav).** `nearest_slab()` assumes "an object
   stands in front of its background". True for a bottle, false for a door,
   wall, or anything seen *behind* clutter: the box always contains the
   foreground, and its nearest dense slab (`CLUSTER_MIN` points) wins. The goal
   lands inside the clutter and every later layer fights a goal that cannot
   be reached.
2. **Snap ignores reachability.** "Nearest pose that fits" is the nearest free
   pose to the goal, not one connected to where the rover is. A pocket
   between chair legs "fits" and is unreachable.
3. **No early "this goal is unreachable".** 8 attempts, 270 s, on the same
   point; nothing tells the brain "the point you picked is boxed in, there is
   open floor to the right" so it could re-look or re-choose.
4. **Path quality in clutter** (the owner's "curves", "goes into blocked
   areas"): global inflation 0.80 m at cost_scaling 2.0 makes every gap
   narrower than ~1.6 m expensive, so routes bend widely; unknown cells are
   allowed (`allow_unknown: true`), so the planner may prefer an unseen
   (actually blocked) way over a seen open one. Needs measuring, not guessing.

**Proposed order (each measured on the floor with the owner):**

| # | change | exit number |
|---|---|---|
| D1 | pixel_to_goal: for targets the VLM calls *large/background* (door, wall, cupboard, room) take the **dominant far slab** of the box (or the box's bottom-centre floor contact), not the nearest; reply which one it used | the door goal lands within 30 cm of the real door, from 3 positions |
| D2 | reach snap = nearest pose **reachable from the rover** (flood fill on the padded-footprint costmap), standoff in front of the target | 0 goals placed in a pocket |
| D3 | reach gives up after 2 failed tries with a reason + the free direction, as a `[SYSTEM]` turn to the brain | brain re-looks instead of 8 retries |
| D4 | measure the room (door width, chair-leg gaps, cot, desk) and set inflation / cost scaling / unknown cost from those numbers; N0 course re-run | course: time, recoveries, closest clearance vs the 2026-09-27 baseline |

### D1 — built and checked live, rover not moving (2026-10-03, later)

`pixel_to_goal.object_slab()`: the nearest slab holding >= 8 % of the box's
above-floor points is the object, else the largest. Replies/logs now carry
`slab`, `share`, `skipped` and the top-3 `candidates` (range, points).
Tests: `phase4/tools/test_object_slab.py` (6, incl. the door-behind-a-leg
case the old rule fails). Live check, never drives:
`docker exec rover ... python3 /opt/rover4/tools/ground_check.py "the door"`.

| asked for | chosen | share | truth |
|---|---|---|---|
| black office chair | 0.92 m | 15 % | the chair ✓ (at 25 % it would take the wall behind, 1.97 m ✗) |
| woven cot | 1.42 m | 8 % (passed over 163 pts at 1.18 m) | the cot ✓ (it spans 1.38-1.58) |
| the door (VLM box tight on it) | 1.77 m | 17-19 % | the wall/pillar edge beside the door; LiDAR: pillar 1.95 m, door 2.47-2.60 m from base_link, so ~0.6 m short, but on open floor in front of the door, not in clutter |

So 8 % stays: it is what keeps open-frame objects (chairs) right. The door's
leftover error is the box catching the door frame, and an angled door spreading
over two 10 cm slabs (2.42 m: 412 pts + 2.52 m: 1440 pts). Next for big targets: the
brain says "large" (door/wall/cupboard) and pixel_to_goal merges adjacent
slabs, then takes the largest. Left for after D2/D3, because the 0.6 m now
lands on reachable floor.

### Drive tests to the door, and what they found (2026-10-03, late)

| run | target | drive | why |
|---|---|---|---|
| 1 (D1 v1) | door frame edge, 1.53 m | 0.49 m gained, then "obstacle 0.29 m into the leg", "no path"; cancelled | **red on empty floor**: the camera mount had tipped 0.49 -> 3.02 deg nose-down; ghost_check 1913 camera-only cells 0.85-1.85 m ahead. Re-calibrated (add9474): 497, open floor clean |
| 2 (camera fixed) | **the white pillar beside the door**, 1.61 m | **first try, finished 6.7 cm from the goal**; brain: "I have arrived at the door" | the box clipped the pillar at its edge; the door was 0.8 m behind it |

**D1 v2, centre rule:** the object must also hold >= 25 % of the points in the middle
half of the box's width (the VLM centres its box on what it means; the pillar and
the frame sat at the edges). Tests 10/10 incl. both live cases. From in front of
the door, the pick moved from the frame (0.75 m, 0.3 m in front of the door) to
**the door's plane at its right end** (LiDAR: leaf 0.89-1.13 m across bearing
+8..-25 deg; pick 1.14 m ahead, 0.55 m right).

**Still open: dark, plain surfaces have almost no depth.** The door is near-black
and the D555 runs with its IR projector OFF (load-bearing for cuVSLAM, phase1/README),
so stereo finds little texture on it: the leaf gave few points and the pick fell on
its lighter edge. Next: for structural targets (door, wall, cupboard) ground on the
**LiDAR**, which sees them at 25 cm: the scan points across the box's bearing range,
nearest continuous segment, its middle. The LiDAR is the pose reference already and
is not fooled by colour or gloss.
