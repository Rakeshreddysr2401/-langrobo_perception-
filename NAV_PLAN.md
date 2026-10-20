# NAV_PLAN — paths the rover's body actually fits, and a rover that gets itself out

Owner's goal (2026-09-27): *"always take the best path, one that fits it —
a long-term solution, managing itself, production grade."*

**The working method since 2026-10-19** -- observe, record, measure, root cause with a number,
reproduce in the twin, one change, keep only if measured better, commit with the evidence,
document, real robot, repeat: `~/kino_sim/docs/IMPROVEMENT_LOOP.md`.

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
  `/nav/stuck`; the Pi 5 injects it as a `[SYSTEM]` turn so Kino says what is
  wrong and what it will try.
- `fleet.sh check` gains: nav2 restarts this session, last stuck reason.
*Done:* boxed in on purpose (3 sides), it gets out or says why, every time.

**N5 — Stay good: regression.**
The N0 course is the release gate for every nav2.yaml change: re-run and
compare CSVs before committing. Without the robot: the twin runs this same
nav2.yaml on simulated hardware (kino_sim `./sim scenario <world>`, or
`tools/run_suite.sh` for every world), judged on the true pose.
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
   forgetting — the repo's rule). **Done 2026-10-04** as the way back
   (`trail.py`, 3b15b5e: look down the route it drove, retrace it in nav2 hops)
   and `ghost_clear` (0efdf58: the LiDAR erases marks of things that moved).
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

### G1 — LiDAR grounding for big flat things (2026-10-03, night)

Brain sends `;what=<description>` (Pi 5 72e0bae); `pixel_to_goal` (9fda655 + this)
measures door / wall / cupboard / wardrobe / fridge / sofa with the LiDAR: the
nearest continuous surface across the middle half of the box's bearing, from the
scan nearest the photo, converted with the TF at the photo's stamp. (Converting at
receipt dropped every scan: 34 ms after it, odom -> base_link for its stamp is not
in yet.) Everything else stays on the camera. Live, under the desk: "the wall" ->
LiDAR 1.24 m (camera 1.08 m = the bin and legs in front; thin legs at 0.44 m
passed over); "the white bin" -> camera 0.42 m. Door drive test next, with fresh
batteries (the earlier search turns stalled at 0 deg on the old ones).

### "It can rotate, why does it block?" -- start occupied beside a chest (2026-10-03, ~22:00)

Goal 2.23 m away; reach gave up after 8 attempts in 64 s, every one "start
occupied". Measured at the spot (`/goal_exec/obstacles`, saved as
`phase3/tools/fixture_chest_right_2026-10-03.npy`):

| | |
|---|---|
| around the body | chest of drawers 4.9 cm off the right side; left open; 25 cm ahead, 22 cm behind |
| LiDAR / camera layers under the body | 0 / 0 cells -- the "touching" is the planner's 4 cm padding (0.04 since the S-gap test) plus 2.5 cm cell rounding |
| straight moves | all close in (that is all escape tried) |
| turn left 90 in place | sweep 1.2 cm, ends 6.1 cm off -- fits; right 90: 0.9 / 5.7 |

Two faults in reach, both from the padding going 0.01 -> 0.04 today:
`ESCAPE_NEAR` was still 3 cm (so 4.9 cm was "not pinned" although the planner
refused it), and escape knew only straight moves. Now: pinned below 5 cm
(padding + 1 cm); when no straight move opens space, the turn in place
(+-30/60/90) whose sweep keeps goal_exec's 1 cm and whose end opens the most
space, ties toward the goal, done by goal_exec (it re-checks the swing about
its learned pivot); and after escaping a refused START, plan again at once.
Tests: `phase3/tools/test_escape.py` (real points: turns left; a wall the whole
length of the side or a corridor: no turn).

**Removed (owner, same night):** the reach escape changes made for this --
turn-in-place escape, 3-point turns, ESCAPE_NEAR 0.05, the re-plan after a
refused start, the per-turn logging and `test_escape.py` with its two fixtures
(commits 2b0c9d8, d3e3549, d631ace, 20ee5ac, reverted). reach is back to its
straight-only escape at 3 cm. The owner will pick the approach. Facts kept for
it: beside the chest, reach's own points (LiDAR + depth memory, 29,828) put
the body 3.7 cm off; turns in place swept 0.1-1.2 cm; a 3-point turn (left 45,
forward 10 cm, left 60) ended 10.8 cm clear on those points; MPPI's costmap
pads 0.02 while the planner's pads 0.04 (unchanged).

## Session 2026-10-10 (evening) — the bedroom door: turn and go (tight route)

Owner: "go inside that door and go outside", then from the hall "come to me".
Both failed with room to spare; owner: "it can take left rotation and move".

| | measured (no motion) |
|---|---|
| the gap past the open door leaf | 77 cm (rover 38); leaf 30 cm ahead; rover's left side 5 cm from a wall corner, so a left turn in place stops at ~25 deg |
| nav2 ComputePathToPose, every goal through the gap | error 208 NO_VALID_PATH; at footprint_padding 0.01 (set live, put back to 0.04) only a 5.3 m loop -- the lattice turns on 0.4 m and pays 5x to turn in place |
| what closed it in the costmap | one camera-only 3-cell speck on the glossy floor (owner: nothing there); later, from the hall, the camera map drew the dark leaf 10-15 cm thicker than the LiDAR's line |
| "come to me" from the hall | VLM found the owner 3 m away through the door; reach: nav2 "no path" x3, two way-back hops, failed after 188 s with no move toward them |

A search over goal_exec's own moves found **F10 L45 F160** in 0.4 s; driven
through `/goal_exec` one step at a time (owner watching): every step reached
(0.4-1.1 cm), out of the door. The speck was gone from goal_exec's view once
the first turn put it in front of the camera.

**Built** (`rover-v1.1.10-tight-route`): `phase3/nodes/tight_route.py` -- A* over
turns (15/30/45/90) and straight drives (5 cm-1 m, reverse <= 10 cm, x3 cost),
each checked with goal_exec's own turn_clear / drive_clear (learned pivot) at
2 cm, one more than goal_exec drives at. reach: on "no path" / narrow / stuck,
`tight_round()` runs BEFORE the way back: plan on LiDAR + camera map; if none,
plan on the LiDAR alone, which only picks the direction (drives cut to 50 cm,
goal_exec still refuses any move touching a camera point); one move, 1.5 s
still, plan again; 2 refusals, 10 moves or the goal's MAX_S end it.

Tests: `phase3/tools/test_tight_route.py` (8: both door fixtures, side doorway
F115 L90 F105, 44 cm slot through / 40 refused, a wall, checks identical to
goal_exec's over 2000 random cases), `test_reach_tight.py` (4: phantom speck
-> out of the door; real speck -> stops before it, never inside; deadline;
cancel). Jetson: slowest plan 1.9 s (budget 3).

**Open:** floor test of "come to me" through the door with reach's tight
round; the camera map's thick dark door (from the hall) may still make the
LiDAR-only plan the one used; the brain should aim "go through the door"
past the opening, not at the leaf (Pi 5).

## Session 2026-10-19 (twin, afternoon) — across the house without touching anything

Owner: talk to KINO in the twin, find why "go near the sofa" and cross-room goals
fail, fix one thing at a time, then the real robot. All of it on the twin
(kino_sim apt_s1: living room with a 55-60 cm sofa / coffee-table squeeze, 90 cm
corridor, bedroom and kitchen doors, clutter at 3 heights), judged on the TRUE pose.
A fixed 5-leg loop was driven before and after each change
(`./sim scenario apt_s1 --goals "0,-1.2,0;4.55,0.61,0;5.19,-2.12,3.142;6.59,2.48,1.571;0,-1.2,0"`).

### What was wrong, the evidence, and the fix

| # | what was wrong | evidence | fix (commit) |
|---|---|---|---|
| 1 | reach **crashed** on the escape's "drive + turn" move | TypeError on `math.degrees(tuple)`; reach dead, goal hung 300 s, later goals unanswered | all 3 move kinds driven; any error in one goal -> `failed: internal error`, reach keeps serving (d450db2) |
| 2 | tight route **ping-pong** | plan "B5 L30 F85" -> B5, re-plan "F5 L30 F85" -> F5, x3 | a <= 10 cm set-up drive runs with the move it sets up (40364d2) |
| 3 | LiDAR **blind within 2 cm of the body** | reach believed 5.7 cm, truth 0.4 cm; the escape turned into the coffee table (-3.3 cm). 21299 real scans: nothing of the rover in the plane | points dropped only INSIDE the outline + 5 mm (gap_pass, goal_exec_node, follow_node); costmap LiDAR min range 0.20 -> 0.05 (d5fe105) |
| 4 | **drove under** a 28 cm tabletop | obstacle band stopped at 27 cm, rover 26.3 cm; mesh check: nothing of the table at 6-26.3 cm inside the outline; wheels jammed on its base | nvblox slice + depth_obstacles to 31 cm (9453551) |
| 5 | house route and nav2 **disagreed** | route straight through the living-room clutter (LiDAR map, blind to ball / bin / bottle); nav2 went round the table; legs pulled the rover back and forth | route uses nav2's obstacles AND costs (87d7533) |
| 6 | **pinned** along a whole side | 1.2 cm along the table: no short move opens space, MPPI refuses all (2 cm margin) -> 3 goals with 0.0 m driven; nav2's own retries then ran 120 s | escape **steering slide** on raw points; **pinned watch** (still 10 s + something inside 2 cm -> escape now). MPPI padding 0.01 tried and REVERTED (grazed the sofa 1.4 cm) (a285a82) |
| 7 | plans **flip-flopped** | BT replanned every second and took any plan; the rover turned back | keep the path unless blocked / goal changed / 10 s, as nav2 ships it (f2f68e7) |

### Measured on the loop (time per leg; slipper = 3 cm, below every sensor)

| run | bedroom -> living | living -> corridor | corridor -> bedroom | bedroom -> kitchen | kitchen -> living | furniture touched |
|---|---|---|---|---|---|---|
| before | **failed** (16 cm driven) | 196 s | 31 s | 99 s | -- | none, but 0.6-1.7 cm close |
| house route on, nothing else | 288 s | timeout | 151 s | 91 s | 245 s | **4 of 5 legs** |
| + fixes 1-4, 6 | -- | 195 s | 32 s | 92 s | timeout | kitchen bin 0.8 cm |
| **+ 5, 7, pinned watch** | (start) | **108 s** | **30 s** | **88 s** | **174 s** | **none** |

### Pose drift (kino_sim `./sim odomprobe`)

Fused /odom stayed 1-6 cm from the truth over 11-39 m from a clean start (brief
5-30 cm spikes when the LiDAR odometry slips in fast turns). The 10-36 cm seen
earlier came after contacts. The twin's cuVSLAM is 3-33 m off (a twin bug; the
real graded VO is 0.03-10 cm), so the twin is harder on the fusion than the robot.

### Open, in order

1. **Real robot** (owner present): the loop's kind of goals at home, tight spots
   first; `REACH_HOUSE_ROUTE` stays OFF on the real robot until then.
2. MPPI and the collision monitor model a turn about base_link with no slide;
   this skid-steer slides 6-16 cm per turn -> grazes in turns near walls.
3. Small low things: a 7 cm toy block driven through, the 3 cm slipper unseen.
4. Speed: legs still 1.5-3 min.

### The same evening on the REAL robot (owner at RViz)

After a power cycle; SLAM resumed the 2026-10-18 house map (lock checked:
78% of live LiDAR points within 10 cm of its walls, median 5 cm). Recorded:
`logs/bags/20261019-174749_rviz_real` (+ `_rviz_real2`, `_rviz_real3`).

| goal | before the evening's fixes | after |
|---|---|---|
| 1-3, same room, 1-2 m | reached: 0.7 cm / 0.8 cm / 9.5 cm (9.5: after a nav2 drive the finish is turn-only, by design) | -- |
| > 3 m, other room (4 goals) | failed "no path ... 3 times" (straight-line legs into furniture; 98-343 s) | house route ON (19b2026) |
| into the bedroom through a 49-60 cm door past the bed's foot | failed: the tight route needed 2.9-5.6 s on the Orin, had 3 s | TIGHT_BUDGET_S 8 (b01b871): **through** in 8 precise moves (~1 cm each) |
| back out through that door (> 3.2 m) | failed in 41 s, no move: legs found no point nav2 can reach and only waited | legs failure -> tight route aimed along the house route (22c7423): **through** (L15 B25 F100 F100), then nav2 |
| a goal 16 cm inside furniture | -- | refused honestly: "the goal is inside something ... no spot within 50 cm fits" |

How the door was diagnosed (all from the recording): single LiDAR scans put
the gap at 48-60 cm (the 27 cm of the first, merged picture was smearing);
nav2's costmap was right (inscribed band = body + 4 cm pad leaves a 5 cm
channel for the centre; its 16-heading lattice cannot use it at that angle);
the tight route, replayed offline on the recorded points of that moment,
found the way every time -- it only lacked time on the Orin.

Also found:
- **The Orin was saturated**: load 24 on 6 cores, 157 MB RAM free, 726 MB
  swapped. `detections_3d.py` (follow layer) used 58% CPU and 1.1 GB while
  publishing NOTHING (bring-up: "detections_3d publishing 0 FAIL"); stopped
  for the session (it comes back with `./rover up`). The bag recording with
  the 30 Hz depth cloud added 42%: record without `/depth_gate/passed`.
- In the doorway the camera map held something ~32 cm ahead (two goal_exec
  refusals, ~45 s of re-planning): the door leaf's edge or the threshold --
  to look at in `_rviz_real3`.
- After a nav2 drive the finish is turn-only (FINISH_NEAR 10 cm): up to
  ~10 cm off. Proposed: a straight nudge along the heading after the turn
  (twin first).

### Later the same evening: front sonars, the chairs, tight-route speed

**Chairs (owner's layout, measured from the robot):** the widest way between a
splayed plastic chair and a towel hanging to 6 cm was 52 cm for the LiDAR but
**42 cm** with the camera (the chair's feet, the towel's hem, a stool's legs
are under the LiDAR's plane). The rover is 38 cm; the tight route plans with
2 cm a side = 42: at the very edge, no plan in 8 s -- the robot was right.
In the twin as `chairs44` / `chairs50` (and the door as `door49`).

**Tight route speed** (b7191fe): it re-planned after every move (~10 s each,
~30 cm per 13 s). Now it follows its plan while the rover is where the plan
expected (4 cm / 4 deg); re-plans on a refusal or a miss. Tests pass; live on
the robot; **not yet exercised by a real goal with a plan** -- prove on
chairs50 / door49.

**Two front sonars** (owner's hardware, 2026-10-19; HARDWARE.md 2b): HC-SR04
type on their OWN ESP32 (MicroPython, `phase1/firmware/sonar/`), USB to the
Pi 5, whose `sonar_node.py` publishes `/sonar/left`, `/sonar/right`
(`sensor_msgs/Range`, frames `sonar_left` / `sonar_right` at
(0.182, ±0.14, 0.065), fov 0.52 rad). Bench: 15 cm target read 14.4-15.5 / 15.0.
A cone says HOW FAR, not WHERE, so they are used three ways, none of which
can make the rover more timid near what it already knows:

| use | rule | commit |
|---|---|---|
| nav2 collision monitor | `range` sources: echo -> an arc; `approach` caps speed so contact stays >= 1 s away, stops only at contact. `source_timeout 0.0`: a silent sonar is NOT a stop (the Pi node publishes +inf while its board is missing) | 7fde92e |
| witness (goal_exec, reach) | an echo <= 60 cm that no LiDAR / camera point in its cone explains -> one obstacle point ahead of that sensor (a slipper, a toy) | 01c7f61 |
| vouch (goal_exec, reach) | a LONE camera speck (<= 8 cm, no LiDAR within 10 cm), within 10 deg of a sonar's axis at 15-60 cm, whose last 3 echoes are all >= 10 cm beyond it (or none) is overruled -- the bedroom-door reflection that cost ~45 s (recorded fixture); a furniture edge / slipper patch never | 390c7dc |

The camera map the rover decides on is FLAT (fused_obstacles: no heights), so
silence cannot vouch a tabletop edge above the cone away -- the speck rule is
what keeps the morning's "drove under the table" from coming back.

Not yet: a drive test of the three (twin chairs / door + a hidden low object,
then the robot); the sonar service installed at boot (`sudo` on the Pi 5);
S1-left and straight-ahead mounting confirmed (`assumed` in params.yaml).
The Pi 5 dropped off the network the same night (no ping): power is the
first suspect -- it now feeds two ESP32s over USB.

## 2026-10-20: the Orin overload by detections_3d (fixed on the twin)

Real 2026-10-19: `detections_3d.py` at 58 % CPU + 1.1 GB with nobody subscribed.
The twin never showed it: `TWIN=1 ./rover up` skipped the follow/detect layer.
Started there by hand it sat at 29 % / 1.46 GB, idle. By elimination (twin, node
alone, 30 s): the 1 Hz idle YOLO 4 % + ~700 MB (its first inference creates the
GPU context), every depth (896x504) and colour frame into Python 7 %, and a Python
TF listener on /tf 18 % -- all for no reader (the Pi 5 brain dropped its detections
consumer; tools/approach.py).

| commit | change | measured (twin) |
|---|---|---|
| 00ab7aa | detections_3d SLEEPS without demand (target hunt, or a subscriber to detections_3d / target_result / people): no camera, no /tf, no YOLO; wakes in <= 0.25 s, sleeps 10 s after; a fresh TF buffer per wake | asleep 1.1-1.4 % CPU (was 29 %); first answer 0.2-0.4 s warm, 1.7-3.9 s on the first wake after a start; awake 48 % (YOLO 5 Hz, its job) |
| 386ddf9 | follow_node waits up to DETECT_WAKE_S 5 (+1) s for the first /vision/people; its 3 s pick window counts from that first message | "follow me", nobody in view: 'finding' in 0.5 s; tests pass |
| (this) | `TWIN=1 ./rover up` runs the follow layer like the robot; the detect gate's text | full twin bring-up |

Memory: once woken, the GPU context stays (1.46 GB). The detect gate's 8 s "what it
can see" sample wakes it at every bring-up, so follow-me's first answer is fast;
the Jetson keeps ~2.3 GB available. Next: confirm on the real robot (`./rover up`,
then `docker exec rover top`).


## Session 2026-10-20 (evening, twin) — tight places: check the camera, say "too tight", no wandering

Owner: "when we have a goal it needs to reach there, however complex it is" -- better
moves and decisions in tight places, twin first. Started from the real robot's reach log
of the afternoon (kino_sim `docs/handoff/reach_real_20261020.jsonl`, findings T1-T9 in
kino_sim `docs/NAV_NEXT.md`). Built on branches `nav-20261020-evidence` and
`nav-fix5-tight-leads`; merged into `rover-v1.1.16-review` the same night (below). Twin worlds added (kino_sim):
`stool_gap`, `gap36`, `dead_end`, `corridor_s` (60 cm).

### What was wrong, the evidence, and the fix

| # | what was wrong (real robot 2026-10-20) | fix (commit) | status |
|---|---|---|---|
| T1 T2 | when no plan fitted ALL points, the LiDAR-only plan was driven and goal_exec refused its first move **7 of 15** times on camera points (chair feet, a stool seat); the **same** refused plan was made again ("F40 L30 F135 L30 F20" twice) | the LiDAR plan is checked against the camera's points with goal_exec's own tests BEFORE a move (`tight_route.Evidence`): the clear part is driven, or a small look turn gives a second viewpoint; a camera point still there from another pose (5 cm / 10 deg by odometry) is believed for the goal; one the camera cannot see blocks plans only until the next move; a refusal makes the points the camera sees suspects (86e5682, 47a44dd, f70c47e) | unit tests (`test_tight_evidence`: refused 2 -> 0, repeated plan 1 -> 0; door speck test 2 -> 0 refusals); **twin: being measured** |
| T3 | way_back of **24 / 32 hops** for ~2 m nearer: it replayed the trail in time order | the shortest way over the trail (points < 30 cm apart joined; the rover is 38 cm wide); none that drives > 2.5 x its gain + 1.5 m (50218cd) | test: 12.2 m -> 3 m; twin dead_end back-out 49 s / 4.1 m -> **40 s / 2.7 m** |
| 7 | a goal through a gap too narrow took minutes ("no way from here" after recoveries) | `widest.py`: the max-min (bottleneck) path to the goal on the LiDAR's points; under 42 cm (body + a pass's 2 cm a side) even read generously -> **"too tight: the widest way is N cm, the rover needs 42"** at once (eb3cf72) | twin gap36 (36 cm): **269 s, 3.8 m -> 3.3 s, 0 m**; 42-60 cm gaps never refused |
| T5 | nav2 tries gaps it cannot plan (needs ~51 cm): door49 nav2 drove ~5.5 m in 67 s, then "no path" | the widest way 42-52 cm -> the tight route leads (branch `nav-fix5-tight-leads`) | **unmeasured** |
| -- | (laptop vs Orin) the tight search was bounded by time, so the laptop found plans the Orin never does | MAX_EXPAND 40000 -> 6000 (730ba75) | **REVERTED** (e432bf7): twin door49 and chairs44 timed out ("no fit found in the time"). The Orin's search is now emulated only in the unit tests |

### Measured in the twin (old = d7ee4e7; one run each unless noted)

| world | old | fixes (47a44dd, before f70c47e) |
|---|---|---|
| gap36 (36 cm, cannot fit) | failed 269 s, 3.8 m | **failed in 3.3 s, 0 m, with the number** |
| dead_end (in, then back out) | 23 s / 49 s (4.1 m) | 23 s / **40 s (2.7 m)** |
| stool_gap | 56 s / 70 s | 58 s / 64 s |
| door49 (49 cm) | 111 s (8.1 m) / 59 s | 119 s / 72 s |
| chairs44 (44 cm, 1 cm a side) | run 1: 30 s / 60 s; run 2: **failed 258 s** / 16 s | 182 s / timeout; 144 s / 130 s |
| corridor_s (60 cm S) | 106 s / failed | failed / 129 s; failed / failed |

No touches anywhere (judged on the true outline). chairs44 and corridor_s swing between
runs for every version (1 cm a side is the camera's noise): they need several runs per
version -- in progress. Two rules of the first evidence version made chairs44 slower
(a passing camera glitch believed for the whole goal) and were corrected (47a44dd,
f70c47e) from these runs.

### Learned

- A twin world must be possible: a 36 x 38 cm body cannot take a right-angle bend in a
  corridor under (36 + 38) / sqrt 2 = 52.3 cm, nor turn in place under ~56 cm.
- A limit taken from one day's log is a sample, not a bound (the 6000-pose cap).
- Believe the camera only when two views agree; never make a passing point a wall.

### Open, in order

1. Twin: the corrected evidence (f70c47e) vs the old code, several runs on chairs44 /
   corridor_s / door49; then fix 5 alone.
2. Nav2's rotation shim in front of MPPI (installed, unused): the dithering at gap mouths
   (stool_gap: 29 s of turning back and forth before the tight route).
3. The D555 temporal depth filter (flicker 1 cm a side decides chairs44).
4. Real robot with the owner, after the twin results.

### Merged into `rover-v1.1.16-review` (owner, 2026-10-20 ~22:15)

Owner: "commit all the codes into remote; if any issues on the real rover I will take care
and adjust." Merged as they stand -- fix 5 (tight route first, 42-52 cm) and the last
evidence correction (f70c47e) are **unmeasured in the twin**; the comparison runs were
stopped for the merge. If one misbehaves on the real robot, undo just that one
(`cd ~/rover && git revert --no-edit <commit>`, then `./rover nav` -- reach and
goal_exec are re-read at the layer's start):

| what you see on the real robot | likely fix | undo |
|---|---|---|
| goes for the tight route where nav2 used to manage a 45-55 cm gap; or turns/creeps more at doorways | fix 5 (tight route leads) | `git revert 5340e74` |
| refuses a gap that fits with "too tight: the widest way is N cm" | widest way (eb3cf72) -- by design it reads 5 cm generous and uses the LiDAR only, so this should not happen; a LiDAR return off a person or a reflection could | `git revert eb3cf72` (later commits build on it: check the merge) |
| looks around ("a look from another angle") a lot before tight moves; or drives a part of a plan, then stops | camera evidence (86e5682, 47a44dd, f70c47e) | `git revert f70c47e 47a44dd 86e5682` |
| "the way back drives X m to get Y m nearer" where the old long way back would have worked | way back (50218cd) | `git revert 50218cd` |

Logs that tell which one acted: the reach status lines (`/reach/status`, logs/reach_*.jsonl):
`phase: widest`, `check: ...`, `mode: look`, `way_back ... route_m`.

