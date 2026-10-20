# NAV_GUIDE — how this rover chooses a path and moves, and where to make it smarter

Written 2026-10-11 for the owner: *"learn costmaps and nav2, understand why the
path and movement are limited, fix them, and find where small decision-making
models fit, to grow this into a real product."*

Every number here is from `phase3/config/nav2.yaml`,
`phase3/bt/navigate_to_pose.xml` and `phase3/nodes/reach_node.py` at `e99b265`.
The older notes in `docs/archive/knowledge/06-07` describe NavFn, RPP and 5 cm
cells. They are out of date; read this page instead.

---

## 1. The whole chain, sensor to wheel

```
 SENSE                    MAP (costmaps)                 DECIDE + MOVE
 ─────                    ──────────────                 ─────────────
 D555 depth ─► nvblox ─► ghost_clear ─┐
                                      ├─► global costmap ─► Smac Lattice planner ─► /plan
 C1 LiDAR /scan ──────────────────────┤     (8x8 m, 2.5 cm)       (keeps its path; replans when blocked / 10 s)
                                      └─► local costmap  ─► MPPI controller ─► /cmd_vel_nav
                                            (4x4 m, 2.5 cm)        (10 Hz)
                                                                       │
     velocity_smoother ─► collision_monitor ─► turn_shaper ─► /cmd_vel ─► ESP32 ─► wheels
       (accel limits)      (slows near LiDAR    (lifts turns over
                            points, 1.0 s)       the 0.8 rad/s deadband)

 ABOVE nav2:  Pi 5 brain (LLM/VLM) ─► /reach/goal ─► reach_node ─► nav2 / goal_exec / tight_route
```

Three layers make decisions, at three speeds:

| layer | asks | speed | lives in |
|---|---|---|---|
| **brain** (Pi 5, Gemma + VLM) | *what* to do: "go near the door" → a goal pose | seconds | `~/ros2_ws` on the Pi 5 |
| **reach_node** + nav2 behaviour tree | *what to try next* when a try fails | ~1 s | `phase3/nodes/reach_node.py`, `phase3/bt/` |
| **planner + controller** | *which way*, *what velocity now* | on a blocked path or every 10 s / 10 Hz | nav2 |

Most "it moved stupidly" moments come from a lower layer doing exactly what its
numbers told it. So learn the bottom first.

---

## 2. Costmaps — what the planner believes about the floor

### 2.1 A cell, a cost

A costmap is a grid (here 2.5 cm cells). Each cell holds 0-254 and means
*"how bad is it for the rover's **centre** to be here?"*

| cost | name | meaning |
|---|---|---|
| 0 | free | open floor |
| 1-252 | inflated | allowed, but costs more the closer to something |
| 253 | inscribed | the body would touch something wherever it faced |
| 254 | lethal | an obstacle is in this cell |
| 255 | unknown | nobody has seen it (`allow_unknown: true` lets the planner use it) |

Planners look for the **cheapest** route, not the shortest. Costs are the
only way to tell the planner "keep away from walls".

### 2.2 Layers — four, stacked, merged by maximum

Global costmap, in order (`nav2.yaml` `plugins:`):

| layer | source | what it adds | weakness |
|---|---|---|---|
| `nvblox_layer` | camera depth → nvblox 3D map → `ghost_clear` → 2D grid, heights **3–31 cm** (31 since 2026-10-19: the rover is 26.3 cm, a 28 cm tabletop must be an obstacle) | low things the LiDAR misses (shoes, chair legs below 25 cm) and overhangs up to 31 cm. **Remembers** them | sees 87° ahead only; does not decay, so people who walked by stay as "specks" (S10) |
| (not a layer) **front sonars** | `/sonar/left`, `/sonar/right`: 2 cones (±15°) at 6.5 cm, front corners, 2 cm-4 m | what is low and close in front, under the LiDAR and inside the camera's blind 40 cm -- used by the collision monitor and reach / goal_exec (§5), NOT painted into the costmaps (a cone says how far, not where) | ahead only; soft things may swallow the ping |
| `lidar_layer` | `/scan`, one plane at 25 cm, 360°, returns from **5 cm** (20 cm until 2026-10-19 left ~15 cm in front of the nose unmarked) | everything round the rover, live; raytracing clears stale marks | one height only |
| `footprint_clear` | our plugin (`phase3/plugins`) | cells under the rover's own outline are free — it's standing there | global only |
| `inflation_layer` | the layers above | the cost ramp around every lethal cell | **see §2.4** |

The master grid takes the per-cell **maximum** of the layers. A layer can
raise a cell's cost but can never lower another layer's cost. That's why
`footprint_clear` sits after the obstacle layers and before inflation.

Local costmap: the same without `footprint_clear`, 4 × 4 m, updated 5 Hz.
MPPI checks against it.

### 2.3 The footprint and padding decide which gaps exist

```
body          36 x 38 cm  (front +0.182, rear -0.178, sides ±0.190)
global pad    +4 cm  → inscribed radius 0.218 m → smallest gap it can plan through ≈ 0.44 m
local pad     +2 cm  → inscribed radius 0.198 m
corner swing  circumscribed 0.32 m → needs ~64 cm of floor to turn all the way round in place
```

Gaps narrower than ~44 cm show as **"no valid path"** from the planner.
That's intended: reach's `tight_route` and gap pass then crawl them straight,
with 2 cm a side (`reach_node.py` steps 0 and 3).

The local pad (MPPI's HARD margin) stays **2 cm**. 1 cm was tried on
2026-10-19 (twin): on 2.5 cm cells MPPI then drove a corner 1.4 cm into the
sofa. Inside 2 cm MPPI refuses every motion; reach notices (the *pinned watch*,
§5) and steps the rover off on raw points at 1 cm precision. reach, goal_exec
and follow see LiDAR points right up to the outline (+5 mm); until 2026-10-19
they dropped everything within 2 cm of it as "the rover itself", which hid
furniture beside the body.

### 2.4 Inflation — the one setting that shapes every route

Cost beyond the inscribed radius is

```
cost(d) = 252 · exp(-cost_scaling_factor · (d − inscribed_radius))      for d < inflation_radius
cost(d) = 0                                                             beyond inflation_radius
```

Ours: `inflation_radius 0.80`, `cost_scaling_factor 2.0`. Worked out
(global, d = distance from the obstacle):

| d (m) | 0.25 | 0.30 | 0.40 | 0.50 | 0.60 | 0.70 | 0.79 | 0.81 |
|---|---|---|---|---|---|---|---|---|
| **k = 2 (now)** | 236 | 213 | 175 | 143 | 117 | 96 | **80** | **0** |
| k = 5 | 214 | 167 | 101 | 61 | 37 | 22 | 14 | 0 |
| k = 10 | 182 | 110 | 40 | 15 | 5 | 2 | 0 | 0 |

**What k = 2 does:** the cost is still **80** at the edge of the ramp, then
falls straight to 0. In any space under ~1.6 m wide (most of a home:
corridors, beside the cot, between chairs) **every cell is between 80 and 236**.
Nowhere looks like free floor. Effects:

- the lattice's `cost_penalty 3.0` makes every metre there cost 2-4× open floor, so the search runs longer and hits the 1.5 s limit sooner ("no valid path" with room to spare)
- MPPI's `CostCritic` (weight 6) charges every sampled trajectory, so the scores sit close together and MPPI hesitates and wiggles between them instead of driving
- the 80 → 0 cliff at 0.80 m pulls routes onto a line exactly 0.80 m from the nearest wall, wherever the room is wider than 1.6 m

**Why it is 2.0:** the comment says 10.0 was tried on 2026-09-27 and nav2
segfaulted. The 2026-09-28 crash trace later put **every** planner segfault of
those weeks in `NvbloxCostmapLayer::updateCosts`, which has since been
replaced. So the inflation values were probably never the crash. **k ≈ 5 with
radius 0.80 is worth a measured trial** (fix F2, §6).

### 2.5 See it yourself

```bash
# in the rover container, stack up
ros2 topic echo /global_costmap/costmap --once --field info     # size, resolution, origin
# then RViz:
#   Map display, /global_costmap/costmap, colour scheme "costmap"
#   pink/purple = lethal + inscribed, the cyan→red ramp = inflation
#   /reach/markers: what reach is doing; blocker view: who put each obstacle there
```

Exercise: park the rover 50 cm from a wall. Read the cost at the rover's cell
in RViz (Publish Point shows the cell). It should match the table above
(≈ 143 at k = 2).

---

## 3. Planner — Smac State Lattice

Searches over **(x, y, heading)**, not just (x, y), with 104 *motion
primitives*: short arcs (0.4 m radius), straights, turns in place, reverse
(`config/lattice_diff_2p5cm_r0p4.json`). Every pose of every candidate path is
checked with the **full padded outline**, so a path it returns is one the body
fits.

A path's cost = length × penalties:

| knob | ours | effect |
|---|---|---|
| `cost_penalty` | 3.0 | how much inflation cost counts (higher = hugs the middle more) |
| `rotation_penalty` | 5.0 | turning in place is expensive (the skid-steer slides 6-33 cm a turn) |
| `reverse_penalty` | 3.0 | reverse allowed but priced — the rear is blind below 25 cm |
| `non_straight_penalty` | 1.05 | curves slightly dearer than straights |
| `max_planning_time` | 1.5 s | **runs out = "no valid path"**, even when one exists |
| `tolerance` | 0.25 m | goal blocked → nearest free pose within 25 cm |
| window | 8 × 8 m rolling | a goal > ~4 m away is "outside bounds" → reach drives it in legs |

`allow_unknown: true`: the camera cannot see the floor under the rover, so
without this it could never plan from a fresh start. The cost: it will route
through floor nobody has looked at.

**Why it is slow on this Jetson:** 2.5 cm × 16 headings × 8 m = millions of
states, on a CPU at load 12-14 (E4). `cache_obstacle_heuristic` helped.

---

## 4. Controller — MPPI, and every limit on speed

Every 0.1 s, MPPI:

1. samples **1000** velocity sequences, each **30 steps × 0.1 s = 3 s** ahead
2. rolls each forward with the diff-drive model (it trusts that what it asks for happens — which is true only because `turn_shaper` makes it true)
3. scores each with **critics**, then blends them, weighted by score, into one command

| critic | weight | rewards |
|---|---|---|
| `CostCritic` | 6.0 | low costmap cost; **rejects** any trajectory whose outline touches lethal |
| `PathAlignCritic` | 8.0 | staying on the planner's line |
| `PathFollowCritic` | 5.0 | making progress along the path |
| `GoalCritic` / `GoalAngleCritic` | 5 / 3 | getting to the goal, and facing it at the end |
| `PreferForwardCritic` | 5.0 | forward over reverse |
| `ConstraintCritic` | 4.0 | staying inside the velocity limits |
| `PathAngleCritic` | 2.0 | pointing along the path |

**Who limits speed, top to bottom:**

| limit | value | when it bites |
|---|---|---|
| MPPI `vx_max` / smoother | 0.35 / 0.36 m/s | open floor |
| `wz_max` | 0.9 rad/s real | turns |
| `CostCritic` vs inflation | — | **everywhere under 1.6 m wide at k = 2 (§2.4)** |
| `collision_monitor` Approach | 1.0 s to contact | 35 cm warning at full speed; it scales speed down near LiDAR points |
| `controller_frequency` | 10 Hz | coarse reactions; NAV_PLAN N3 aimed for 20 Hz, CPU-bound |

---

## 5. Who decides what to try next

1. **nav2's behaviour tree** (`phase3/bt/navigate_to_pose.xml`): keep the current path while it is valid; replan only when it is blocked, the goal changes, or it is 10 s old (2026-10-19; at 1 Hz each plan could pick the other way round and the rover turned back) → follow → on failure: clear local costmap → BackUp 0.30 m → Wait 5 s, round-robin, up to 6 times. Spin is **removed** (2026-10-03: it hit a chair in a 45 cm gap).
2. **reach_node** wraps nav2 and is the real recovery brain: tight route → way back along its own trail → look (turn the camera) → pass a narrow gap → wait → retry, up to 8 attempts / 300 s; same failure 3 times from one spot → stop and say so. It watches nav2 while it drives: **dithering** (turning back and forth, no nearer) and, since 2026-10-19, **pinned** (still 10 s with something inside nav2's 2 cm margin) cancel nav2 and go to the escape. The **escape** steps off what the rover is parked against: a short drive, a turn about the learned pivot, drive-then-turn, or — along something the whole length of a side — a **steering slide** (crawl on, steering off the nearest point). A crash inside one goal no longer kills reach: the goal fails with "internal error" and reach keeps serving.
   Far goals go in **legs** along a route through the house map (`house_route.py`, ON by default since 2026-10-19; `REACH_HOUSE_ROUTE=0` = the old straight-line legs), which also carries nav2's own obstacles and costs, so the route and nav2 agree about which way round the furniture to go. Where the route goes through a gap nav2 cannot plan (a 49 cm door: its 16-heading lattice and 4 cm padding do not fit), legs find no reachable point and the **tight route** takes the gap instead, aimed along the route, with up to 8 s to search (the Orin needs 3-6 s for a door).
3. **goal_exec** does the exact finish (~1.4 cm) and learns the pivot slide from every turn.
4. **The front sonars** (2026-10-19) sit beside these, never in the costmaps: nav2's collision monitor slows for an echo close ahead (stops only at contact; a silent sonar is ignored); goal_exec and reach treat an echo the LiDAR / camera map cannot explain as one more obstacle point (the *witness*), and overrule a lone camera speck a sonar squarely hears nothing at (the *vouch*: floor reflections on polished marble). Rules and why: `phase3/nodes/sonar_witness.py`.
   The tight route follows its plan between moves while the rover is where the plan expected (4 cm / 4°), and re-plans only on a refusal or a miss.
   **Since 2026-10-20:** a plan made on the LiDAR alone is checked against the camera's points before it is driven (`tight_route.Evidence`): a camera point is believed only when it is still there from a second viewpoint (5 cm / 10° by odometry) -- a glossy-floor speck goes away, a chair foot stays and is planned around; one the camera cannot see blocks plans only until the next move. On the first "no path" of a goal, once stopped, `widest.py` measures the **widest way** to the goal on the LiDAR (a max-min path): under 42 cm the goal ends at once, "too tight: the widest way is N cm, the rover needs 42". The **way back** takes the shortest way over the trail and refuses one that drives far more than it gains. A widest way of 42-52 cm sends the tight route first, before nav2 (which plans only from ~51 cm).

That reach layer is **rules written from floor tests**. It is the right place
for the first learned decisions (§7).

---

## 6. Why the paths and moves are limited — causes, ranked

| # | limitation you see | cause | evidence | fix |
|---|---|---|---|---|
| L1 | slow, hesitant, wiggly in rooms; drives on a line 80 cm off a wall | inflation k = 2: no free floor anywhere under 1.6 m wide, cliff at 0.80 | §2.4 table | **F2** |
| L2 | "no valid path" with obvious room; minutes of retries | 1.5 s planning limit on an overloaded CPU; stale nvblox specks closing corridors | E15 (241 s at a door), S10, E4 load 12-14 | **F3**, **F5** |
| L3 | way back closed by "nothing" | nvblox never forgets; people leave specks; camera faces away so they never clear | S10 measured, 12 blobs on the corridor | **F5** |
| L4 | small turns stall beside furniture | skid scrub + battery sag; deadband 0.8 rad/s | E3/S6, 2026-10-03 battery measure | charge first; power redesign (ROVER_BUILD_PLAN §4.5) |
| L5 | the same caution in a hallway and in open floor | one parameter set for every situation | config | **F4** |
| L6 | ages before it starts moving | VLM round trips (10-45 s) on the critical path | E5, INTELLIGENCE_PLAN §2 | not nav2: brain-side |
| L7 | no number says if a change helped | the N0 course (NAV_PLAN) was never run | NAV_PLAN status table | **F1** |

### The fix order (one at a time, owner on the floor, as agreed for M1-M5)

| fix | what | done when |
|---|---|---|
| **F1** | Run the N0 course before touching anything: `phase3/tools/plan_check.py` offline on a saved costmap + ~8 real goals with `navto.py` (pocket, doorway, behind, long run). Record time, recoveries, min clearance, plan failures | a baseline CSV committed in `logs/` |
| **F2** | Inflation `cost_scaling_factor` 2.0 → **5.0**, radius stays 0.80. Watch for crashes (libbackward traces go to `logs/nav_crashes/`) | F1 course: faster, no new contacts, no crash in 30 min |
| **F3** | Planning time: profile `planner_server` CPU on the course; try global costmap at 5 cm with a regenerated 5 cm lattice (local stays 2.5 cm) if > 0.5 s typical | "no valid path" only where the gap truly is < 44 cm |
| **F4** | Two controller profiles: **open** (vx 0.35, CostCritic lower) and **tight** (vx 0.15, CostCritic higher). The BT already has a `ControllerSelector`; reach (or a small node) picks by LiDAR clearance | quicker in open, no more contacts in tight |
| **F5** | Specks: nvblox slow decay *or* clear cells the LiDAR sees through **and** the camera re-observed as empty. Owner's call (nvblox "never forget" is deliberate) | S10's corridor stays open after people pass |

Each is a one-line yaml change or a small node. Each gets its own commit with
the course CSV, as NAV_PLAN N5 asks.

---

## 7. Where small intelligent models fit

The rule every working product follows (vacuums, warehouse AMRs, Astro): **a
model never sits inside the 10 Hz motion loop.** Models choose *what*, *where*
and *which strategy*; the classical stack moves. That keeps it safe and
explainable. Places in this stack, cheapest first:

| where | decision | model | data you already have |
|---|---|---|---|
| **M-a reach: which recovery** | after a failure, pick tight route / way back / look / pass / wait / give up | start: the current rules + an **episode log**. Then a small decision tree or gradient-boosted classifier (scikit-learn, runs in microseconds) on features: 8 LiDAR sector clearances, nav2 error code, attempts so far, distance to goal, gap width | reach status JSON already has phase, why, result — log it per attempt |
| **M-b controller profile** | open vs tight (F4) | first a rule on LiDAR clearance; later the same classifier style | `/scan` |
| **M-c "is this goal reachable?"** | before driving, predict a no-path and pick another pixel / look again in seconds instead of 241 s (E15 fix 2) | rule: `ComputePathToPose` dry run (reach `plan_ok` does this); model only if rules fall short | E15 logs |
| **M-d semantic costmap layer** | rugs, cables, pet bowls, glass doors → extra cost or keep-out | a small detector/segmenter on the D555 colour (YOLO-seg, TensorRT on the Orin), projected with depth into a costmap layer | needs labelled frames — the VLM teacher → student loop in INTELLIGENCE_PLAN §1.4 |
| **M-e learned speed** | how fast is safe here | from logs: speed vs clearance vs outcome | collision_monitor state + odom |
| **M-f people-aware** | slow and pass behind a walking person | `/vision/people` with track ids already exists (follow me) → a costmap layer ahead of their motion | person_tracker.py |

**The first step is not a model, it's the log.** Without episodes there is
nothing to learn from. Proposed: reach writes one JSON line per attempt to
`/logs/episodes.jsonl` (goal, start pose, features above, chosen recovery,
result, seconds). After ~200 real goals, a decision tree trained on it shows
which rule is costing time — and it is readable, so it can become a better
rule or ship as the model.

---

## 8. Learning path — do these on the rover

1. **Read the costmap.** RViz, global costmap, colour scheme `costmap`. Park near a wall, check §2.4's numbers.
2. **Break a gap on purpose.** Two boxes 50 cm apart, then 42 cm. Send a goal through. Watch: plan at 50, "no valid path" at 42, then reach's tight route crawling it.
3. **Watch MPPI think.** Set `visualize: true` for one session (CPU cost): the sampled trajectories draw in RViz. See how they bunch when the costmap is all ramp.
4. **Follow one goal through every layer:** `/reach/status` → nav2 log → `/plan` → `/cmd_vel_nav` → `collision_monitor_state` → `/cmd_vel`. Find where the speed dropped.
5. **Change one number, measure** (F2 is a good first). That's the habit that turns this into a product.

Further reading: nav2 docs, *Tuning Guide* (docs.nav2.org/tuning) and the
MPPI and Smac pages. Read them next to `nav2.yaml`: every value there says
why it was set, when, and what was measured.
