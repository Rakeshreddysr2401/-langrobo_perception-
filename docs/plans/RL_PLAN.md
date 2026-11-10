# RL_PLAN — a learned driver for tight spaces: trained in Isaac Lab, kept safe by a shield, improved from every real drive

Written 2026-11-06 for the owner: *"I want my robot to move more intelligently — can we use
reinforcement learning with Isaac Lab and Sim, so that once it moves it learns?"* and *"keep a
complete plan ready and document it; I will see."*

**Status 2026-11-07: R1-R4 BUILT** (training: `~/kino_sim/rl/`; robot: `nav/nodes/rl_drive.py`,
`reach_node.rl_round`, `./rover rl on|off`, `./rover demo start|stop`; policy `models/rl/current` -> v002),
twin-validated, first real tries done. **Where it stands and what to do next: `~/kino_sim/docs/NAV_NEXT.md`
section 13** (real findings, sim-to-real gaps for v003, platform blockers, real data, order of work).
The rest of this file is the original plan.

Checked 2026-11-06 before writing: the laptop (`kino`, now 192.168.1.8 on DHCP, same host key as
.9) has Isaac Lab 3.0.0 + RSL-RL 5.5.1, smoke-tested (`~/kino_sim/docs/VERSIONS.md`), RTX 4050
6 GB idle. The Jetson's `rover` container has numpy 1.26.4 and **no onnxruntime**. The twin's
world files are boxes and cylinders with heights (`build/worlds/*.json`, 3040 boxes / 36 cyls).

---

## Contents

0. [The plan on one screen](#0-the-plan-on-one-screen)
1. [Goal and how success is measured](#1-goal-and-how-success-is-measured)
2. [Why the robot fails today, and what RL can and cannot fix](#2-why-the-robot-fails-today-and-what-rl-can-and-cannot-fix)
3. [Architecture](#3-architecture)
4. [The policy](#4-the-policy)
5. [The safety shield](#5-the-safety-shield)
6. [The training environment](#6-the-training-environment)
7. [Deployment on the Jetson](#7-deployment-on-the-jetson)
8. [Evaluation: the test ladder](#8-evaluation-the-test-ladder)
9. [Data, logs and versions](#9-data-logs-and-versions)
10. [How it improves: the learning loops](#10-how-it-improves-the-learning-loops)
11. [Files and repos](#11-files-and-repos)
12. [Phases, tasks, effort](#12-phases-tasks-effort)
13. [Risks and what we do about them](#13-risks-and-what-we-do-about-them)
14. [Decisions for the owner](#14-decisions-for-the-owner) · 14a. [Other architectures considered](#14a-other-architectures-considered-owners-question-2026-11-06-vlas-or-other-ways)
15. [Words used here](#15-words-used-here)

---

## 0. The plan on one screen

- **Job:** a small neural network (the *policy*) drives the rover through the **hard last
  metres**: doors, gaps between chairs, narrow corridors, out of corners. nav2 keeps open floor,
  goal_exec keeps the exact finish, the map keeps room-to-room routes.
- **Eyes:** what the sensors see *now* (LiDAR, fresh camera points, sonars, its own motion),
  10 times a second. No search to time out, no stale map specks.
- **Safety:** every command it proposes passes a **shield** that sweeps the body outline
  (slide included) against the points first. The policy can be clumsy; it cannot touch.
- **Training:** Isaac Lab on the laptop, ~2048 virtual rovers at once in random rooms, moving
  by the rover's **measured** motion model, across every battery state. Hours, not weeks.
- **Learning every day:** real failures become training rooms, the sim is re-fitted to the real
  motion, the policy is retrained overnight and **used only if it is measurably better and
  touches nothing** on the twin.
- **Off switch:** `REACH_RL=0` is today's robot, exactly.

## 1. Goal and how success is measured

### 1.1 Targets, from the twin baseline of 2026-11-01

`~/kino_sim/docs/NAV_NEXT.md` §10: 11 worlds, nothing changed, rover `e60f311`.

| world | baseline | after tight_route fixes (e1d3fe0) | target |
|---|---|---|---|
| door49 — real bedroom door, 49 cm past the bed | failed 122 s | reached 87 s (1 run) | reached < 45 s |
| chairs44 — 44 cm between the real chairs | failed 165 s | not re-run | reached < 60 s |
| corridor_s — 60 cm S-corridor | failed 166 s | not re-run | reached < 45 s |
| apt_hard — 6 goals in an apartment | 6/6, **touched** the jamb -0.3 cm | not re-run | 6/6, no touch |
| gap45, door49 — hand-over from nav2 | nav2 dithered 38–43 s | — | hand-over < 5 s |
| gap36 — impossible (36 cm, rover 38) | "too tight" in 3.8 s | — | unchanged |
| the other 6 worlds | reached | — | no worse (time + 20 % at most) |

### 1.2 Metrics (every run, sim, twin and real)

| metric | definition | source |
|---|---|---|
| **touch** | true outline vs world geometry < 0 cm at any instant. **Any touch fails the run** | twin judge (`./sim scenario`); sim env; real: owner's eyes + sonar < 2 cm |
| reached | within 5 cm and 10° of the goal | `/reach/status` |
| time | goal sent → reached / gave up | `/reach/status` |
| min clearance | smallest outline-to-point distance during the run | twin truth; real: shield's own measure |
| shield rate | % of policy steps the shield changed | `rl_episodes.jsonl` |
| refusal quality | impossible gaps refused, and how fast | gap36 |

## 2. Why the robot fails today, and what RL can and cannot fix

The robot already has every sensor it needs nearby. The failures come from **how decisions are
made** on that data, and that is what a policy changes.

| cause today | evidence | RL? |
|---|---|---|
| tight_route's search (A* over poses) runs out of time on the loaded CPU | door49 / chairs44 "no fit found in the time", 5–7k poses in 8 s (NAV_NEXT §10) | **yes** — the policy answers in < 1 ms per step, there is no search |
| the planners assume a clean pivot; the skid-steer slides 3–6 cm a turn (6–33 cm per 90° by pack) | apt_hard jamb touch (MPPI corner swing, NAV_NEXT §10 next-step 2) | **yes** — it trains on the measured slide at every pack state |
| nav2 swings between two choices before handing over | gap45 / door49: 38–43 s dithering | **yes** — trained for steady progress; reach hands over sooner (§3.3) |
| stale specks in the remembered map close a free way | corridor_s: the true shapes have a way (found in 63 poses), the rover's own specks closed it | **partly** — it reads fresh points, not the map; a fresh speck still needs the two-viewpoint filter (`tight_route.Evidence`), which the policy inputs keep |
| things under ~3 cm | slippers, the 1 cm tablet (home_real) | **no** — no sensor sees them; no model can avoid them |
| no drop-off sensing | ROVER_BUILD_PLAN §9.6 | **no** — hardware |
| long routes through the house | — | **not its job** — slam map + house_route + nav2 |

## 3. Architecture

### 3.1 Where it sits

```
 Pi 5 brain (LLM/VLM)          "go to the bedroom"                     WHAT      (seconds)
        │ /reach/goal
 reach_node                    which tool for this moment              STRATEGY  (~1 s)
   ├─ open floor ─────────────► nav2: Smac lattice + MPPI               unchanged
   ├─ within 1.2 m, clear ────► goal_exec: exact finish, 1.4 cm         unchanged
   ├─ impossible ─────────────► widest.py: "too tight", fast            unchanged
   └─ narrow / no path /  ────► RL DRIVE (new, inside reach)            HOW       (10 Hz)
      dithering / pinned            policy(obs) → (vx, wz) proposal
                                    shield(proposal, points) → safe (vx, wz) or stop
                                    → /cmd_vel → turn_shaper → ESP32 → wheels
                                  hand back to goal_exec for the exact finish
   RL gave up → tight_route → way back → look → wait → retry           all unchanged
```

### 3.2 Decision: the policy runs INSIDE reach_node, not as a new node

| option | for | against |
|---|---|---|
| **in reach (chosen)** | reach already gathers the exact points the policy and shield need (`near_now()`: LiDAR + fused camera points with the two-viewpoint evidence + sonar witness); reach already drives closed-loop itself (`slide_along`, `drive_straight`); no new topics, no new process on a CPU at load 13–17 | reach_node grows (1613 lines); mitigated: the policy and shield are ROS-free modules like tight_route |
| new `rl_drive` node | separate restart | duplicates the point pipeline; one more process; hand-over races on /cmd_vel |

### 3.3 When reach uses it (`REACH_RL=1`)

| trigger (reach already detects each) | today's next step | with RL |
|---|---|---|
| nav2 "no valid path" and widest way 42–60 cm | tight_route | **RL first**, tight_route if RL gives up |
| narrow ahead (nav2 "collision ahead", goal_exec "obstacle … leg") | gap pass | RL first |
| dithering watch fires | escape | RL |
| pinned watch fires (still 10 s inside nav2's 2 cm) | escape | RL |
| widest way < 42 cm | "too tight" | unchanged — RL never tries the impossible |

**RL gives up** (back to the old chain) when: no progress of 10 cm in 15 s, or 3 shield stops in
a row, or 60 s. **Hand-back:** when within 0.3 m of the goal with the outline clear, goal_exec does
the exact finish (it is better at the last centimetres than any policy).

### 3.4 What does not change

nav2 config, goal_exec, tight_route, widest, trail, escape, sonar rules, the brain, every topic.
`REACH_RL=0` (the default until R5 passes) is today's robot.

## 4. The policy

### 4.1 Observation (what it sees), 177 numbers, per step at 10 Hz

| block | size | content | normalised |
|---|---|---|---|
| clearance now | 72 | per 5° sector: smallest **distance from the body outline** to any point in that sector (LiDAR; v2: min with fresh camera low points) | clip 0–3 m, ÷3 |
| clearance before | 72 | the same, previous step (motion cue) | same |
| sonars | 2 | left, right range (v2; v1 fills 1.0 = nothing) | clip 0–2 m, ÷2 |
| goal | 4 | distance, sin/cos of bearing (body frame), final heading error | dist ÷3 |
| motion | 2 | measured vx, wz (from /odom) | ÷0.3, ÷1.0 |
| last action | 2 | previous vx, wz | as action |
| pack state | 3 | goal_exec's learned pivot (px, py) for the turn direction, turn gain k | px,py ÷0.1 |
| progress | 2 | time in RL mode ÷60, walking-distance gain over the last 2 s | — |
| **total** | **159 (v1) / 177 with v2 extras** | | |

*Why "clearance from the outline", not raw range:* the rover is a 36 × 38 cm box, not a point.
Giving distance-from-the-body makes the policy's job "keep every number above 0" directly.

### 4.2 Action

`vx ∈ [-0.12, 0.25] m/s`, `wz ∈ [-0.9, 0.9] rad/s`, tanh-squashed, held 0.1 s. Reverse is capped
lower because the rear is blind below 25 cm.

### 4.3 Network

MLP 256 → 128 → 64, ELU, ~70 k weights. **Memory option:** if rung 1–2 show it re-trying the same
move in dead ends (it sees only the last 0.2 s), a GRU of 64 units goes before the MLP (RSL-RL
supports recurrent policies); still well under 1 ms. Forward pass in **numpy** on the Jetson (< 1 ms; no
onnxruntime in the container, and none needed for a matrix multiply). Exported as `policy.npz`
(weights + observation normaliser) alongside `policy.onnx` for inspection.

## 5. The safety shield

`nav/nodes/rl_shield.py`, ROS-free, unit-tested, used in training (§6.8) and on the robot.

**Every step, before a command reaches the wheels:**

1. Roll the proposal (vx, wz) forward **0.8 s** with the motion model, slide included (the
   learned pivot for that turn direction), in 0.05 s steps.
2. Sweep the outline + **MARGIN 1 cm** (goal_exec's) along that motion against the points reach
   decides on (LiDAR, confirmed camera points, sonar witness).
3. Clear → send it. Not clear → try, in order: the same turn at half speed; the proposal's turn
   with vx = 0; 8 nearby commands (±0.05 m/s, ±0.2 rad/s); first clear one is sent.
4. None clear → **stop**, count a shield stop. 3 in a row → RL gives up (§3.3).
5. Also stop on: /odom older than 0.3 s, /scan older than 0.25 s, teleop MANUAL (as today).

Every intervention is logged (§9). A policy with a high shield rate is a bad policy, and the
nightly gate (§10) rejects it.

## 6. The training environment

### 6.1 Framework

Isaac Lab 3.0 **DirectRLEnv**, headless, with RSL-RL PPO; train/play/export via Isaac Lab's
`scripts/reinforcement_learning/{train,play}.py` (the path proven on the laptop).

**The world is torch tensors on the GPU, not USD meshes.** Each env gets a **new random room at
every reset**; rebuilding meshes per reset is slow, and the sensors that matter are planar
(LiDAR at one height, camera low points, sonar cones). Exact ray–box and ray–circle casts in
torch, batched over all envs. Isaac Sim's full rendering is used where it matters: the twin judge
(§8 step 3).

### 6.2 World representation (per env)

- up to **48 boxes** (x, y, sx, sy, yaw, z0, z1) + **16 cylinders** (x, y, r, z0, z1) — the same
  kinds as the twin's world files, so those load directly
- visible to the **LiDAR** if the plane (25 cm) lies in [z0, z1]
- visible to the **camera points** if [z0, z1] overlaps 3–31 cm, inside 87° ahead, 0.25–3 m
- visible to a **sonar** if inside its ±15° cone, 2 cm–4 m (soft-object dropouts randomised)
- **touch** = the true outline (36 × 38 cm, no margin) overlaps anything with z0 < 26.3 cm (the
  rover's height) — the twin judge's rule

### 6.3 Room generators (mixed, weights adjustable)

| generator | parameters | weight | real source |
|---|---|---|---|
| doorway | opening 42–90 cm, wall 5–15 cm, optional door leaf 0–90°, obstacle near the far side | 25 % | door49, door80 |
| S-gap | two objects 42–70 cm apart, offset 20–60 cm, one thin (3–7 cm leg) | 20 % | gap45/50/60 |
| corridor | 48–80 cm wide, 0–2 bends of 60–90° | 15 % | corridor_s |
| dead end | pocket 45–70 cm wide, 50–90 cm deep, goal behind | 10 % | dead_end |
| pinned start | start 1–3 cm from a wall / sofa along one side | 10 % | pinned_side |
| clutter | 6–20 boxes and legs in 3 × 3 m, one ≥ 44 cm way through | 15 % | clutter, chairs44 |
| impossible | the only way < 40 cm | 5 % | gap36 — the right answer is to stop, not touch |
| **moving obstacles** (added to every generator from curriculum level 2) | 0–2 "people" (30 cm cylinders, 0.3–1.0 m/s, walk across or along the way, pause) and a "pet" (20 cm, erratic) | in 30 % of episodes | follow_node's people; the corridor specks people left (S10) — the right answer is wait or go round, never cut in |

Goals: 0.8–3.5 m away, through the hard part, reachable by construction (except "impossible").

### 6.4 Sensor simulation

| sensor | sim | noise / faults randomised |
|---|---|---|
| LiDAR (C1) | 720 beams at 0.5°, 10 Hz, 0.05–16 m, then sector min to 72 | ±1 cm range, 2 % dropped, incidence dropout (kino_sim `config/rover.yaml` lidar), 0–0.1 s latency |
| camera points | low obstacles in the FOV band, 15 Hz | ±0.5 cm; **0–8 false specks** near the path that vanish on the next look (glossy floor, people: S10, corridor_s) |
| sonars | min range in cone | floor echoes, silence on soft objects (`sonar_witness.py` rules) |
| odometry | true motion + noise | ±1 cm, ±0.5° per step (fusion2 sd 0.6 cm) |

### 6.5 Motion

`~/kino_sim/hardware/motion.py`, vectorised in torch (same equations, same yaml): side deadband
1 cm/s; in-place turn `k·(0.52|wz| − 0.21)`, nothing below ~0.4; 22 % turn following while
driving; 0.97 forward gain; 0.2 s first-order lag; 2–3 % noise; **rotation about the pivot P**.

### 6.6 Domain randomisation (one draw per episode)

| what | range | why |
|---|---|---|
| pivot left / right | between `fresh` and `weak` packs (kino_sim `config/rover.yaml` motion.packs), plus ±2 cm | 6–33 cm slide per 90° |
| k | 0.65–1.05 | pack state |
| lag | 0.1–0.3 s | |
| forward gain | 0.9–1.03 | floor, tyres |
| command latency | 0–0.1 s | WiFi to the ESP32 path |
| sensor noise | §6.4 | |

**The pivot and k the policy is told** (§4.1 pack state) are the true ones ± goal_exec's
typical estimate error, so it learns to use them without trusting them blindly.

### 6.7 Episode

physics/motion step 0.02 s, policy every 5 steps (10 Hz); 60 s max (600 decisions).
**Ends:** reached (5 cm, 10°) · touch · 60 s · in "impossible", standing still 3 s counts as the
right answer.

### 6.8 Reward

Per decision step:

| term | value |
|---|---|
| progress | + 2.0 × (walking distance before − after), walking distance = BFS on a 5 cm grid of the room at reset (tight_route's `_guide` idea) |
| reached | + 10 |
| touch | − 10, episode ends |
| near | − 0.05 × max(0, 3 cm − min clearance) / 3 cm |
| time | − 0.01 |
| reverse | progress made in reverse counts × 0.33 |
| smooth | − 0.02 × (|Δvx| / 0.1 + |Δwz| / 0.5) |
| shield | − 0.2 when the shield (run in training too, as the robot will) changed the command |
| impossible | + 5 for stopping within 0.5 m of the narrowest point without a touch |

Training **with** the shield in the loop means the policy learns the robot's real situation:
proposals that the shield would refuse cost reward and do nothing.

### 6.9 Curriculum

Level 0: gaps ≥ 70 cm, 0 specks, `fresh` pack only. Each time the success rate over the last
2000 episodes passes 80 %: −4 cm on the smallest gap (floor 42 cm), widen the pack range, raise
the specks and noise. Level and success are logged per iteration.

### 6.10 PPO settings (RSL-RL), first values

| | |
|---|---|
| envs | 2048 (no rendering; fits 6 GB) |
| steps per env per iteration | 32 |
| iterations | ~3000 (≈ 200 M steps), checkpoints every 100 |
| learning rate | 3e-4, adaptive (desired KL 0.01) |
| gamma / lambda | 0.99 / 0.95 |
| epochs / minibatches | 5 / 4 |
| clip / entropy | 0.2 / 0.005 |
| init action std | 0.5 |
| observation normalisation | on (exported with the weights) |
| expected wall time | 1–3 h per full run on the RTX 4050 (to be measured in R1) |

## 7. Deployment on the Jetson

| piece | where | does |
|---|---|---|
| `rl_policy.py` | `nav/nodes/` | ROS-free: build the 159/177 observation from reach's points + pose + goal; numpy forward pass |
| `rl_shield.py` | `nav/nodes/` | ROS-free: §5 |
| `reach_node.py` | `nav/nodes/` | `run_rl(g)`: the 10 Hz loop (like `slide_along`), triggers §3.3, give-up and hand-back rules, the episode log |
| `models/rl/vNNN/` | repo (`models/` is mounted read-only at `/models`) | `policy.npz`, `policy.onnx`, `card.md` |
| `REACH_RL`, `REACH_RL_MODEL` | env in `./rover nav` | on/off; which version |
| RViz | `/reach/markers` | the proposed vs shielded command and the swept outline, so a human sees what it intends |

## 8. Evaluation: the test ladder

Each rung must pass before the next. Every result is written into `models/rl/vNNN/card.md`.

| rung | where | what | pass |
|---|---|---|---|
| 1 | sim, held-out seeds | 2000 episodes per generator, levels as trained | ≥ 90 % reached (gaps ≥ 44 cm), **0 touches**, "impossible" stopped ≥ 95 % |
| 2 | sim, the twin's worlds | door49, chairs44, corridor_s, gap45/50/60, dead_end, pinned_side, apt_hard, clutter, gap36 loaded from `build/worlds/*.json`, 20 runs each with randomised packs | 0 touches; faster than tight_route on the same world (offline, `replay_tight.py`-style) |
| 3 | **twin, full stack** | `TWIN=1 ./rover up` with `REACH_RL=1`, `./sim scenario` on the 11-world suite, 2 runs each, vs the 2026-11-01 baseline + e1d3fe0 fixes | no touch anywhere; §1.1 targets met; no world worse by > 20 % |
| 4 | **real floor**, owner watching, pack charged | gap45 boxes, the bedroom door, the chairs, a corner start; 3 runs each; MANUAL on the phone is the stop | no touch; reached; card updated with the real numbers |

Only after rung 4 does `REACH_RL=1` become the default.

## 9. Data, logs and versions

**`logs/rl_episodes.jsonl`** (Jetson, one line per RL activation):

```json
{"t": "2026-11-20T18:03:11", "policy": "v003", "goal": [x, y, th], "start": [x, y, th],
 "trigger": "no_path", "result": "reached|gave_up|cancelled", "why": "...", "s": 23.4,
 "min_clear_cm": 2.1, "steps": 234, "shield_changed": 11, "shield_stops": 0,
 "pack": {"px": 0.05, "py": 0.04, "k": 0.85}, "load": 14.2, "dump": "rl_dumps/....npz"}
```

**`logs/rl_dumps/*.npz`**: for failures, gave-ups, > 30 s, or any shield stop — the points
(LiDAR, camera, sonar), poses and commands of the whole activation, same layout as
`tight_dumps` (abfc30c) so `replay_tight.py`-style tools read both.

**Versions:** `models/rl/v001`, `v002`, … each with `card.md`: training commit (kino_sim), rover
commit, generators and levels, rung 1–4 results, date. Never overwritten; reverting = one env var.

## 10. How it improves: the learning loops

Learning from scratch on the real robot is not an option (millions of tries, thousands of
contacts, a person resetting it). The robot learns in five loops, fastest first:

| loop | what changes | input | when | risk |
|---|---|---|---|---|
| **L1 live adaptation** | nothing is retrained: the policy reads goal_exec's live pivot + k (§4.1) and was trained over every pack, so it adapts as the battery drains | every turn | instantly | none |
| **L2 sim re-fit** | the motion ranges the sim trains on (§6.6) | logged `/cmd_vel` vs LiDAR-measured motion (`./rover record` + `./rover grade`, and every RL episode) | nightly | none |
| **L3 failure rooms** | the training mix: each `rl_dumps` file becomes a room (points clustered to 3 cm cylinders, the start, the goal) and 20 % of training episodes are drawn from these | real + twin failures | nightly | none |
| **L4 retrain + gate** | v(n) → v(n+1), fine-tuned from v(n) for ~1 h | L2 + L3 | nightly, laptop | **used only if** rung 1–2 better than v(n) AND rung 3 no touch; else discarded, with the reason logged |
| **L5 residual online RL** | a small correction network on top of the frozen policy, learning while driving, inside the shield, capped at ±20 % of the command | live | only if L1–L4 plateau | low |

```
 real drive ─► rl_episodes + rl_dumps ─► laptop (night): L2 refit + L3 rooms
     ▲                                             │
     │                               L4 fine-tune v(n) → v(n+1)
     │                                             │
     │                         rung 1-2 better? ── no ─► discard, log why
     │                                             │ yes
     │                         rung 3 twin, 0 touches? ── no ─► discard
     │                                             │ yes
     └────────── owner approves → models/rl/v(n+1), REACH_RL_MODEL=v(n+1)
```

The owner approves each new version (§14 Q5) until the loop has a track record.

## 11. Files and repos

```
laptop ~/kino_sim (twin repo; training lives with the twin)
  rl/
    README.md                how to train, play, evaluate, export
    world.py                 torch: boxes/cyls per env, ray casts, touch test
    generators.py            §6.3 room generators + loading build/worlds/*.json
    motion_torch.py          hardware/motion.py, vectorised (one source: config/rover.yaml)
    sensors.py               LiDAR / camera points / sonar sim (§6.4)
    shield_torch.py          §5, batched (must match rl_shield.py: parity test)
    env.py, env_cfg.py       Isaac Lab DirectRLEnv: obs, reward, resets, curriculum
    agents/rsl_rl_ppo_cfg.py §6.10
    tasks.py                 gym.register("Kino-TightDrive-v1", ...)
    eval.py                  rungs 1-2, writes the card
    export.py                checkpoint -> policy.npz + policy.onnx
    rooms_from_dumps.py      L3
    refit_motion.py          L2
    tests/                   motion parity vs hardware/motion.py, shield parity, ray-cast vs twin selftest

Jetson ~/rover
  nav/nodes/rl_policy.py, rl_shield.py      ROS-free
  nav/nodes/reach_node.py                    run_rl(), triggers, logging
  nav/tests/test_rl_shield.py, test_rl_policy.py
  models/rl/vNNN/{policy.npz, policy.onnx, card.md}
  RL_PLAN.md                                     this file
```

**Parity tests are the guard against sim-to-real drift in the code itself:** the torch motion
model must match `hardware/motion.py` to 1 mm over 10 s of random commands; `shield_torch` must
make the same decision as `rl_shield.py` on 10 000 random cases; the torch LiDAR must match the
twin's selftest on door80 within 1 cm.

## 12. Phases, tasks, effort

Effort = working sessions with Claude on the laptop/Jetson; owner time noted separately.

| phase | tasks | done when | effort |
|---|---|---|---|
| **R0** plan | this document; owner answers §14 | owner OK | done |
| **R0.5** demonstrations (parallel to R1) | `./rover demo`: record the owner's phone-teleop drives through tight spots — the same observation as §4.1 + the commands — into `logs/rl_demos/`; 20–50 drives | demos recorded | 1 session + **owner ~1 h driving** |
| **R1** environment | world + ray casts; generators (doorway, S-gap first); motion_torch + parity test; DirectRLEnv; reward; 50-iteration smoke | trains, reward rises, export works, parity tests pass | 2–3 sessions |
| **R2** full training | **behaviour cloning on the R0.5 demos first** (the policy starts by copying the owner), then RL from there (demo-augmented: a small imitation term in the loss for the first iterations); all generators, curriculum, randomisation, shield in the loop; tune | rung 1 | 2–4 sessions, GPU hours overnight |
| **R3** offline real worlds | load twin worlds in 2D; compare with tight_route | rung 2 | 1 session |
| **R4** on the Jetson | rl_policy, rl_shield (+ parity with torch), reach `run_rl`, logs, markers; twin suite | rung 3 | 2–3 sessions |
| **R5** real robot | charged pack, D555 power-cycled, owner on the floor | rung 4 | 1 session + **owner ~1 h** |
| **R6** learning loop | L2 refit, L3 rooms, L4 nightly with the gate | v002 better than v001 on the real failure list | 2 sessions |
| **R7** v2 inputs | fresh camera low points + sonars into the observation; retrain | rungs 1–4 again, low obstacles avoided, not just stopped for | 2 sessions |

While R1–R3 run, **the real robot is not needed** (only the laptop). R4 needs the Jetson for the
twin stack; the real `rover` container must be stopped then (twin and real stacks on one Orin at
load 13–17 is not tested — §14 Q4).

## 13. Risks and what we do about them

| risk | how it shows | what we do |
|---|---|---|
| sim ≠ real motion | twin fine, real rover misses turns | measured motion model + randomisation (§6.5–6.6); L2 refit; rung 4 is the judge |
| sim ≠ real sensors | policy confused by real noise | noise, dropouts, specks, sonar faults in training (§6.4); rung 3 uses the twin's full sensor sim |
| policy learns to lean on the shield | high shield rate, jerky | shield penalty in reward (§6.8); shield rate in the gate |
| reward hacking (e.g. stops forever to avoid −10) | no progress, "safe" | time penalty, give-up rule, success-rate metric; rung 1 checks progress |
| the torch world differs from the twin | rung 2 / 3 disagree | parity tests (§11); twin worlds load directly |
| CPU on the Jetson | late ticks | numpy forward pass < 1 ms; load logged per episode |
| low objects | touches things under 25 cm in v1 | shield sees camera + sonar points even in v1; v2 puts them in the observation |
| it's worse than tight_route somewhere | rung 3 | per-trigger switch: use RL only where it wins |

## 14. Decisions for the owner

| # | question | recommendation |
|---|---|---|
| Q1 | Scope: RL only for the hard last metres (this plan), or also try it as the open-floor controller instead of MPPI later? | hard metres first; open floor only if R5 numbers argue for it |
| Q2 | Training code lives in `~/kino_sim/rl/` (with the twin) — OK? | yes: same machine, same worlds, same motion yaml |
| Q3 | First version without v2 inputs (LiDAR policy, camera+sonar only in the shield), v2 right after? | yes: smaller first step, fewer sim-to-real unknowns |
| Q4 | During R4, stop the real `rover` container while the twin stack runs on the Jetson? | yes |
| Q5 | Nightly retrain (L4): automatic, or a new version waits for your OK? | your OK until 3 versions in a row pass rung 4 |
| Q6 | Real-floor tests (R5): which places in the house, and when is the pack charged? | the bedroom door, the chairs, a box gap; after a full charge |

## 14a. Other architectures considered (owner's question, 2026-11-06: "VLAs, or other ways?")

| approach | examples | fits this rover? | decision |
|---|---|---|---|
| classical stack + RL policy + shield | warehouse AMRs; RL locomotion (Spot, Unitree) | yes: tiny policy, proven pattern | **this plan** |
| **VLA** (vision-language-action: camera + text → motor commands, one large model) | RT-2, OpenVLA, π0, GR00T N1 | no, not for driving: billions of weights, cannot run at control rate on the Orin Nano, needs thousands of hours of demos; built for manipulation | **later, for the arm** ("pick up the bottle"), on a bigger computer, behind the same kind of shield |
| navigation foundation models (pretrained on many robots, go to a goal photo, explore) | ViNT, NoMaD | small enough for a Jetson, but camera-only: weaker than the LiDAR in tight spaces | optional experiment for exploration after R5 |
| vision-language navigation + 3D semantic memory | VLMaps, OK-Robot, ConceptGraphs | yes, on the Pi 5 brain | brain side, partly built (INTELLIGENCE_PLAN.md); decides *where*, this plan does *how* |
| **imitation learning** (copy the owner's driving) | behaviour cloning, Diffusion Policy | yes | **added: R0.5 demos + cloning before RL in R2** |
| world models (learn to predict, plan inside the prediction) | DreamerV3, Cosmos | research-heavy, GPU-heavy | not now |

Also added after this review: **moving obstacles** (people, a pet) in training (§6.3), and a
**memory (GRU) option** for dead ends (§4.3).

## 15. Words used here

| word | meaning |
|---|---|
| **policy** | the small neural network: sensor numbers in, (speed, turn) out |
| **reinforcement learning (RL)** | learning a policy by trial and error against a **reward** (a score per step) |
| **PPO** | the RL algorithm used (Proximal Policy Optimization): stable, standard, in RSL-RL |
| **episode** | one try: from a start pose until reached, touched, or time out |
| **environment (env)** | one simulated rover + room; 2048 run side by side on the GPU |
| **domain randomisation** | changing physics/sensor details every episode so the policy works on the real one too |
| **curriculum** | easy rooms first, harder as it succeeds |
| **shield** | a hand-written safety check that can override the policy |
| **sim-to-real gap** | the difference between the simulator and the real robot that can make a policy fail |
| **fine-tune** | continue training an existing policy on new data instead of starting over |
| **residual RL** | a second small network that learns a correction on top of a frozen policy |
| **ONNX / npz** | file formats for the trained network's weights |
