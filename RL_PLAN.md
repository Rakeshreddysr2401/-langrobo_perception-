# RL_PLAN — a learned driver for tight spaces, trained in Isaac Lab, improved from every real drive

Written 2026-11-06 for the owner: *"I want my robot to move more intelligently — can we use
reinforcement learning with Isaac Lab and Sim, so that once it moves it learns?"*

Status: **plan; nothing built yet.** Laptop checked 2026-11-06: Isaac Lab 3.0.0 + RSL-RL 5.5.1
installed and smoke-tested (`~/kino_sim/docs/VERSIONS.md`), RTX 4050 6 GB idle.

---

## 1. What we want to achieve (and how we will know)

The classical stack is good in open space and weak in **close quarters**. The twin baseline of
2026-11-01 (`~/kino_sim/docs/NAV_NEXT.md` §10, 11 worlds) shows where:

| world | today | target for the learned driver |
|---|---|---|
| door49 (real bedroom door, 49 cm past the bed) | failed 122 s; 87 s after the tight_route fixes (1 run) | reached < 45 s, 0 touches |
| chairs44 (44 cm between real chairs) | failed 165 s (tight route timed out) | reached < 60 s, 0 touches |
| corridor_s (60 cm S-corridor) | failed 166 s | reached < 45 s, 0 touches |
| apt_hard | 6/6 but **touched** a wall at a door jamb (-0.3 cm) | 6/6, 0 touches |
| gap45 / door49 | nav2 dithered 38–43 s before handing over | hand-over < 5 s |
| gap36 (impossible) | "too tight" in 3.8 s | unchanged: still refused fast (widest.py stays) |

Success is measured the way the twin already judges: **a touch fails the run** (true outline vs
world), time-to-goal and minimum clearance are recorded. Then the same on the real floor, owner
watching.

## 2. Where RL sits — and what stays classical

The rule every working robot follows (NAV_GUIDE §7, INTELLIGENCE_PLAN §1): **a model chooses,
classical code keeps it safe.** RL gets one job — driving the last few metres through tight
places, where hand-written rules spend minutes — and never gets the wheels without a check.

```
 Pi 5 brain (LLM/VLM)          "go to the bedroom"            WHAT     (seconds)
        │ /reach/goal
 reach_node                    which tool for this situation  STRATEGY (~1 s)
   ├─ open floor  ───────────► nav2 (Smac lattice + MPPI)     unchanged
   ├─ last 1.2 m  ───────────► goal_exec (exact, 1.4 cm)      unchanged
   ├─ too tight   ───────────► widest.py says so, fast        unchanged
   └─ NARROW / no path / ────► rl_drive  ◄── NEW: learned policy, 10 Hz
      dithering / pinned            │  proposes (vx, wz)
                                    ▼
                               rl_shield ◄── NEW: sweeps the body outline along the proposal
                                    │         (with the learned slide) against LiDAR + camera
                                    │         points; unsafe → slows / picks a safe nearby
                                    ▼         command / stops. Never switched off.
                               /cmd_vel → turn_shaper → ESP32 → wheels
   fallback when rl_drive gives up: tight_route → way back → look → wait   (all unchanged)
```

Nothing is removed. `REACH_RL=0` gives today's robot exactly.

## 3. The policy

| | |
|---|---|
| **observation** | 72 LiDAR beams (5°), each as **clearance from the body outline** along that beam (so the policy knows its own shape), clipped 0–3 m, last 2 scans; goal distance + bearing (sin, cos) + final-heading error; measured vx, wz; last action; **the live pivot estimate (px, py) and turn gain k** from goal_exec (§6 L1) |
| **action** | vx ∈ [-0.12, 0.25] m/s, wz ∈ [-0.9, 0.9] rad/s, 10 Hz |
| **network** | MLP 256-128-64, ~60 k parameters; < 1 ms on the Orin's CPU (onnxruntime) — no GPU taken from the camera/nvblox |
| **v2** | per beam, the min of LiDAR and the camera's **fresh** low points (`tight_route.Evidence`: seen from two viewpoints, so a glossy-floor speck is not one) so shoes and chair feet under 25 cm are in the observation too; the two front sonars as two more ranges (the nose zone the camera cannot see) |
| **not raw images** | the depth/colour image straight into the network was considered (owner, 2026-11-06) and left out: GPU cost on the 6 GB laptop and on the Orin, and images transfer sim → real far worse than points. The camera enters as points |

**Why this helps where the map fails (owner's question, 2026-11-06).** The policy acts on what
the sensors see *now*, 10 times a second, with no search to time out (door49, chairs44) and the
slide learned (apt_hard jamb). It does not read the remembered map, so stale specks that closed
corridor_s do not block it. It does **not** fix things no sensor sees (< 3 cm) or replace the map
for room-to-room routes.

## 4. Training — Isaac Lab on the laptop

**Framework:** Isaac Lab 3.0 DirectRLEnv + RSL-RL PPO (the installed, tested path; train, play and
ONNX export are Isaac Lab's own scripts).

**The world is computed on the GPU in torch, not as 3D meshes.** Every one of ~2048 parallel
rovers gets a **new random room at every reset**; meshes cannot be rebuilt that cheaply, and the
LiDAR is one plane at 25 cm anyway (exact ray–segment / ray–circle casts). The full Isaac Sim
twin is where the result is then judged (§7).

**Rooms (procedural, per reset):** a doorway 40–90 cm (with or without a door leaf open at 0–90°),
S-gaps (bottle + chair leg, as gap45), 48–70 cm corridors with bends, dead-end pockets (back out,
then turn), furniture boxes, 3–7 cm chair legs, start pose against a wall (the "pinned" case).
Plus the kino_sim worlds (`build/worlds/*.json`) converted to 2D — held out for testing.

**The rover moves by the measured motion model**, not an ideal diff drive:
`~/kino_sim/hardware/motion.py` (./rover response, 2026-09-26) — 1 cm/s side deadband, in-place
turn `k·(0.52|wz| − 0.21)`, 22 % turn following while driving, 0.2 s lag, 2–3 % noise, the turn
**slides about a pivot**. Vectorised in torch.

**Domain randomisation — so the policy works on every battery and floor:**

| what | range | why (measured) |
|---|---|---|
| pivot (left / right) | fresh → weak packs, `config/rover.yaml` | 6–33 cm slide per 90° (TODO §43) |
| turn gain k | 0.65–1.05 | pack state |
| lag | 0.1–0.3 s | |
| LiDAR | ±1 cm noise, 2 % dropped beams, 0–0.1 s latency | C1 |
| pose | ±1 cm, ±0.5° per step | fusion2 sd 0.6 cm |
| false points | 0–8 specks near the path, gone on the next look | glossy floor, people walking past (S10, corridor_s) |
| sonar | floor echoes, silent on soft things | sonar_witness.py rules |

**Reward (per step):** + progress along the **walking distance** to the goal (grid BFS at reset,
as tight_route's guide); +10 reached (within 5 cm, 10°); **−10 and end on contact**; −0.01 time;
− clearance under 3 cm; − jerk; reverse ×3 cost (rear blind below 25 cm, as tight_route).

**Curriculum:** gaps start at 70 cm and narrow toward 42 cm as success passes 80 %.

**Budget:** no rendering, so 2048 envs fit the 6 GB GPU; first policy expected in 1–3 h.

## 5. Deployment on the Jetson

| file | does |
|---|---|
| `phase3/nodes/rl_drive_node.py` | loads `models/rl/<version>/policy.onnx`; `/rl_drive/goal` → drives at 10 Hz on `/odom` + `/scan` (+ camera points), publishes `/cmd_vel` only while active (like goal_exec); `/rl_drive/status` JSON |
| `phase3/nodes/rl_shield.py` | no ROS: the proposal's swept outline over 0.6 s, slide included, vs points at goal_exec's MARGIN; logs every intervention |
| `reach_node.py` | new step before the tight route when narrow / no path / dithering / pinned; `REACH_RL=0/1` |
| `models/rl/<version>/card.md` | what it was trained on, its test numbers, the git hashes |
| `./rover rl` | starts the node, like every other layer, with its own gate |

## 6. How it improves — "once it moves, it learns"

On-robot RL from scratch is not an option: millions of tries, thousands of contacts, a person
resetting it. So the robot learns in **five loops**, fastest first:

| loop | what learns | from | when | risk |
|---|---|---|---|---|
| **L1 live adaptation** | the policy's input carries goal_exec's learned pivot and k; it was trained across all packs, so it adapts as the battery drains | every turn | instantly | none (no weights change) |
| **L2 sim fitted to reality** | the motion-model ranges the sim trains on | logged `/cmd_vel` vs the LiDAR-measured motion (`./rover record` + `grade`) | nightly | none |
| **L3 failures become training rooms** | the policy, on the exact places it failed | every rl_drive goal that failed, took > 60 s or needed the shield: its points are saved (as `logs/tight_dumps`, abfc30c) and rebuilt as a sim room; 20 % of training | nightly | none |
| **L4 retrain + gate** | v(n) → v(n+1), from the last checkpoint, ~1–2 h on the laptop | L2 + L3 | nightly | **deployed only if** it beats v(n) on the held-out set AND the twin suite has no new touch |
| **L5 residual online RL** | a small correction on top of the frozen policy, while driving, inside the shield | live | later, only if L1–L4 plateau | low (shield) |

```
 real drive ──log──► laptop: refit sim (L2) + rebuild failure rooms (L3)
     ▲                         │
     │                  retrain from v(n) (L4) ──► held-out test ──► twin suite (0 touches)
     │                                                                    │ pass
     └──────────────── v(n+1) on the rover next morning ◄─────────────────┘
```

**Logged per rl_drive goal** (`logs/rl_episodes.jsonl`): goal, start pose, result, seconds, min
clearance, shield interventions, policy version, pack estimate, and the scan points of the hard
moments. Without this log there is nothing to learn from — it ships with the first node.

## 7. The test ladder (each step must pass before the next)

1. **Sim, held out:** ≥ 90 % reached on unseen synthetic rooms with gaps ≥ 44 cm, 0 contacts;
   gap 40 cm refused or not attempted.
2. **kino_sim worlds in 2D:** door49, chairs44, corridor_s, gap45, dead_end vs tight_route on
   the same points (`phase3/tools/replay_tight.py` style).
3. **Twin, full stack** (`TWIN=1 ./rover up`, `./sim scenario`): the 11-world suite vs the
   2026-11-01 baseline. Must be: no touch anywhere, faster in the tight worlds, no worse elsewhere.
4. **Real floor, owner watching:** gap45 boxes, the bedroom door, chairs; charged pack first;
   teleop MANUAL is the stop.

## 8. Phases

| phase | work | done when |
|---|---|---|
| **R0** | this plan | committed |
| **R1** | Isaac Lab env in `~/kino_sim/rl/`: torch world + LiDAR + motion model + rewards; 50-iteration smoke run | trains, reward rises, ONNX exports |
| **R2** | full training with curriculum + randomisation | ladder step 1 |
| **R3** | 2D replay of the kino_sim worlds | ladder step 2 beats tight_route |
| **R4** | `rl_drive_node` + `rl_shield` + reach hook + episode log | ladder step 3 |
| **R5** | real robot | ladder step 4 |
| **R6** | L2–L4 nightly loop | v2 measurably better than v1 on the real failure list |

## 9. Limits, said up front

- **Low things.** The policy's v1 eyes are the LiDAR plane at 25 cm. The shield uses LiDAR +
  camera points, so low things still stop it, but the policy cannot plan around them until v2.
  Things under 3 cm (slippers, a tablet) stay invisible to everything (NAV_NEXT §10).
- **No drop-off sensing** (ROVER_BUILD_PLAN §9.6) — unchanged; a person watches.
- **Sim-to-real.** The motion model is from 2026-09-26 measurements; L2 exists because it will
  drift. A policy that only works in the sim fails ladder step 4 and is not used.
- **Not everywhere.** nav2 keeps open-floor driving; a learned local controller for the whole
  house is a later question, decided by numbers from R4–R5.
- **The CPU.** The node is tiny, but the Jetson runs at load 13–17; the episode log notes the
  load so a slow tick is not blamed on the policy.
