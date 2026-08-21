# rover

**Goal: a rover that reaches its goal in known and unknown environments.**

Four phases, each standing on the one before it:

| phase | goal | status |
|---|---|---|
| **1 — perception** | know where it is | ✅ **complete** — all gates pass |
| **1b — publish it** | a pose nav2 can navigate on | ✅ **complete** — `/odom` at 20 Hz |
| **2a — see it** | RViz, the pose, the track it draws | ✅ **complete** |
| **2b — build it** | map the room as it drives | ✅ **complete** — nvblox |
| 2c — keep it | a map that survives a power cycle | deferred by choice |
| 2d — localize | recognise a room mapped before | deferred by choice |
| **3/4 — navigate** | click a goal, it plans and drives there | 🟡 **running, no goal driven yet** |

Phases 2–4 must also work in unfamiliar places; that is the point of the goal.

---

## Where to look

| document | what is in it |
|---|---|
| **[OPERATIONS.md](OPERATIONS.md)** | **the runbook** — bring-up, gates, calibration, troubleshooting by symptom |
| **[ARCHITECTURE.md](ARCHITECTURE.md)** | the system as built — machines, topics, frames, and *why each decision went that way* |
| **[PHASE1.md](PHASE1.md)** | perception — every number, technique and fault found |
| **[PHASE2.md](PHASE2.md)** | mapping — nvblox, the map, and seeing it |
| **[PHASE3.md](PHASE3.md)** | navigation — nav2, and everything shaped by the pivot fault |
| **[TODO.md](TODO.md)** | open faults, and the dead theories kept so they are not re-litigated |
| **[PLAN.md](PLAN.md)** | the plan of record, and what was settled in the design interview |
| **[knowledge/](knowledge/)** | the concepts — frames, visual odometry, fusion, costmaps, planners |

---

## Quick start

```bash
./rover camera      # the D555 alone; verifies the IR emitter is OFF
./rover pose        # + cuVSLAM and the gyro
./rover fused       # + the fused pose -> /odom and TF odom -> base_link
./rover map         # + nvblox: build the room as it drives
./rover nav         # + nav2: plan a route and drive it
./rover status      # what is alive right now
```

Then drive it from a phone at **`http://192.168.1.16:8091`** — and **flip it to
MANUAL**, it defaults to AUTO and the buttons do nothing until you do.

Watch it from a laptop with RViz — see **[phase2/LAPTOP.md](phase2/LAPTOP.md)**.

To measure rather than just watch:

```bash
./rover compare --return                  # hand-pushed, graded
python3 -u /logs/loop_test.py             # driven: does the line close?
python3 -u /logs/map_view.py              # the map, as text, in the terminal
```

---

## What Phase 1 achieved

Four sensors, each asked only what it can honestly answer, and each covering for
the others when they fail.

| gate | target | result |
|---|---|---|
| scale | 2.00 m ±5% | ✅ 195.4 cm (−2.3%) |
| drift, hand-pushed | ≤ 10 cm | ✅ 2.5 cm |
| **drift, driven hard** | ≤ 10 cm | ✅ **4.9 cm** through 12 teleports |
| stationary | no phantom motion | ✅ 0.08° over 161 s |
| heading, 360° spin | ≤ 10° | ✅ 3.68° |
| teleop | moves and stops | ✅ balance 1.00 |

**The result that matters:** driven hard in a room the camera could barely see,
cuVSLAM ended **79.6 cm** from the start and the wheels **55.7 cm** — the fused
estimate ended **4.9 cm**. Beating both of its own inputs is the whole point.

| when this fails | this carries it |
|---|---|
| cuVSLAM goes blind, teleports, or dies | wheels + gyro |
| wheels slip in a turn | gyro heading |
| gyro drifts | cuVSLAM, straight-line only |

---

## The hardware

| machine | role |
|---|---|
| Jetson Orin Nano | cuVSLAM, pose, fusion — all inside `orin-nav:1.1` |
| Pi 5 | micro-ROS agent, teleop web |
| ESP32 | 50 Hz PID, 4× encoders, 2× BTS7960 |
| RealSense D555 | stereo IR + depth + IMU, over **Ethernet**, not USB |

**The container has no build recipe.** `orin-nav:1.1` was made by `docker
commit`, not from a Dockerfile. Never install into it; never delete it. `phase1/`
is mounted read-only, so nodes are edited on the host and the layer restarted.

---

## The lesson worth carrying forward

Almost every fault found in Phase 1 **presented as healthy**: an IR emitter
silently on while every rate was green, a camera streaming at 30 Hz while
refusing option changes, a filter that appeared to fuse and did nothing, a fused
pose that was *worse than its own worst input* because it kept listening to a
sensor that had gone blind.

That is why the tooling is shaped the way it is — read parameters back rather
than trusting they were set, show every source separately rather than one
confident number, and let the board report its own loop rate rather than
inferring it. See [PHASE1.md](PHASE1.md) §7 for the full table.
