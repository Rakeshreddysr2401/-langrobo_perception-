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
| **3/4 — navigate** | give it a goal, it plans and drives there | ✅ **driving goals** — see below |

Phases 2–4 must also work in unfamiliar places; that is the point of the goal.

> **Just powered everything back on?** Follow **[STARTUP.md](STARTUP.md)** — the
> five boxes, the layer order, what to do physically when one fails, and the
> LangGraph Studio link. It is the one document to follow after a power cycle.

## Where it actually stands (2026-08-23)

**It navigates.** Autonomous goals of 1.00 m and 1.20 m were planned and driven,
finishing 4.9 cm and 3.6 cm from the target against a 5 cm tolerance, in 6 s and
17 s. A 1 m run was tape-checked: odometry read 95.8 cm, the floor said 93–95.

```bash
./rover camera && ./rover pose && ./rover fused && ./rover map && ./rover nav
./rover vlm                                    # phase 4: VLM pixel -> nav2 goal
./rover view                                   # RViz on the laptop

docker exec -it rover bash -lc 'source /opt/ros/jazzy/setup.bash; \
  export ROS_DOMAIN_ID=0; python3 -u /logs/goto.py 2.0 1.5 90'
```

`goto.py x y [theta] [--rel]`. It refuses to drive if the goal is unreachable, if
the route crosses a blocked cell, or if anything else owns `/cmd_vel`.

**What is NOT done, in the order it matters:**

| | why it matters |
|---|---|
| 🔴 **cuVSLAM diverges, silently** (§21) | three times so far. The pose drops to dead reckoning and every gate stays green. Check `vo_z` after every bring-up |
| 🔴 **obstacle avoidance untested** | no goal has yet been driven with something deliberately in the way. The costmap stops it *in theory* |
| 🟠 **loop closure never fires** (§16) | drift is never corrected, so range is limited to what raw odometry carries — about a room |
| 🟡 **the estimate has one measurement** (§15) | one tape reading. A −22° heading error from a loop test is still unexplained |
| 🟡 **the drift gate has never been run** (§2) | it was blocked on wheel telemetry, working again as of 2026-09-06 (§24) |

**Blind spots that no amount of tuning fixes.** The rover sees nothing below
10 cm, nothing above 24 cm, nothing outside 87°, and **nothing downward at all** —
there is no drop-off detection. Autonomous runs need a human watching.

**2026-09-06 — color camera on, for a VLM object-locator.** The D555 now also
streams RGB (`enable_color:=true`, 424x240x15) so the Pi 5 can hand a frame to
a VLM: "go to the red bottle" → coordinates → `/goal_pose` → nav2 drives
there. Verified both halves of that idea separately — cuVSLAM/nvblox/nav2 are
unaffected by the extra stream, and nav2 correctly accepts a published goal
and drives on it. The wheels didn't turn either way at first (§24), but a
full power-cycle of every device fixed that same day: a 0.7 m autonomous goal
was planned and driven end to end, `Goal succeeded`, no divergence (§26).

**2026-09-06 (later) — the VLM bridge landed, and the fleet was checked live.**
`./rover vlm` is a real layer now: `phase4/nodes/image_bridge.py` publishes the
color frame as JPEG for the Pi 5's `look()`, and `phase4/nodes/pixel_to_goal.py`
turns a VLM-picked pixel into an odom-frame nav2 goal. The matching Pi 5 fix
went in the same day — `ros2_bridge.py` had `frame_id = "map"` hardcoded in
both `get_current_pose()` and `_nav_worker()`, and **this rover has no map
frame**, so every `navigate_to_pose`/`approach_*` call had been failing
silently since it was written. Both are `"odom"` now.

All five boxes were then checked live and all five answered — including the
**Mac mini, which is not idle**: it serves `gemma-4-12B` (multimodal) to the
Pi 5's brain and is reachable from both machines. Every rate passes its gate
except depth, at 8.3 Hz against 10 (§27). Full readout: **[FLEET_STATUS.md](FLEET_STATUS.md)**.

**Talking to it: Telegram works today, voice does not.** The Pi 5 voice stack
is built and was verified end-to-end earlier the same day, but it cannot start
right now — the Pi 5 has neither a microphone nor a speaker attached (§28).

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
| **[FLEET_STATUS.md](FLEET_STATUS.md)** | every box checked live — Jetson, Pi 5, Mac mini, ESP32, D555 — and what blocks voice |
| **[READINESS.md](READINESS.md)** | the cross-cutting view — every subsystem's measured values against what is missing, and **what actually blocks unattended operation** |
| **[JETSON_LOAD.md](JETSON_LOAD.md)** | the box itself — power mode, disk, what's running, and what's dev tooling vs. the rover |
| **[VOICE_PLACEMENT.md](VOICE_PLACEMENT.md)** | why STT/TTS moved to the Pi5 — the full-stack GPU/RAM measurement that forced the call, and where the build lives |
| **[phase2/LAPTOP.md](phase2/LAPTOP.md)** | watching it from a laptop in RViz |
| **[phase1/teleop/README.md](phase1/teleop/README.md)** | the hold-to-move web control, and its units |
| **[PLAN.md](PLAN.md)** | the plan of record, and what was settled in the design interview |
| **[knowledge/](knowledge/)** | the concepts — frames, visual odometry, fusion, costmaps, planners, **and §09: how these faults were actually found** |

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
