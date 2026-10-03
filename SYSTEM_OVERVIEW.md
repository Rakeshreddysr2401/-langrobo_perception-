# System overview: the whole robot, every machine, and how clean it is (2026-10-03)

The one page for "where are we and how does it all fit". It covers the four
repos on three computers, how they connect, the goal, and an honest check of
how tidy each one is. For what is open and what to do next, see
[OPEN_ISSUES.md](OPEN_ISSUES.md). For the numbers, see
[LOCALIZATION.md](LOCALIZATION.md).

Everything below was checked live on 2026-10-03: all three machines reached
over ssh, git state read, the Pi 5 tests run, and the laptop's copies diffed
against this repo.

---

## 1. The goal

**Mitra**: a home robot you talk to ("Mitra, go near the bottle and tell me
what's on it"). It understands the request, finds the thing from its photos,
drives there exactly (to within about 1 cm) without touching anything, and
reports back by voice or Telegram.

The order it is being built in, from the README and INTELLIGENCE_PLAN.md:

1. **Body + pose** (M1–M4): know where it is to about 1 cm. ✅ Done and live.
2. **Safe navigation** (M5): nav2 plus an exact finish, tight gaps. 🟡 Works down to a ~50 cm gap. 45 cm is open.
3. **Brain uses the body** (B1–B3): moves, photo grounding, memory. ✅ Floor-tested. Memory is now the photo log (2026-10-02).
4. **Fast eyes / smart search / learning** (B4–B6): ⬜ next.
5. Later: a parking assistant, then an arm.

## 2. The machines and the repos

| machine | address | repo (path → remote, branch) | its job |
|---|---|---|---|
| **Jetson Orin Nano** | 192.168.1.15 | `~/rover` → `-langrobo_perception-`, `rover-v1.1.7` | **the body's brain**: sensors, pose, map, nav2, exact moves, pixel→goal. Everything runs in one Docker container (`rover`, image `orin-nav:1.1`) |
| **Pi 5** | 192.168.1.16 | `~/ros2_ws` → `pi5_ros2_ws`, `dev-1.4.1` | **the mind**: LangGraph brain (3 agents), voice (STT/TTS/wake word), Telegram, teleop web page, micro-ROS agent for the ESP32 |
| **Laptop** | DHCP (192.168.1.17 today) | `/workspace/mitra_sim` → `mitra_sim`, `main` | **a window and a twin**: RViz for the real robot, plus the Gazebo "Mitra twin" world |
| | | `/workspace/ros2_ws/src/rover_sim` → `rover_sim`, `dev-0.1.1` | the older Gazebo sim (mecanum body, its own nav2). Still what `fleet.sh sim` starts |
| Mac mini | DHCP, `singireddys-mac-mini.local:8080` | (none) | llama.cpp: Gemma 4 12B, LLM + VLM, `--jinja --parallel 4` |
| ESP32 | micro-ROS over WiFi UDP 8888 | firmware in `~/rover/phase1/firmware/` | 4-wheel PID, encoders, 500 ms cmd_vel watchdog |
| D555 camera | 192.168.11.55 (PoE, own subnet) | (none) | stereo IR, depth, colour, **and the gyro** (OPEN_ISSUES #2) |
| RPLidar C1 | USB on the Jetson | driver built by `lidar/build.sh` | 360° scans at 10 Hz. The pose reference |

## 3. How they connect

```
            voice / Telegram / phone
                     │
   ┌─────────────────▼──────────────────┐      HTTP (OpenAI API)     ┌──────────────┐
   │ Pi 5  ~/ros2_ws                    │ ─────────────────────────► │ Mac mini     │
   │  agent_node ─ LangGraph:           │   chat / local_agent /     │ llama.cpp    │
   │   chat · local_agent · navigate    │   navigate + vision slot 3 │ Gemma (VLM)  │
   │  stt_node · tts_node (voice)       │ ◄───────────────────────── └──────────────┘
   │  teleop_web.py :8091 (MANUAL/AUTO) │
   │  micro-ROS agent ◄─────────────────┼──── WiFi UDP 8888 ──── ESP32 (wheels)
   └───────────────┬────────────────────┘                          ▲ /cmd_vel
                   │ ROS 2 DDS, domain 0, plain multicast            │ /wheel_state
   ┌───────────────▼────────────────────────────────────────────────┴───┐
   │ Jetson  ~/rover  (container "rover")                               │
   │  D555 ─► vo_node (cuVSLAM)  gyro_node   RPLidar ─► lidar_odom      │
   │                 └────────► fusion2 ─► /odom + TF ◄────┘            │
   │  slam_toolbox (map→odom)   depth_gate/turn_gate ─► nvblox (map)    │
   │  nav2 (Smac lattice + MPPI) ─► reach ─► goal_exec ─► turn_shaper   │
   │  image_bridge (JPEG for look())   pixel_to_goal (photo px → goal)  │
   └───────────────┬────────────────────────────────────────────────────┘
                   │ DDS (subscribe only)              ssh: ./rover view pushes the config
   ┌───────────────▼─────────────────────────────┐
   │ Laptop  RViz (~/rover_live.sh + .rviz)      │
   │         mitra_sim Gazebo twin, DOMAIN 42 ──► a 2nd copy of the Jetson stack (domain 42)
   └─────────────────────────────────────────────┘
```

**The wire between the Pi 5 brain and the Jetson body** is a short list of
topics. The full contract is in the Pi 5's INTEGRATION_GAPS.md §7:

| topic | from → to | meaning |
|---|---|---|
| `/camera/color/image_raw/compressed` | Jetson → Pi 5 | the photo `look()` takes. Its stamp is the key for everything below |
| `/vision/pixel_snapshot` | Pi 5 → Jetson | "hold this photo's depth + pose" (24 held) |
| `/vision/pixel_query` → `/vision/pixel_result` | Pi 5 ↔ Jetson | the VLM's pixel/box on that photo → object x,y + a standoff goal |
| `/reach/goal`, `/goal_exec/turn`, `/goal_exec/goal` (+ `/status`, `/cancel`) | Pi 5 → Jetson | go there: nav2 + exact finish, or one exact move |
| `/cmd_vel` | Jetson or teleop → ESP32 | the wheels (via the Pi 5's micro-ROS agent) |
| `/teleop/mode` (MANUAL cancels reach + goal_exec) | Pi 5 → all | the human override |

**Who starts whom:** `./rover up` on the Jetson brings up every layer, each
gated on the one below. It then checks the ESP32 and the Pi 5, starts Studio,
and runs `./rover view`, which finds the laptop by scanning for the host with
`~/rover_live.sh`, pushes the RViz config, and starts RViz. From the Pi 5,
`scripts/fleet.sh rover` runs the same `./rover up` if the stack is not up.
The Pi 5's own services (brain, voice, micro-ROS, teleop) start at boot from
systemd.

**What one request goes through:** "go near the bottle" → STT on the Pi 5 →
`chat` hands off to `navigate` → `ask_photos` (the VLM picks a photo + box) →
`/vision/pixel_query` → `pixel_to_goal` uses *that photo's* depth and pose →
goal → `/reach/goal` → nav2 route + `goal_exec` exact finish → arrival photo
check → spoken report.

## 4. How each repo is organised

### Jetson `~/rover` (this repo): 132 tracked files

| where | what |
|---|---|
| `rover` | the 1,134-line launcher. Every layer and every drive command (`up`, `status`, `goto`, `navto`, `reach`, `pass`, `record`, `grade`, `view` …) |
| `description/` | the one source of the rover's measurements → URDF + STEP CAD, with a drift check |
| `phase1/` | the pose: vo_node, gyro, lidar_odom, fusion2, health, calibration; test harness (record/grade); ESP32 firmware; teleop page (deployed to the Pi 5 by copy) |
| `lidar/` | RPLidar driver build + yaw/lag calibration |
| `phase2/` | mapping: nvblox launch, slam_toolbox config, turn/depth gates, RViz config + laptop docs |
| `phase3/` | nav2 config + BT, goal_exec, reach, pivot_goto, obstacle sources, a C++ costmap plugin, simulators (`sim_goal_exec`, `sim_gap_pass`) |
| `phase4/` | VLM bridge: image_bridge, pixel_to_goal, detections_3d |
| top-level `.md` | README, STARTUP, OPERATIONS, ARCHITECTURE, OPEN_ISSUES, LOCALIZATION(+_GAPS), SENSOR_FUSION_PLAN, NAV_PLAN, INTELLIGENCE_PLAN, ROVER_BUILD_PLAN |
| `docs/archive/` | the history (phase logs, old TODO/READINESS) |

### Pi 5 `~/ros2_ws`: 185 tracked files

| where | what |
|---|---|
| `src/langrobo_core/` | **pure Python brain, no rclpy**: `registry.py` (one AgentSpec per agent), `prompts.py`, `graph/`, `agents/`, `tools/` (look, movement, approach, photos, photo_recall, survey, locate, telegram, web), `services/` (llm, config, telegram, permissions, health, object_memory …), 434 tests |
| `src/langrobo_ros/` | the ROS shim: `agent_node.py`, `ros2_bridge.py`, launch, systemd; wake-word models |
| `src/pi5_voice_pkg/` | STT, TTS, Bluetooth audio owner |
| `scripts/` | `fleet.sh` (whole robot), `run_*.sh` (systemd entry points), voice/wake-word tools |
| 12 top-level `.md` + `docs/voice/` | CLAUDE, README, HOW_IT_WORKS, ARCHITECTURE_LLD, FIND_AND_GO, INTEGRATION_GAPS, OPERATIONS, TODO, TELEGRAM, NETWORKING, MAC_MINI_TASKS, INTENT_ROUTING_PLAN; voice ×4 in `docs/voice/` |

### Laptop

| where | what | source of truth |
|---|---|---|
| `~/rover_live.sh`, `~/rover_live.rviz` | the RViz launcher + config | **this repo**, `phase2/rviz/`. Pushed by `./rover view`; identical today |
| `/workspace/mitra_sim` (27 files; also cloned on the Jetson at `~/mitra_sim`) | the twin: Gazebo world + measured body + sensors (`./mitra up/rviz/goal/scenario/test`), `jetson/sim_stack.sh` (the Jetson half), tests, `docs/STATUS.md` | its own repo |
| `/workspace/ros2_ws/src/rover_sim` | the older sim (mecanum, own nav2, sim/real contract gate) | its own repo |
| `~/old/` | leftovers archived 2026-10-03 (old sim copy, old RViz configs, stray colcon output, a stale LAPTOP.md); see its README.txt | nothing reads them |
| `~/ros2_ws` (mycobot_ros2, ros2_fundamentals_examples) | learning / future arm material | unrelated to the rover |

## 5. Is it clean? The health check

### What is good

- **All three git trees are clean.** Jetson and Pi 5 are in sync with their
  remotes, and mitra_sim is clean on `main`.
- **Pi 5 tests: 434 passed in 18 s.** pyflakes is clean apart from one deliberate `noqa` probe.
- **Nothing has drifted between copies:** the deployed teleop
  (`~/langrobo_teleop/teleop_web.py`) matches `phase1/teleop/`, the laptop's
  RViz files match `phase2/rviz/`, and `~/mitra_sim` on the Jetson is now a
  clone of the twin repo.
- **Clear boundaries:** `langrobo_core` imports no rclpy (it is testable
  anywhere), the measurements have one source (`description/params.yaml`),
  and the laptop holds no source of truth for the real robot.
- **The `.gitignore` files are well kept.** Bags, CSVs, weights and build
  output are excluded, with the reason written down. No broken links between
  the top-level docs here.
- **The twin is a real step forward.** It runs the Jetson's *unchanged* code on
  domain 42 and judges every run on the true pose. On its first night it found
  two real bugs (far goals failing, a turn ending 0.2 cm from a box), both
  fixed in 302de1b.

### What was not clean, and what was done (all 13 resolved 2026-10-03)

| # | where | issue | done |
|---|---|---|---|
| C1 | Jetson `ARCHITECTURE.md` | stale: `fusion.py`, NavFn + RPP, "not an EKF" (contradicting its own §5), none of goal_exec / reach / pixel_to_goal / gates / the twin, open faults pointed at the archived TODO | ✅ §0 stack, §1 diagram, §2 topics, §4 node table refreshed; points at OPEN_ISSUES.md and here |
| C2 | Jetson `OPEN_ISSUES.md`, mitra_sim `docs/STATUS.md` | headed 2026-10-04, work done 2026-10-03 | ✅ both say 2026-10-03 (commit 53c3a99's message still says 10-04; history is not rewritten) |
| C3 | Jetson `README.md` | status table dated 2026-09-26 | ✅ dated 2026-10-03; close quarters, the brain (B1–B3) and the twin added |
| C4 | Laptop `rover_sim` | 4 commits never pushed | ✅ pushed (`dev-0.1.1`, 06b8951) |
| C5 | two sims | `fleet.sh sim` starts the OLDER `rover_sim`; the twin is `mitra_sim` | ✅ `fleet.sh` header + its `sim` output and the Pi 5 CLAUDE.md say which is which. Retiring `rover_sim` stays the owner's call |
| C6 | `~/mitra_sim` on the Jetson | hand copy of the twin's `jetson/` | ✅ now a git clone; `sim_stack.sh up` warns when it is not a clone, has local edits, or is behind origin; `sim_stack.sh` is executable in git (mitra_sim f74337a). The old copy is at `~/mitra_sim.handcopy-2026-10-03` (identical; delete it when you like) |
| C7 | Pi 5 repo root | stray screenshot, `ESP_32_frimware/` lock-file dir, `.env*` world-readable (644) with live keys | ✅ screenshot removed (it was an empty Studio page); dir moved to Pi 5 `~/old/`; `.env` and both backups `chmod 600` (kept, not deleted) |
| C8 | Pi 5 `TODO.md` | header said `dev-1.3.8` | ✅ `dev-1.4.1` |
| C9 | Pi 5 `object_memory.py` / `survey.py` | looked dead | ✅ **not dead**: the survey is an opt-in feature (`LANGROBO_PHOTO_SURVEY=1`, default off), `survey.py` also owns the vision-tool slot every photo prompt uses, and approach uses object_memory's helpers. The comments in `look.py` / `approach.py` that said every photo feeds memory now say it is opt-in |
| C10 | Pi 5 `INTEGRATION_GAPS.md` §1 | listed `/vision/detections_3d` etc. as unanswered | ✅ marked closed by removal: the brain no longer reads detections_3d, target or music (tools gone); the Jetson publisher is opt-in (`./rover detect`) and returns with B4 |
| C11 | Pi 5 `tools/approach.py:566` | "unused" `Twist` import | ✅ **false finding**: it is a deliberate availability probe marked `# noqa: F401`; pyflakes ignores noqa. Left as is |
| C12 | Laptop home | leftovers | ✅ moved to `~/old/` with a README.txt (nothing deleted; `~/LAPTOP.md` was a stale copy that differed from `phase2/LAPTOP.md`) |
| C13 | doc sprawl | 16 top-level docs on the Pi 5 | ✅ the four voice docs moved to Pi 5 `docs/voice/`; all 31 files referencing them follow. 12 top-level docs remain there |

Commits: Jetson (this one), Pi 5 0cd4503, mitra_sim f74337a, rover_sim pushed. Pi 5 tests 434/434 after.

### What is clean in code but open in behaviour

Unchanged from [OPEN_ISSUES.md](OPEN_ISSUES.md), most damaging first:

1. The Jetson is CPU-bound. EventsExecutor helped; fusion2 + lidar_odom are
   not yet graded on it.
2. The gyro rides the camera link: a camera stall freezes the pose.
3. Things under 3 cm (slippers, a laptop) are invisible, so the rover drives
   over them. Found by the twin.
4. Tight gaps pass 1–4 cm from the obstacle. 45 cm cannot be promised at a
   2.5 cm map resolution.
5. The VLM is slow (~9 tok/s) and loose. Mac access is needed for `--swa-full`
   and speculative decoding.
6. Power: the turn slide moves with the battery, and the PoE injector shares
   a socket with the ESP32.

## 6. Where we are, in one paragraph

The body is precise and works: the pose is good to about 1 cm, exact moves are
live, and nav2 plus an exact finish reach goals to about 1 cm. The brain can
already be asked by voice or Telegram to find an object from its photos and go
to it, with an arrival check. The work has moved from "does it work" to
"is it safe in clutter and fast enough to talk to". The open problems are
physical (low objects, tight gaps, CPU load, a gyro on a fragile link, power)
and the VLM's speed. The new twin is the tool for testing the first two
without risking the robot. The code is in good shape, and the tidying found
on 2026-10-03 (C1–C13) is done. What is left is behaviour, not housekeeping:
the list above and OPEN_ISSUES.md.
