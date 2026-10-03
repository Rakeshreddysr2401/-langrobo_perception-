# Open issues — where things stand, 2026-10-03

## 2026-10-03 (night) — the Mitra twin (simulator) and what it found

A laptop Gazebo twin of this rover: github.com/Rakeshreddysr2401/mitra_sim
(private). The laptop is only the WORLD (measured body, lidar, depth camera,
gyro, the wheels' real slide); THIS code is the brain, run as a second
instance in the rover container on ROS domain 42 (`~/mitra_sim/jetson/
sim_stack.sh up|down|status`), beside the real stack on domain 0, with its
learned state in /tmp/mitra_state (ROVER_STATE_DIR). Every run is judged on
the true pose against the true obstacles. Results: mitra_sim docs/STATUS.md.

| found by the twin | status |
|---|---|
| goals > ~4 m away failed at once: the global costmap is an 8 x 8 m rolling window ("outside bounds") | fixed 302de1b: reach drives legs of <= 3.2 m |
| turn-and-shuffle came 0.2 cm from a box while the pivot was unlearned | fixed 302de1b: goal_exec +3 cm until learned |
| things under 3 cm (slippers, a laptop) are invisible to nvblox and the lidar: driven over | **open** -- a 1 cm slice sees them but floor noise then boxes the rover in (rejected); next: 2 cm, or a close-range floor sensor |
| gap passes 1-4 cm from the chair leg (S-gap 45/50/60) | **open** -- MPPI margin 4 cm froze it (rejected); next: speed / cost near obstacles |

The real rover's reach / goal_exec load 302de1b at the next `./rover nav`.

## 2026-10-03 — close quarters (narrow-gap tests with the owner)

Test: a 45 cm gap, spray bottle one side, a ~7 cm chair leg the other (an
S-gap); the rover touched the leg in all three runs. Root causes and fixes,
all committed on `rover-v1.1.7` (33fd348, 146361c, 15029cd, 11ee9b7 + the
review commit after it):

| what | fix | status |
|---|---|---|
| Nav2 Spin swung the outline blind in the gap | Spin out of both BT recoveries | in |
| reach's pass refused since 09-27 (sent 1 cm, goal_exec needs 2) | `PASS_MARGIN = 0.02` | in |
| Turn smear: objects mapped again 3-10 cm beside themselves | `phase2/nodes/turn_gate.py` + `depth_gate.py`: depth to nvblox only after 0.8 s with no turn; depth memory places frames at their stamp | measured (central-view smear 17 -> 5 cells at 6-10 cm) |
| Chair leg barely in the map | nvblox slice 5 -> 3 cm | measured (leg ~6 cells -> tip + edge) |
| Planner routed with 0.8 cm to the leg | global `footprint_padding` 0.04 | plan-checked; 45 cm gap = no path |
| Live depth reads mid-air points in front of white walls (emitter off for cuVSLAM) | exact moves use `phase3/nodes/fused_obstacles.py` (nvblox fused map); `ROVER_OBSTACLES=depth` reverts | live turns OK; tests `phase3/tools/test_obstacle_sources.py` |
| reach drove into the gap mouth / looped escape + nav2 | on no path or a jam: look, straight pass first (1.8 / 1.3 / 0.9 m) | floor run at 60 cm next |

**Open:**
- A 45 cm gap is below what can be promised: the fused map is 2.5 cm cells,
  the pass needs 38 + 2 x 2 cm, and it reads 41 cm. sim_gap_pass agrees
  (46 cm is the boundary). For 45: a sharper near view -- tilt the D555 down
  15-20 deg, or a low front webcam + a floor/not-floor model (owner has a
  Logitech webcam), or learned stereo (Isaac ROS ESS; not in the image).
- footprint_clear (global) clears 2 cm past the now 4 cm padding: the
  planner does not see anything within ~6 cm of the rover's outline.
- After moving obstacles, restart the map layer (`./rover map`): the static
  map has no decay, so the old positions stay until seen free.
- depth_gate costs ~17% of one core (Python relay of raw frames); a C++
  relay if CPU gets tight.


## 2026-10-02 (Pi 5 brain, floor-tested with the owner)

The robot's memory is now its PHOTOS (Pi 5 `tools/photo_recall.py`, `ask_photos`): the VLM reads every logged photo, picks one and a box, and pixel_to_goal places it with THAT photo's held depth + pose -- the background object survey is off. "go near X" asks the photos first; on arrival code takes one photo and checks X is really there; the drive tools carry an errand (`then`: "...and tell me what is on it") to the arrival report. Teleop: the page re-reads the mode every 2 s and logs every switch (81caf56) -- and the Pi 5 was still running the 08-22 copy, so 9d780b1 (MANUAL cancels reach + goal_exec) was **deployed only now**.

One page to come back to. Details and measurements live in the linked
documents; this is the list.

## Done 2026-09-26 (floor-tested with the owner)

| what | result | where |
|---|---|---|
| B1: the brain moves through the Jetson's exact controller | `move_robot` turns 6/6 within 2°; `F:30,L:90,F:30` legs end 0.8 / 0.5 cm; navigation goes through `/reach` | INTELLIGENCE_PLAN.md §5 |
| B2: coordinates from the moment of the photo, on the VLM's box | one bottle located from 3 poses: all within 0.7 cm (was 65 cm) | INTELLIGENCE_PLAN.md §5 |
| B3: object memory + the checker | found a bottle 1.45 m away, arrived 1.4 cm; backed off and turned away, "go near the bottle" turned back to it, "still where I saw it", arrived 1.0 cm | INTELLIGENCE_PLAN.md §5 |
| goal_exec turn fix | retries judged while coasting + arrival-time jitter; reproduced in the simulator, fixed | commit d313f5b |
| Idle depth subscriptions dropped | Jetson aligned depth 22 → 28 Hz, stalls > 0.5 s in 45 s: 3 → 0 (one sample) | commit 38a4c52 |
| Studio stale run queue | cleared; orphaned worker processes were writing it back | memory note |
| Lingering-obstacle checker | `phase3/tools/ghost_check.py` | OPERATIONS.md §5 |

## Fixed 2026-09-28 — code review of both repos (offline; NOT yet run on the rover)

Each needs one look on the floor after deploy; the "check" column says what.

| what | fix | check on the rover |
|---|---|---|
| **Object memory ids collided** (Pi 5) | the id was the sighting time in ms, and the survey stores every object of a photo with that photo's time: all ~5 shared one id, so forgetting one forgot the whole photo, and approach's "has it moved?" compared an entry with itself (also why 2 tests were flaky). Ids are now unique; old files are repaired on load | "go near X" after a survey; `/status` → `objects_remembered` |
| `reach_and_wait` could cancel its own goal (Pi 5) | set the new goal's key BEFORE cancelling the background drive, as `start_nav_to_pose` already did; the old order let the old worker send `/reach/cancel` after the new goal | "go to the chair" while a drive is running |
| Late pixel replies wiped a waiting one (Pi 5) | the reply cache dropped ALL entries at 32; now drops the oldest | — |
| `#9` again, in 8 more places | `set -e` + a probe of something that is down (`curl` 7, `grep`/`ls`/`tail` finding nothing) ended the script silently: `./rover goto/navto/drive/pass` with teleop down, `./rover camera` emitter retry, `./rover view`, `./rover grade`, `./rover status` (no nav2 restarts yet → no health report), and the Pi 5's `fleet.sh check` when the brain/LLM/teleop was down — the moment it exists for. New `refuse_if_manual` (uses `$PI5_HOST`, not a hard-coded IP) | `./rover status` on a fresh nav2; `fleet.sh check` with the brain stopped |
| `fleet.sh check` said "LLM down" when a token is set | sends `LANGROBO_API_TOKEN` from `.env`; a 401 is its own FAIL | — |
| Depth gap + TF lag dropped the photo (Jetson) | pixel_to_goal now also waits for the photo's camera pose when TF lags the stamp (both come from the same CPU load), then the still-camera rule of c4a5ee5 | `grep 'after a depth gap' /tmp/pixel_to_goal.log` |
| A gyro stall silenced `/fusion/status` too (Jetson, #2) | status keeps coming from the LiDAR callback with `gyro_alive: false`; **goal_exec refuses to move on it** ("gyro stalled: the pose is frozen") — it used to refuse only because status went silent; `health.py` says so | `./rover status` during the next stall: pose ✗ "the gyro has stopped" |

From the floor-test record (TODO / commit history; the robot's own logs were not reachable):

| what | fix | check on the rover |
|---|---|---|
| **The photo survey used a slot the documented server does not have** (Pi 5) | every doc said `--parallel 3` (slots 0-2); the survey pins slot 3, so on such a server each survey call failed, counted only as an error, and object memory stayed empty. **2026-09-29:** slot 3 is the vision-TOOL slot for every one-shot photo prompt (search views, locate, survey) -- `_vlm_locate` used to run on local_agent's slot 1 and overwrite the vision conversation's cached photos up to 8 times per search; on a smaller server the survey is off rather than evicting an agent; docs and `fleet.sh check` say `--parallel 4 --swa-full` | Pi 5 boot log: "vision-tool slot 3"; `/status` → `photo_survey.enabled` true, `errors` 0 |
| **MANUAL flipped mid-move did not stop the brain** (Pi 5; 2026-09-27 a "look around" turn went ~120° across a MANUAL→AUTO flip) | the switch was checked only when a tool started; a scan/search kept turning against teleop's zeros and a reach retried for minutes. agent_node now watches the switch (2 Hz) and on the edge into MANUAL cancels the drive and interrupts the tool, as a new utterance does | "look around", flip MANUAL after the first turn: it stops and says so |

Second review, 2026-09-29:

| what | fix | check on the rover |
|---|---|---|
| **Any Telegram member could drive the robot** (Pi 5, security) | `CAP_MOVE` was defined and never checked; family and guests could send it anywhere. Every motion tool now refuses a sender without it; "stop" stays open to all. Tool schemas unchanged (the KV cache is untouched) | from a family account: "go forward" is refused, "stop" works |
| **MANUAL did not cancel reach or goal_exec** (teleop) | it cancelled nav2 goals only; goal_exec kept commanding against teleop's zeros (the rover jerks) and reach retried for minutes. MANUAL now also publishes `/reach/cancel` and `/goal_exec/cancel`. **Redeploy:** copy `phase1/teleop/teleop_web.py` to `~/langrobo_teleop/` on the Pi 5, then `pkill -f teleop_web.py` (phase1/teleop/README.md) | `./rover goto 0.5 0` then flip MANUAL: stops at once **Deployed 2026-10-02** -- it had never reached the Pi 5; verified: teleop publishes /reach/cancel and /goal_exec/cancel. |
| Saved places lost on a power cut mid-save (Pi 5) | `locations.json` is written atomically | — |

Checked and fine: the ESP32's 500 ms /cmd_vel watchdog; goal_exec and gap-pass
simulator suites (both SUITE PASS).

New: `GET :8090/status` on the Pi 5 carries `photo_survey` (photos, objects,
errors, queue) and `objects_remembered`; `fleet.sh check` prints them.

## Open — most damaging first

| # | issue | evidence | next step |
|---|---|---|---|
| 1 | **The Jetson is CPU-bound, and the camera pays for it** | every core 76-88%; the D555 driver logs "callback took too long"; depth stopped for up to 9 s (in the frames' own stamps); `/odom` stamped 50 ms apart arrived 0.6-441 ms apart | a longer measurement, then the heavy Python nodes: fusion2 ~40%, lidar_odom ~38%, gyro ~29%, goal_exec / reach ~27% idle (likely TF listeners at 66 Hz), pixel_to_goal ~25%, image_bridge ~21% **2026-10-02:** idle py-spy showed the Python nodes spending 77-86% of their time in rclpy's stock executor; EventsExecutor cut pixel_to_goal 24.7->11.0%, goal_exec 26.4->8.1%, image_bridge 21.1->17.5%, gyro 28.7->15.7%, reach 24.3->8.4% (each floor-tested); Jetson idle 27.0->37.8%. **fusion2 + lidar_odom are changed but load only at the next ./rover up: grade them then** (turns within 2 deg, F:30 ending ~1 cm, /fusion/status healthy; ROVER_EXECUTOR=single reverts). Also: nvpmodel is 15W (CPU 1.5 of 1.73 GHz) -- a higher mode is the owner's call (power supply). |
| 2 | **The gyro rides the camera's link** (LOCALIZATION_GAPS G12) | the same link stalled for 9 s; fusion2 publishes the pose from the gyro callback, so a camera stall freezes the pose; since 2026-09-28 the status says so (`gyro_alive: false`) and goal_exec refuses | an IMU on the ESP32 or the Jetson; or fusion2 predicting on wheels + LiDAR while the gyro is out (an estimator change: grade it offline on a recording with a gyro gap first) |
| 3 | **The VLM is slow and loose** | Gemma 4 12B on the Mac: ~9 tok/s writing; y off by up to 45 px (boxes + pixel_to_goal's padded slab work around it); sensitive to wording ("spray can" missed where "air freshener spray can" was found) and misses flat things (a keyboard). Photo questions now ~5 s warm (photos cached on slot 3). A false sighting seen once (a toy car taken for something by a backpack) -- caught by the Pi 5 arrival check | the Mac: `--swa-full` + speculative decoding (needs ssh to the Mac); a better model later |
| 4 | ~~The checker's "bottle gone" path is only unit-tested~~ **superseded 2026-10-02** | the photo-based approach + the arrival check: floor-tested "it has moved about 0.6 m", "can't see the toy car in front of me" (a wrong sighting), and a full search for an absent object | -- |
| 5 | **Obstacles linger in the RViz costmap** (owner report) | not reproduced in 3 walk-ins; the costmap cleared in steps up to ~14 s after nvblox; twice 150-400 stale cells were flushed mid-test | run `ghost_check.py` while one is on screen (OPERATIONS.md §5); do NOT turn nvblox decay on (it forgets low obstacles) |
| 6 | Turns land slightly short; slides are large | all within 2° but ~1.3° short; a 180° turn slid 34 cm | tighten the tolerance once the coast is modelled; the slide is the pivot, which moves with the battery |
| 7 | ~~Object memory~~ **replaced 2026-10-02 by the photo log** | the Pi 5 no longer keeps "object at x,y" (owner's call); photos are asked directly, 24 kept = SNAPSHOTS here | pixel_to_goal's 24 held snapshots are now the memory's depth: keep SNAPSHOTS >= the Pi 5's photos.MAX_PHOTOS |
| 8 | Studio and the Telegram brain run the same graph | both can drive; only agent_node hears nav-done; a restart that kills only the parent leaves orphaned workers | one brain at a time for DRIVING; restart kills the whole process tree. (Studio takes no voice/Telegram input — only agent_node polls — so it cannot steal a request; its own drives report to nobody.) Pi 5 skills are now `/robot-start` and `/robot-stop` |
| 9 | ~~`./rover up` exited 7 right after starting Studio~~ **fixed 2026-09-27** | curl's exit 7 (Studio not listening yet) inside a `$(...)` under `set -e` ended the script; two sibling checks had the same trap | `|| true` on all three; the laptop RViz step runs again |
| 10 | Seen once, not reproduced | a captured frame PIL could not open; a Pi 5 ssh drop mid-run | watch |
| 11 | Localization leftovers | depth time offset unmeasured (G8); wheel ticks unstamped (G6); Pi 5 clock ahead of the Jetson (G9) | LOCALIZATION_GAPS.md |
| 12 | Power system | the turn slide depends on the battery; charge before motion tests | ROVER_BUILD_PLAN.md §4.5 |

## Ahead in the plan

| step | what | INTELLIGENCE_PLAN.md |
|---|---|---|
| B4 | fast eyes: a detector on the GPU (TensorRT), feeding object memory | §4 |
| B5 | smart search: overlapping exact turns, one VLM call lists everything | §4 |
| B6 | learning loop: episode log, weekly detector fine-tune on the owner's own objects | §1.4, §4 |

## Suggested order

0. Deploy both repos and walk the "check on the rover" column of the
   2026-09-28 table (≈10 minutes).
1. #4 — two minutes at the rover.
2. #1 + #2 together — CPU load and the camera-borne gyro are behind most of
   the flakiness, and B4 needs a healthy Jetson.
3. B4, then B5.

## Housekeeping

- The Pi 5 sudo password was typed into a chat session on 2026-09-26: change it.
- 2026-10-03 tidy of all repos and machines (SYSTEM_OVERVIEW.md §5, C1–C13): done. Left
  for the owner: delete `~/mitra_sim.handcopy-2026-10-03` on the Jetson (identical to
  the clone now at `~/mitra_sim`), and decide whether the older `rover_sim` is retired.
