# Open issues — where things stand, 2026-09-26

One page to come back to. Details and measurements live in the linked
documents; this is the list.

## Done today (floor-tested with the owner)

| what | result | where |
|---|---|---|
| B1: the brain moves through the Jetson's exact controller | `move_robot` turns 6/6 within 2°; `F:30,L:90,F:30` legs end 0.8 / 0.5 cm; navigation goes through `/reach` | INTELLIGENCE_PLAN.md §5 |
| B2: coordinates from the moment of the photo, on the VLM's box | one bottle located from 3 poses: all within 0.7 cm (was 65 cm) | INTELLIGENCE_PLAN.md §5 |
| B3: object memory + the checker | found a bottle 1.45 m away, arrived 1.4 cm; backed off and turned away, "go near the bottle" turned back to it, "still where I saw it", arrived 1.0 cm | INTELLIGENCE_PLAN.md §5 |
| goal_exec turn fix | retries judged while coasting + arrival-time jitter; reproduced in the simulator, fixed | commit d313f5b |
| Idle depth subscriptions dropped | Jetson aligned depth 22 → 28 Hz, stalls > 0.5 s in 45 s: 3 → 0 (one sample) | commit 38a4c52 |
| Studio stale run queue | cleared; orphaned worker processes were writing it back | memory note |
| Lingering-obstacle checker | `phase3/tools/ghost_check.py` | OPERATIONS.md §5 |

## Open — most damaging first

| # | issue | evidence | next step |
|---|---|---|---|
| 1 | **The Jetson is CPU-bound, and the camera pays for it** | every core 76-88%; the D555 driver logs "callback took too long"; depth stopped for up to 9 s (in the frames' own stamps); `/odom` stamped 50 ms apart arrived 0.6-441 ms apart | a longer measurement, then the heavy Python nodes: fusion2 ~40%, lidar_odom ~38%, gyro ~29%, goal_exec / reach ~27% idle (likely TF listeners at 66 Hz), pixel_to_goal ~25%, image_bridge ~21% |
| 2 | **The gyro rides the camera's link** (LOCALIZATION_GAPS G12) | the same link stalled for 9 s; fusion2 publishes the pose from the gyro callback, so a camera stall can freeze the pose and goal_exec pauses | an IMU on the ESP32 or the Jetson; until then fusion2 keeps publishing on LiDAR + wheels |
| 3 | **The VLM is slow and loose** | Gemma 4 12B: 6-40 s a call; y off by up to 45 px (boxes work around it); "bottle" missed where "the orange bottle" was found | B4: a detector on the idle GPU for everyday objects; VLM for descriptions and confirmation |
| 4 | **The checker's "bottle gone" path is only unit-tested** | the floor test covered "still there" only | remove the bottle, ask again (2 minutes) |
| 5 | **Obstacles linger in the RViz costmap** (owner report) | not reproduced in 3 walk-ins; the costmap cleared in steps up to ~14 s after nvblox; twice 150-400 stale cells were flushed mid-test | run `ghost_check.py` while one is on screen (OPERATIONS.md §5); do NOT turn nvblox decay on (it forgets low obstacles) |
| 6 | Turns land slightly short; slides are large | all within 2° but ~1.3° short; a 180° turn slid 34 cm | tighten the tolerance once the coast is modelled; the slide is the pivot, which moves with the battery |
| 7 | Object memory is narrow | written only by locate / approach, not `look()`; two matching objects → best match only; no re-check on arrival; entries expire after 1 h | B5: every look records everything it sees; re-check on arrival |
| 8 | Studio and the Telegram brain run the same graph | both can drive; only agent_node hears nav-done; a restart that kills only the parent leaves orphaned workers | one brain at a time; restart kills the whole process tree (the rover-start skill still says `pkill`) |
| 9 | `./rover up` exited 7 right after starting Studio | the laptop RViz step never ran; started by hand | not investigated |
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

1. #4 — two minutes at the rover.
2. #1 + #2 together — CPU load and the camera-borne gyro are behind most of
   the flakiness, and B4 needs a healthy Jetson.
3. B4, then B5.

## Housekeeping

- The Pi 5 sudo password was typed into a chat session on 2026-09-26: change it.
