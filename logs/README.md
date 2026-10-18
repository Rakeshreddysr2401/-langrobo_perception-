# `logs/` — what the running rover writes here

Mounted into the container as `/logs` (the twin's are `~/.rover_twin/logs`), so
everything here outlives a container restart. Run OUTPUT is git-ignored
(`.gitignore`); only the few files below are tracked.

| path | written by | what it is |
|---|---|---|
| `bags/<time>_<scenario>/` | `./rover record S` (phase1/harness/run.py) | one test run's bag; `./rover grade --summary` writes `bags/SUMMARY.md` |
| `calib/cam_lidar.jsonl` | `./rover camera --lidar` (phase1/nodes/cam_lidar_calib.py) | camera-to-LiDAR poses; `--solve` prints the params.yaml values |
| `goal_exec_pivot.json` | goal_exec, after every goal | the learned pivot, so a session starts where the last one ended |
| `nav_crashes/` | phase3/launch/nav2_supervise.sh | libbackward traces of nav2 nodes that died (one kept in git as evidence) |
| `drive_marks.json` | `./rover drive` (phase3/nodes/pivot_goto.py, `PIVOT_MARKS`) | its marks; tracked so a fresh clone starts from them |
| `compare-*.csv`, `*.txt` | old Phase 1 runs | ignored; the numbers they support are in docs/archive/PHASE1.md |

The diagnostic scripts that used to live here (`goto.py`, `around.py`,
`keepalive.py`, `calibrate_rotation.py`, ...) were removed on 2026-09-26
(commit `a09afc1`, "keep only what the rover needs"): their jobs are `./rover`
commands now (`goto`, `reach`, `lidar --calibrate`, `values`, `health`).
Recover one with `git show a09afc1^:logs/<name>.py`.
