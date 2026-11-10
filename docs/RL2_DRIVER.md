# RL2 — the route-following driver on the robot (kino_0.1.0)

`reach` hands tight stretches of a goal to a learned driver (`rl_round`). Since kino_0.1.0 that
driver can be **rl2**: a small CNN that follows the walking-distance route through doors and
between furniture. Trained and tested in kino_sim (`~/kino_sim/rl2/README.md`,
`docs/KINO_0.1.0_PLAN.md` there). The v002 driver still works unchanged.

## Switch
```bash
./rover rl on /models/rl/rl2_r008/policy.npz   # rl2 (r008/900); the stack must be up
./rover rl on /models/rl/current/policy.npz    # back to v002 (`rl on` alone keeps the last model)
./rover rl off                                  # no learned driver at all
./rover rl status
```
`rl_round` detects the kind from the file (an rl2 policy has conv layers). Episodes:
`logs/rl_episodes.jsonl` (result, min_clear_cm, shield_changed); failed rounds dump to
`logs/rl_dumps/` when that directory exists.

## Files (phase3/nodes)
| file | role |
|---|---|
| `rl2_drive.py` | the observation, exactly as training builds it: state 31 numbers (target, 8 route points, motion, pack, time) + patch 3 x 64 x 64 at 4 cm: ch0 the latest LiDAR scan, ch1 camera + costmap-memory points, ch2 floor not known |
| `rl2_policy.py` | the CNN in plain numpy (copy of kino_sim `rl2/policy_np.py`) |
| `known_floor.py` | 4 cm odom grid of what the camera has looked at + where the body stood |
| `rl_drive.py` | the shield (all drivers): every command rolled 0.8 s ahead in 0.05 s steps; refused -> half speed -> turn in place -> stop |
| `tight_route.py` | `_guide`: the route; rl2 uses clear 0.22 m (only where the rover fits) and `extend=True` (defined near things) |
| `reach_node.py` | `rl_round` builds all of the above each 0.1 s; `KnownFloor.mark` from odom at <= 10 Hz |

## Safety rules (each from a measured failure; kino_sim tests keep robot and training equal)
- **Shield in 0.05 s steps** (all drivers): at 0.1 s a hard turn moved the rear corner ~4 cm
  between checks, past the 2 cm margin.
- **Creep rule** (all drivers): a point already within 2 cm blocks only motion toward it —
  before, every move (backing away too) was refused and the rover was pinned.
- **Reverse only over known floor** (rl2): nothing on the rover looks back; the outline + 2 cm
  must stay over floor seen by the camera or driven over.
- The route passes only ways >= 42 cm (rover 38 cm + 2 x 2 cm).

## Numbers
- Unseen generated flats, 4 x 1024 episodes, routes to 12 m: 0 contacts, 90.7 % reached.
- Jetson, one full step (observe + policy + shield): 6.9 ms median, 7.1 ms p95 (budget 100 ms).
- Twin (truth-judged), v002 baseline on house_map + 2 flats: 16/19, 0 touches; rl2: see the
  kino_sim plan's status table.

## Before a real run
The twin streams the camera to the Jetson over WiFi: on 2.4 GHz (7 Mbit/s) the camera gate
fails; on 5 GHz (75 Mbit/s) it is fine. Goals come from RViz (a human) or the brain's VLM.
