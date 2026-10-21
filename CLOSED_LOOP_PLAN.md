# Closed-loop approach: "go near X" that keeps seeing X (plan, 2026-10-21)

Status (2026-10-21): stages 1-4 BUILT and tested (1-3 off the robot, 4 on the
twin; branch `detector-trt`). Stage 5 (the Pi 5 brain uses it) not started;
`./rover approach <label>` runs the closed loop on the real robot without the
brain. Stage 6 (floor test) next, with the owner. See
INTELLIGENCE_PLAN.md B3-B5 for the older steps this extends.

## The idea

Today "go near X" is one shot: one photo -> VLM pixel -> 3D point -> goal ->
drive blind -> one VLM arrival check (~3 VLM calls, 15-120 s standing still,
no correction if the first estimate is off or the object moved).

Closed loop: find X ONCE (detector for named things, VLM / a tap / a pointing
arm for unnamed ones), LOCK it (odom 3D point + an image crop of its look),
then every frame on the Jetson: predict where it is in the new image from the
pose, confirm it there (tracker / detector), refine the 3D point, update the
/reach goal when it moves > ~10 cm. VLM only for the first find and a rare
"still there?" check. nav2 stays (through /reach, no default BT): it plans the
path; the loop only supplies the goal.

## Flow (after)

1. Ask -> Pi 5 fast intent routing (no LLM) for "go near X" / "follow me" / "stop".
2. D555 colour + depth; YOLO on the GPU (TensorRT).
3. Find once: COCO class -> YOLO; unknown name -> open-vocab detector;
   unnamed ("that dot") -> VLM once, or a tap / pointing gesture.
4. Pixel -> 3D (pixel_to_goal, photo-time depth + pose). Flat things with no
   depth: intersect the camera ray with the floor plane.
5. Lock (new node): 3D point in odom + crop + box size.
6. /reach/goal at once (0.45 m standoff, as today).
7. Track every frame (~15-30 Hz): project the stored point with the current
   pose -> search a window -> confirm the look -> update the point (smoothed)
   -> new /reach goal if moved > ~10 cm.
8. Lost briefly: drive on the stored point; lost long / look mismatch: one VLM check.
9. Last ~1 m: goal_exec finishes on the refined stored point (low things leave
   the camera's view).
10. Arrival: no VLM call if tracking held.

Big objects (table): standoff on the nearest reachable side of the object's
footprint, not its centre. Follow me = the same loop with "person" (follow_node).

## Stages and what each needs

| stage | work | needs |
|---|---|---|
| 1 | yolov8n on TensorRT; detections_3d uses the engine; measure CPU before/after. **Done 2026-10-21** (OPEN_ISSUES.md): 16 % vs 36 % of a core at 5 Hz, 0.6 vs 1.4 GB, same detections; 15 Hz now costs less than 5 Hz did. yoloe-11s-seg: later (needs mask output in yolo_backend.py) | Jetson only (GPU load a few minutes: mind power) |
| 2 | Lock-and-track node + offline unit tests. **Done 2026-10-21**: see "Stage 2 as built" below | Jetson only |
| 3 | Tracker + pose prediction on recorded drives. **Done 2026-10-21**: see "Stage 3 results" below | Jetson only |
| 4 | Full test in the twin. **Done 2026-10-21**: see "Stage 4 results" below | laptop twin (Isaac Sim 6.2) |
| 5 | Pi 5: fast routing + the new tool | Pi 5 + twin |
| 6 | Floor test | everything, owner present, motion only with the owner's OK |

TODO: add depth images to the bag recording list so stage 3 can test the full lock.

## Stage 2 as built (2026-10-21)

| piece | what |
|---|---|
| `phase4/nodes/target_lock.py` | the lock, no ROS: predict through the pose -> template match near the prediction (rescaled for range; floor things also for foreshortening) -> depth / floor plane / bearing -> gate -> fuse. States: tracking, coasting, occluded, out_of_view, too_close, lost. Also `goal_update_due()` (when an approach should re-send /reach's goal) and `standoff_goal()` |
| `phase4/nodes/target_track_node.py` | ROS: `/target/lock` + `/target/cancel` -> `/target/state`. Asleep (~1 % of a core) it keeps only image_bridge's JPEG bytes; colour, aligned depth and TF only while locked (5-8 % of a core at 15 Hz, 147 MB). `./rover track`, started by `./rover up` |
| `pixel_to_goal.py` | its reply now also carries `object.z` and `camera` (the photo's camera pose), so a lock on a 40 s old photo needs no TF from then |
| `phase4/tools/test_target_lock.py` | 13 cases on a ray-cast world with exact truth, ALL PASS: still 0.3 cm; driving + weaving 0.3 cm; a 25 cm wrong first guess fixed by frame 3; a 30 cm move found again in 1.7 s; occluded 1 s without jumping; a look-alike 45 cm away never taken; out of view and back; a floor dot with no depth 2.2 cm; gone -> "lost" at 3 s; too close -> point held; 1 cm / 0.5 deg pose noise 1.6 cm; exposure swings + turn blur 30/30 tracked; goal rules. 1.2 ms per update |
| `phase4/tools/sim_target_track.py` | the real node over ROS, fed the rendered world: locked on a 3 s old photo, 60/60 frames tracked while driving, 25 cm guess -> 0.3 cm, cancel -> idle |

Design decisions:
- **Seeing and driving are separate.** /reach restarts its attempts on every new
  goal, so the tracker never sends goals; the approach (stage 4) re-sends only when
  `goal_update_due()` says so (>= 12 cm shift, >= 2 s apart, never in the last metre).
- **One owner of the photo's pose**: pixel_to_goal (it already snapshots it); the
  tracker gets it in the lock instead of running a second always-on TF listener.
- **A lock is never dropped by the tracker**: "lost" is the caller's cue for one VLM check.
- Not done (by design, later stages): finding the thing (detector / VLM), the
  approach policy wired to /reach, the Pi 5 tool.

Known limits (synthetic world; stage 3/4 will measure the real camera):
- ~~matching needs the whole look on screen~~ fixed in stage 3: at least half of it
  on screen is matched (direction only);
- a look-alike could take the lock after a LONG occlusion (the search widens while
  unseen) -- the VLM check after "lost" is the guard;
- floor positions far away are coarse (~3 cm per pixel at 1.5 m with the camera
  30 cm up); they sharpen as the rover nears.

## Stage 3 results: the tracker on real recorded drives (2026-10-21)

Two drives (2026-10-18 house survey, 2026-10-19 RViz goals; 16,927 colour frames
at ~4 Hz, the fused pose, the full TF, no depth). Truth without depth: static
objects YOLO saw from many places, triangulated; only those whose views agree
within 6 px count (5 of 16 -- the rest were sets of identical dining chairs that
merge into one "object"). Rerun: `phase4/tools/bag_frames.py` then
`phase4/tools/replay_target_lock.py` (docstrings).

| what | result |
|---|---|
| A. pose prediction (truth projected through the recorded pose vs where YOLO saw it) | median 2.4 px / 4.5 px (p90 6-12), the same driving and turning |
| A. camera vs pose time offset | best at +0 ms (survey) and flat over 0..+60 ms (real): no colour offset to correct (LOCALIZATION_GAPS G8 updated; depth still open) |
| B. tracker locked on the object, no depth (bearing only) | **tracking 100 % of the frames YOLO saw it, on all 5 objects**; 2.5-7.7 px; "off the object" 0-11 % = a few px outside YOLO's own jittery box |
| B. locked 25 cm off sideways, no depth | 4 of 5 still 100 %; error 25 cm -> 5-12 cm (the rest is range, which depth fixes live) |
| B2. sets of identical chairs / fridges (10, no truth) | **0 jumps between look-alikes** in 60 s each |

Changed by what the real frames showed:
- **A big thing cut by the frame edge was "out_of_view"** (a chair at the side,
  low camera): now its visible part (>= 50 %) is matched, and only the direction
  is corrected from it (a part's depth is not the whole thing's). Synthetic
  case `edge` added: 7/7 frames tracked while cut, 0.8 cm.

Learned, not yet handled:
- **A lock taken while something covers the thing** (a door edge in the box) stores
  the wrong look and never matches again (chair@910). The lock should come from a
  clear view: stage 4/5 should lock on the clearest of the search's photos, and a
  lock that never once tracks in its first ~2 s should be retaken.
- The 25 cm-off lock on one chair tracked only 14 %: without depth a wrong range also
  mis-scales the look. Live, pixel_to_goal's range comes from depth; recheck in stage 4.

## Places vs things (owner's question, 2026-10-21)

"Go to the kitchen" has nothing to look at: the goal is a saved map spot, driven
by /reach on the map pose (LiDAR + VO, ~1 cm), with obstacles seen live -- not
blind, but the goal itself is not confirmed by sight. The tracker is for THINGS.
"Go to the kitchen table" = place first (map), then look, lock, closed loop.
To add (later stage): an **arrival check for places** -- one photo ("is this the
kitchen?") or a lock on a known landmark there (the fridge) to correct the final pose.

## Stage 4 results: closed loop on the twin (2026-10-21)

`phase4/tools/closed_loop_approach.py` plays the brain (find -> pixel_to_goal ->
/target/lock -> /reach, goal re-sent on the refined point), judged on
/sim/truth/pose. World `clutter`, teal bottle 0.9 m ahead, 2 runs per cell:

| first estimate | one-shot (today's brain) | closed loop |
|---|---|---|
| good (VLM box +-40 px in y) | ~1 cm, ~1 deg | ~1 cm, ~1 deg |
| 25 cm off sideways | misses by **24 cm**, faces 29 deg off | **1.3-1.4 cm**, 1.5-1.7 deg |
| 25 cm too far | stops **21 cm** short | **1.1-1.2 cm** short |

pixel_to_goal's box grounding already absorbs the VLM's y error in a clean
scene; the closed loop is what fixes a WRONG first point (background depth,
clutter in the box, a moved thing).

Fixed because of the twin (each a real bug the offline tests could not show):
- the tracker paired colour with the latest depth; they share a stamp but arrive
  in either order, so half the frames ran without depth -> a range error was never
  corrected. Now paired by stamp; `/target/state` says `with_depth`.
- the look was scaled from the first guess's range -> stopped matching when that was
  wrong. Now from the current best point.
- `final_correction_due()`: one exact move after arrival if the refined point is
  >= 5 cm off the goal's (a backstop; not needed in these runs).
- the test driver's own races (TF / discovery at start) -- fixed in the tool.

Regression on the twin with this branch (no navigation code changed:
`git diff rover-v1.1.16-review..detector-trt` touches no phase1-3 / nav file):
`./sim scenario clutter` PASS; `./sim scenario apt_s1` all 8 goals reached, with
the KNOWN open items: the 3 cm slipper driven into (below every sensor,
OPEN_ISSUES) and two grazes in turns (sofa -0.4 cm, shoe rack -1.1 cm; the
6-16 cm turn slide, NAV_PLAN open item 2).
The Pi 5 brain suite (`./sim agents apt_s1`) could not run: the Mac mini's LLM
server was unreachable (`singireddys-mac-mini.local` does not resolve).

On the real robot (stage 6): `./rover approach bottle` (closed) and
`./rover approach bottle --mode oneshot`, a real bottle 1-2 m away; it prints
what the robot measured.
