# Closed-loop approach: "go near X" that keeps seeing X (plan, 2026-10-21)

Status (2026-10-21): stage 1 BUILT and tested off the robot (branch
`detector-trt`; live check at the next bring-up); stages 2-6 not started. See
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
| 2 | Lock-and-track node + offline unit tests | Jetson only |
| 3 | Tracker + pose prediction on recorded drives (logs/bags/2026-10-19_*: colour, odom, TF; NO depth images) | Jetson only |
| 4 | Full test in the twin: dot, bottle, table, target moved mid-drive | laptop twin (Isaac Sim 6.2) |
| 5 | Pi 5: fast routing + the new tool | Pi 5 + twin |
| 6 | Floor test | everything, owner present, motion only with the owner's OK |

TODO: add depth images to the bag recording list so stage 3 can test the full lock.
