# Making the rover intelligent — analysis and plan, 2026-09-26

The owner's questions, after the full stack came up and Studio answered:

1. Is there any machine learning / reinforcement learning that improves the
   movement day by day?
2. How do we get intelligent movement? Sometimes it does not move smartly.
3. "Go near the bottle": it rotates for no clear reason and skips the bottle
   even when the bottle is there. The guess: it takes the photo at (x1, y1),
   gets the object at (x2, y2), but by then it is at (x3, y3) with a different
   θ. And if it saved the coordinates of what it saw, it could turn to that
   direction and go there, or check it, when asked.

Speed matters (no long lag), and so does accuracy. Everything below was checked
against the code in both repos (Jetson `~/rover` at `5fbe69e`, Pi 5 `~/ros2_ws`
`dev-1.3.3-minimal` at `dea03a0`) and the running stack, not argued from memory.

---

## 0. The short answer

The **body** is now good. fusion2 holds x, y, θ to 0.8 cm / 0.45° worst over 19
runs, and goal_exec reaches a pose to ~1.4 cm (README). The **brain does not use
any of it.** The rover's intelligence problem is mostly an integration problem,
not a missing-AI problem:

| what the brain does today | what the Jetson already has, unused by the brain |
|---|---|
| turns by **time**, open-loop, at 5 rad/s: graded 31–68 cm slide per turn, one stall at 67° of 90° reported as done (Pi 5 `dea03a0`) | `/goal_exec/goal`: closed on the pose, learns its own pivot, ~1.4 cm |
| sends plain nav2 `NavigateToPose`: gives up on the first bad spell (15 s in a 51 cm corridor, 2026-09-26) | `/reach/goal`: nav2, then an exact finish, and on failure look / pass / wait, retrying |
| grounds the VLM's pixel with the **newest** depth and pose, not the ones the photo was taken at | a pose history in TF, and the camera's own timestamps |
| forgets every object the moment the nav goal is sent | a pose that no longer drifts, so a remembered (x, y) stays valid |

Your guess in question 3 is right, and it is not the only cause. §3 has all of
them.

---

## 1. Machine learning and RL — what exists, and what the market actually does

### 1.1 What this rover learns today

One thing, and it is real online learning. **goal_exec learns the rover's turn
pivot** from every turn it makes (`goal_exec.py` "THE SLIDE, LEARNED"): it
solves `P = (I − R(a))⁻¹ d` per turn direction, smooths it, and saves it to
`/logs/goal_exec_pivot.json` so the next session starts from what the last one
learned. That matters because the slide changes with the battery (33 cm per 90°
on a weak pack, 6 cm charged).

Nothing else learns. The brain (Gemma 4 12B, Q4, on the Mac mini, ~9.5 tok/s)
is a fixed model with prompts.

### 1.2 How robots on the market do it

| robot | navigation | where learning is used |
|---|---|---|
| Robot vacuums (iRobot j-series, Roborock) | LiDAR or camera SLAM, classical planner | **on-device object detection** (cords, socks, pet mess), retrained offline from fleet images. Learning is in *perception*, not the motion |
| Amazon Astro | SLAM + a semantic home map ("go to the kitchen") | detection and recognition models; motion is classical |
| Warehouse AMRs (Locus, Fetch/Zebra, MiR) | LiDAR SLAM + planner, much like nav2 | fleet analytics and traffic tuning, offline |
| Boston Dynamics Spot, ANYmal, Unitree | classical mapping and route replay | **RL for leg locomotion**, trained in simulation (Isaac Gym/Lab) with heavy randomisation, then frozen onto the robot. Not learned day by day on the robot |
| Research: RT-2, OpenVLA, π0, Gemini Robotics | end-to-end vision-language-action models | need large GPUs and thousands of hours of demos. Not usable on an Orin Nano |
| Research: SayCan, VLMaps, ConceptGraphs, **OK-Robot** | an LLM/VLM decides *what*; a **3D semantic memory** says *where*; a classical stack drives | off-the-shelf models, no training. OK-Robot (NYU, 2024) scanned a home, remembered objects with 3D positions, then went to any object asked for by text, with roughly 58% success in real homes |

**The pattern everyone converged on:** a slow, smart layer (LLM/VLM, seconds)
decides *what* to do. A fast layer (detector, 10–30 Hz) says *where things are*.
A memory holds object positions between them. A closed-loop controller (20–50 Hz)
*moves*. The LLM is never inside the motion loop. Figure's "Helix" and Gemini
Robotics call this System 2 / System 1.

### 1.3 Should this rover use RL?

**Not for driving, not now.** A wheeled base with a closed-loop controller that
already reaches 1.4 cm has little left for RL to win, and on-robot RL needs
thousands of trials, a reward, automatic resets and a safe way to fail. Nobody
in the table above does it on a home robot. RL becomes worth it for the **arm**
(grasping), trained in simulation on a desktop RTX GPU, never on the Orin.

### 1.4 "Improves day by day": the learning that does pay here

| loop | what it learns | from what | status |
|---|---|---|---|
| **Pivot / slide** | where the rover really turns about, per direction | every turn, live | ✅ goal_exec |
| **Motion response** | pivot and arc response vs battery state, stall thresholds | `./rover record` runs graded against LiDAR truth | ⬜ needs a battery-voltage reading (see ROVER_BUILD_PLAN.md §4.5) |
| **Detector on your own objects** | *your* bottle, *your* surf excel packet, as a fast detector class | frames the VLM has already labelled during normal use (teacher → student) | ⬜ §4 B6 |
| **Episode log** | success rate per command, why failures happen, which thresholds to move | every "go to X": command, detections, VLM pixel, grounded pose, outcome | ⬜ §4 B6 |

The detector loop is how the rover gets visibly smarter with use. The VLM is
slow but understands anything. Every time it finds something, that frame and
box become a training example, and a weekly fine-tune (ultralytics on the Mac
mini's GPU, exported to TensorRT for the Jetson) turns "find my bottle" from a
10–40 s VLM call into a ~50 ms detection. This is the vacuum makers' data
engine, at one-robot scale.

---

## 2. Why the movement does not look smart

Found in the code:

1. **The brain's turns are timed, open-loop pivots at 5 rad/s.** `movement.py`
   (`_drive_for_duration`, `_duration("L", …)`) is used by `move_robot`, by
   `scan_surroundings` and by the search in `approach_described_object`.
   Graded on 2026-09-24: slides of 31–68 cm per turn, pivot points spread
   36 cm, one turn stalled at 67° of 90° and would have been reported as done.
   After two search turns the rover is somewhere else, facing somewhere else,
   than the brain thinks. **This is most of the "rotating for no reason".**
2. **The brain uses raw nav2, which gives up early.** `ros2_bridge._nav_worker`
   sends `NavigateToPose` directly. nav2 alone failed a 51 cm corridor in 15 s
   (reach_node.py docstring), and it has no exact finish. `/reach` (retry,
   look, pass, wait, then an exact finish) is **already running** in the stack
   (`/reach` is in the node list) and nothing on the Pi 5 calls it.
3. **The rover stands still while it thinks.** Each search step is a VLM round
   trip of 10–40 s, up to 5 steps: more than 3 minutes of turn, freeze, turn.
   From outside that looks like confusion.
4. **The LLM sits on the critical path of every action.** At ~9.5 tok/s the
   routing turn alone takes seconds before anything moves. Deterministic
   routing for fixed phrases ("go near X", "stop") is already designed in the
   Pi 5's `INTENT_ROUTING_PLAN.md`.

---

## 3. "Go near the bottle": every cause, from the code

The path today (`approach.py` → `pixel_to_goal.py`):

```
_fresh_frame()  newest JPEG, stamped on ARRIVAL at the Pi 5 (camera stamp dropped)
      │  10–40 s  Gemma 4 12B: "where is the bottle?" → pixel (u, v)
      ▼
ground_pixel(u, v)  → /vision/pixel_query  (no timestamp in it)
      ▼
pixel_to_goal: NEWEST aligned depth + TF at Time() (latest)
      ▼
standoff goal → plain nav2 → object position discarded
```

| # | cause | where | effect |
|---|---|---|---|
| **C1** | **The pixel is projected with the wrong depth frame and pose** (your guess). The photo is from time t1 / pose P1. pixel_to_goal uses the depth frame and TF from t3, when the query arrives. The camera's own stamp is kept by `image_bridge` but thrown away by `ros2_bridge._on_compressed_image`, which stamps `time.monotonic()` on arrival | `ros2_bridge.py:211-216`, `pixel_to_goal.py` `lookup_transform(..., Time())` | Correct only if the rover did not move between t1 and t3. After a 5 rad/s timed turn it is often still coasting or sliding when `_fresh_frame` takes the first frame younger than 0.8 s, so the pixel lands on the wrong depth (a wall behind, the floor) or the right depth at the wrong bearing |
| **C2** | **The search turns are open-loop and 90° wide, with an 87° camera.** No overlap between views, plus under-rotation, stalls and 30–70 cm slides | `approach.py` `_SEARCH_STEP_DEG = 90`, timed turns | The bottle can sit in the gap between two views, or the rover slid past it: **"skipping the bottle even though it is there"** |
| **C3** | **Nothing is remembered.** Found objects are turned into a nav goal and dropped. A world model existed, fed by `/vision/detections_3d`, and was removed because nothing published that topic | `approach.py` docstring; Pi 5 `INTEGRATION_GAPS.md` §1 | Every request is a full search from scratch, even for an object seen a minute ago |
| **C4** | **One slow VLM call per view, asked for one thing.** The VLM sees a whole room and answers yes/no for one description | `_VLM_LOCATE_PROMPT` | 10–40 s per view, and everything else in that view is thrown away |
| **C5** | **Plain nav2 for the final approach** | `_nav_worker` | Stops up to 10 cm short or gives up; no exact standoff facing the object |

### 3.1 Your idea is right, and it now works

"If it saves the coordinates when it sees something, it can turn to that
direction and go there, or check." Yes, and this is exactly what OK-Robot /
ConceptGraphs do. It was **not** possible on this rover until very recently,
because `odom` drifted 5–19 cm after a few turns (LOCALIZATION_GAPS.md G1). A
remembered (x, y) would have been wrong by the time you asked. With fusion2 on
LiDAR odometry (0.8 cm worst over 19 runs, live since 2026-09-26), **a point
seen at (x2, y2) from pose P1 is still at (x2, y2) when the rover is at P3**.
The turn to face it is just geometry:

```
bearing = atan2(y2 − y3, x2 − x3) − θ3        # from the CURRENT pose
```

This is session memory in `odom`: it resets at power-off, which matches the
no-saved-map choice. Remembering "the bottle is usually on the table" across
days would need a saved map and relocalisation. That is a separate decision,
not needed for any of this.

---

## 4. The plan — in order, each step graded on the rover

Same working agreement as movement M1–M5: one step at a time, each graded
before the next, with the owner doing the physical parts (taped marks, placing
objects).

| step | what | fixes | size | graded by |
|---|---|---|---|---|
| **B1 — the brain moves with the Jetson's primitives** ✅ built, floor test pending (§5) | Pi 5 `ros2_bridge`: `start_reach(x, y, yaw)` publishes `/reach/goal` and reads `/reach/status` for the done callback (it replaces `_nav_worker`'s raw nav2 call). Every brain turn (`move_robot` L/R, search, scan) becomes a goal_exec heading goal, closed on fusion2's yaw, not a timed twist. goal_exec needs a **turn-only mode** (hold x, y; turn to θ) so a "face it" turn never drives off; add and test it in `sim_goal_exec.py` first | §2.1, §2.2, C2, C5 | small–medium, both repos | `move_robot L:90` ×6: heading error < 2°, slide pre-compensated. "go near the chair" through a 55 cm gap: reached, not abandoned |
| **B2 — ground the pixel at the moment of the photo** ✅ built, floor test pending (§5) | Keep the camera stamp end to end: `ros2_bridge` stores (jpeg, header.stamp); `ground_pixel` sends the stamp (in `PointStamped.header.stamp`, which is free today); `pixel_to_goal` keeps a 3 s ring of depth frames and uses the one nearest the stamp, with TF looked up **at that stamp** (the TF buffer already holds 10 s). Refuse if nothing is within 50 ms. Later: pair colour and depth at capture as a snapshot id | C1 | small, both repos | bottle on a taped mark; `locate_object` from 3 poses, including right after a turn: spread of the reported (x, y) < 5 cm |
| **B3 — object memory** | A session memory on the Pi 5: `{label, description, x, y, z, seen_at, seen_from_pose, confidence, source}` in `odom`. Every look / locate / approach writes to it. New tools: `recall_object("bottle")` (where and how long ago) and `face_object` (exact turn to the stored bearing from the **current** pose, then one look to confirm). `approach_described_object` checks memory **first**: known → face → confirm → reach; unknown → search. Objects move, so an entry is a hint to confirm, never a fact to drive at blind; confidence decays with age. **The checker** (owner, 2026-09-26: "sometimes the bottle may be removed by someone"): after facing the stored bearing, one fresh look. Still there → re-ground it (updated x, y) and reach. Moved a little → the new position replaces the old. Gone → forget the entry, say so, and fall back to the search. Re-checked once more on arrival | C3, your question 3 | medium, Pi 5 | see the bottle at pose A; drive elsewhere by hand; "face the bottle": bearing error < 3°. The LOCALIZATION_GAPS requirement ("what the camera sees at (x1, y1) must still be at (x1, y1)"), measured on an object |
| **B4 — fast eyes** | `phase4/nodes/detections_3d.py` (YOLOv8n) already publishes the documented `/vision/detections_3d` contract. Export it to TensorRT so it runs on the GPU (tegrastats 2026-09-26: GPU 0–9% busy, CPU 73–87% on all six cores, so the CPU is the scarce resource), feed B3's memory continuously, and restore the Pi 5 subscription. COCO covers bottle, cup, chair, couch, person, laptop, backpack, etc. Later, an open-vocabulary detector (YOLO-World / YOLOE) for descriptions COCO lacks | C4, §2.3 | medium | CPU and nav rates before vs after (the depth stall under VLM load is known); detections of a bottle at 1–3 m land within 5 cm of its taped mark |
| **B5 — smart search** | Exact turns of ~60° (overlap with the 87° view), one VLM call per view that **lists everything it sees** (all written to memory, not just the target), stop as soon as the target shows, and turn toward unseen directions first | C2, C4, §2.3 | small, after B1+B3 | time-to-find for a bottle placed behind the rover: today up to 5 × (turn + 40 s); target < 60 s, with no view gaps |
| **B6 — the learning loop** | Log every episode (command, detections, VLM pixel and answer, grounded pose, outcome, time). Weekly: success-rate report; VLM-labelled frames → fine-tune the detector on the owner's own objects (Mac mini), TensorRT → Jetson; refit the motion response from harness runs | §1.4 | ongoing | success rate and time-to-reach per week, from the log |

### 4.1 Speed: the latency budget

| request | today | after B1–B4 |
|---|---|---|
| "go near the chair", chair in view (COCO) | routing turn + VLM 10–40 s + nav2 | routing (deterministic) + detection < 0.5 s + reach |
| "go near the red bottle", seen earlier | full search again: up to ~3–4 min | memory lookup + exact turn + one confirm (detector, or VLM if needed) + reach |
| "go near the surf excel packet", never seen | up to 5 × (timed turn + 40 s) | 60° exact turns, one VLM call per view, memory filled on the way; after B6, a detector class |

The VLM stays for what only it can do: understanding descriptions, and
confirming. It leaves the motion path.

### 4.2 Accuracy: the error budget for "park 45 cm in front of the bottle"

| source | size | status |
|---|---|---|
| pose (fusion2) | ≤ 0.8 cm / 0.45° | ✅ measured |
| camera extrinsic (yaw +0.96° vs LiDAR) | ~1–2 cm at 2 m | ✅ calibrated 2026-09-25/26 |
| **pixel ↔ depth ↔ pose time mismatch** | **tens of cm after a turn** | ⬜ **B2** |
| D555 stereo depth at 2–3 m | a few cm, grows with range² | sensor limit: re-ground when closer (reach's exact finish already re-plans) |
| VLM pixel choice | a few degrees; can pick an edge pixel | median window in pixel_to_goal; detector box centres are steadier (B4) |
| depth timestamp offset (G8) | 9 cm at 3 m if 30 ms while turning at 1 rad/s | ⬜ LOCALIZATION_GAPS step 4; irrelevant while grounding at rest |
| final approach (goal_exec) | ~1.4 cm | ✅ once B1 routes the brain through it |

B2 is the largest remaining error, and the cheapest fix.

---

## 5. B1 + B2 as built, 2026-09-26

Both repos: rover `phase3/nodes/goal_exec*.py`, `reach_node.py`,
`phase4/nodes/pixel_to_goal.py`; Pi 5 `ros2_bridge.py`, `tools/movement.py`,
`tools/approach.py`, `tools/locate.py`, `bridges/stub.py`.

| piece | what changed |
|---|---|
| goal_exec **TURN-ONLY MODE** | `/goal_exec/turn` (PoseStamped, heading only): turn to it and stop, within 2°, with the slide reported and not driven back. The same turn primitive as before: swept-outline check about the learned pivot, early stop for the coast, stall stop, the pivot learned from every turn |
| goal ids | `/goal_exec/status` and `/reach/status` carry `goal_stamp`, the goal message's header.stamp, so a caller matches the result to its own goal and not to the one it preempted |
| brain moves | `move_robot` F/B/L/R → `bridge.drive_by` / `turn_by` on goal_exec, in pieces (≤170° turns re-aimed on the measured turn, ≤1 m legs); the reply says what the pose measured ("L:90 (turned +89 deg, slid 5 cm)"); a refusal stops the sequence and names the reason. The approach search (4 exact 90° views, was 5 timed) and `scan_surroundings` (6 × 60°) use the same turn. Timed twists remain **only** as the fallback when goal_exec is not running, and the reply then says so |
| brain navigation | `start_nav_to_pose` → `/reach/goal` (nav2 route + exact finish + look / pass / wait retries); plain nav2 only when reach_node is absent, or with `LANGROBO_NAV_BACKEND=nav2` |
| photo-time grounding | the Pi 5 keeps each frame's camera stamp and, on taking a frame for the VLM, publishes `/vision/pixel_snapshot`. pixel_to_goal freezes the depth frame nearest that stamp (from a 2 s ring, ≤100 ms away) and the camera's odom pose **at** the stamp, for 8 photos. The query carries the stamp; the object is placed from the photo's depth and pose, and `relative` / `goal` are measured from where the robot is now. If the photo is gone, the Pi 5 re-grounds on the newest view only if the robot has not moved since (≤2 cm, ≤1°) |

**Verified, moving nothing (2026-09-26):**
- `sim_goal_exec.py`: suite PASS, with the new turn-only cases in all three
  chassis conditions: 24/24 within 2.5°, worst 1.35°, no forward command.
  The one weak-pack refusal is unchanged from before (a wall near that goal).
- pixel_to_goal live: snapshot frozen 33 ms from the photo; a stamp 30 s old
  is refused `snapshot_expired`; with the rover still, photo-time and newest
  grounding agree to 2 mm.
- The real Pi 5 bridge against the live Jetson: frame stamp → hold → query
  `"at_capture": true`; `turn_by(0.5°)` through `/goal_exec/turn`, matched by
  its stamp: `reached, slid 0.0 cm`.
- Pi 5 tests: 228 pass (213 before, plus 15 for exact moves and photo-time
  grounding).

**Floor tests with the owner, 2026-09-26** (the brain's own code path,
`move_robot` / `_capture` / `_vlm_locate` / `_ground` on the real bridge):

| test | first run | fix | after |
|---|---|---|---|
| `L:90` ×3, `R:90` ×3 | 5/6 within 2°; one R:90 **failed "−5.6° after 8 tries"** | goal_exec judged the retries while the rover still coasted, and measured the turn rate from arrival times (poses stamped 50 ms apart arrived 0.6–441 ms apart). Reproduced in the simulator (1/10), fixed: settle before judging, rate over ≥ 40 ms, the pose stamp as the clock | **6/6 within 2°** (0.5–1.8°, all slightly short); slides 6.5–21.5 cm (timed: 31–68) |
| `F:30,L:90,F:30` | — | — | legs end 0.8 and 0.5 cm from their goals, turn +88°, 10.7 s, each step reported with its measured values |
| bottle ~1.2 m ahead, located from 3 poses | positions **65 cm apart**; one photo hit a 1.1 s depth stall | Gemma's y is off by up to ~45 px (x ~5 px): its "centre" was the top edge of the cap and the depth there was the door behind. Now the VLM gives a **box**, and pixel_to_goal takes the nearest solid slab above the floor inside it | start / after L:15 / after F:30: **all within 0.7 cm** of each other; VLM 6–7 s per call |

Still to grade: "go near the chair" through reach, including one tight gap.
Seen once, not reproduced: a captured frame that PIL could not open (the
tool reports it as a vision-model error).

## 6. Recommendation

Start with **B1 + B2** together: both are small, both are pure integration of
things that already exist, and together they remove the rotating, the
skipping and the wrong-coordinates problem at their roots. Then **B3**, which
is your question 3 as a feature. B4–B6 make it fast and make it improve.

Open question for the owner, not blocking anything: should object memory ever
survive a power-off? That brings back relocalisation, which was deliberately
set aside (fresh start every time).
