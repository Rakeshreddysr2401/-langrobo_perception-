# Todays_Todo.md — the approach path, read end to end (2026-09-10)

These came out of reading the **"go near the chair"** path in full across both
repos on 2026-09-10 — `langrobo_core` and `langrobo_ros` on the Pi 5, `phase4/`
on the Jetson — rather than from a failure. Nothing here is started.

**Ordered as a SERIES: the order to do them in, not the order they were found.**
Measurement first, then the blocker, then speed (which makes the later
verification work affordable), then the silent-wrong-answer holes, then
architecture, then cleanup. That ordering follows READINESS.md's own rule —
*nothing after a measurement step is trustworthy without it.*

Each item is to be **judged on worth before implementing**. Several are
deliberately arguable; those say so.

Status: ⬜ not started · 🔄 in progress · ✅ done · ❌ judged not worth it

---

## WHERE WE LEFT OFF — read this first (2026-09-10, powered down)

Everything was powered off for a break. Nothing here is implemented. What
changed during the session:

**1. The rotation bug is FIXED and verified end to end (2026-09-10).**
`LANGROBO_STEADY_ANGULAR_VEL=1.20` in `~/.langrobo/brain.env` on the Pi 5.
Measured through the real brain: **360→357°, 180→202°, 90→114°/84°**, against
~86/43/21° before. Residual is ±20–25% variance from mechanical spin-down coast,
not a systematic error. Full detail and the closed-loop option in TODO 36.

**Note for next time:** the "still broken" report came from a brain process that
had started *before* `brain.env` existed. After any env or `langrobo_core`
change, check `/proc/<pid>/environ` of the running `agent_node` before trusting
a symptom report — a restart needs a password and is always the user's step.

**2. TODO 33 got much worse, and now outranks everything here.** The micro-ROS
wheel link dies **under driving load** — reproducible on demand, commands and
telemetry stop together, recovers on its own, and it is *not* a current cliff.
Three of four rotation measurements were unusable because of it, and one looked
entirely plausible while being pure dropout.

> **No motion measurement on this rover can be trusted unless `/wheel_state`
> continuity is checked over the same window.** That applies to every item below
> that involves the rover moving.

**3. Item #1 below is largely ANSWERED — this rover CAN pivot.** Measured
2026-09-10: wheels counter-rotate cleanly at L −0.30 / R +0.32 m/s, body ~69 °/s,
held steady for 3 s whenever the link was up. So TODO 14's "cannot turn in
place" looks stale, which means **#2 is probably unnecessary and the real move
is re-enabling nav2's rotate-to-heading** rather than making goals yaw-agnostic.
Confirm on clean data before changing nav2.

### Update — powered down again 2026-09-10, after the rotation + stale-view work

**Rotation is FIXED and confirmed by the user.** Two separate causes, found in
this order:

1. `_STEADY_STATE_ANGULAR_VEL` defaulted to the commanded 5.0 rad/s → every turn
   4.2× short. Fixed via `LANGROBO_STEADY_ANGULAR_VEL=1.20` in
   `~/.langrobo/brain.env`. Verified through `agent_node`: 360→357°.
2. **It still looked broken, because Studio never read that file.** `brain.env`
   is loaded by *systemd* for `langrobo-brain.service` only; Studio is started by
   `start_studio.sh`. Same graph, uncalibrated constants. Fixed in
   `start_studio.sh` (Pi 5 `1dbac6e`) — it now sources the file, says so on
   startup, and warns loudly when it is missing.

**Item #14 (stale camera view) is shipped but NOT verified, and is reopened** —
Pi 5 `86e0b08`, 184 tests pass, but the fix has never executed on a live brain
and `prompts.py` still contradicts it. See its section below and
**[PERCEPTION_STATE.md](PERCEPTION_STATE.md)**.

**The power cycle applies everything.** A cold boot restarts `agent_node`, which
re-reads `brain.env` *and* re-imports `langrobo_core` from `src/`. So both fixes
go live for Telegram and voice with no password step.

**Studio does NOT start at boot** — run `~/ros2_ws/scripts/start_studio.sh` and
check the log says `loaded calibration from ~/.langrobo/brain.env`. If that line
is missing, Studio is uncalibrated and any motion test through it is invalid.

### Still to verify on return

- **#14 in practice.** The stale-view note is an *instruction* to a 12B model,
  not an enforcement. Test: `go forward` → `rotate 180` → `what can you see`.
  It should call `look()` again. If it answers from the old photo anyway, that
  is the signal to build something stronger — measure before building.

### On return, in this order

1. Bring the stack up (`rover-start` — container, six layers, Studio, RViz).
2. **Verify the rotation fix**: ask for a 360° turn and measure what actually
   happens. Expect roughly a full turn. `logs/measure_yaw_rate.py` is the tool;
   read its per-bucket ramp line, not just the mean.
3. **Then TODO 33**, before trusting anything else that moves: capture
   `ssh 192.168.1.16 'journalctl -u langrobo-microros'` **during** a pivot. The
   trigger is reproducible now, so the log this bug has always lacked is finally
   obtainable. Do not power-cycle the ESP32 first — that destroys the evidence.
4. Then pick from the series below, one at a time, on worth.

---

## The series

| # | item | why it is here | effort | status |
|---|---|---|---|---|
| 1 | Test whether the chassis can pivot | gates #2, and TODO 34 is the only 🔴 in this path | 5 min | 🔄 mostly answered — it CAN, see above |
| 2 | Make the approach goal yaw-agnostic | only if #1 says it cannot pivot | small | ⬜ |
| 3 | Costmap-check the standoff goal | the other suspected half of TODO 34 | medium | ⬜ |
| 4 | Downscale the VLM frame | 10–40 s per look dominates every approach | one line | ⬜ |
| 5 | Retry once on unparseable VLM output | a parse failure currently rotates away from a visible object | trivial | ⬜ |
| 6 | Guard the stale-pixel race | closes a confidently-wrong-goal hole | small | ⬜ |
| 7 | Height sanity check on the object | stops "arriving" under a clock on the wall | small | ⬜ |
| 8 | Verify arrival by looking again | success currently means *a coordinate was reached* | small | ⬜ |
| 9 | Fix the Studio conflict properly | a warning that needs a human reading logs is not a fix | medium | ⬜ |
| 10 | Delete the Pi 5's dead standoff geometry | the tested copy is the one that never runs | trivial | ⬜ |
| 11 | Power telemetry into `/rover_diag` | READINESS #2; the rover has run itself flat | small | ⬜ |
| 12 | Search-step overlap and a progress message | a failed search is ~4 min of silence | small | ⬜ |
| 13 | Commit the flow walkthrough as a doc | it exists only in a terminal today | small | ⬜ |
| 14 | Stale camera view after the robot moves | answered "what can you see" from a photo of where it used to be | small | 🔄 shipped but **never executed**, and the prompt contradicts it — see [PERCEPTION_STATE.md](PERCEPTION_STATE.md) |

---

## 14. 🔄 Stale camera view after the robot moves — SHIPPED, NOT VERIFIED

`look()` injects the frame as a HumanMessage labelled `[Current camera view]`
and it **keeps that label for the rest of the conversation**. `local_agent` has
`keep_images=True`. So "go forward 30, turn 180, now what can you see" was
answered from a photo of the place the robot had just left — confidently, with
nothing anywhere saying the view was old.

`look()`'s own docstring made it worse: it told the model not to look again
"unless the scene may have changed", which the model has no way to know.

### The fix is NOT to strip stale images — that was my first recommendation and it was wrong

`utils/message_utils.py` opens with a **cache invariant**: the projection must be
append-only, because each agent's llama.cpp slot caches the KV of its previous
prompt. An image present in turn N and gone in turn N+1 diverges the cached
prefix and re-prefills everything after it — worst exactly where it would bite,
on `local_agent`'s slot with a camera frame in it.

So the movement result carries the news instead. Append-only by construction,
costs a few tokens, and reaches the model at the moment the movement is
reported. Every tool that drives the base appends it: `move_robot`,
`navigate_to_pose`, `approach_described_object`, `scan_surroundings`. `look()`'s
docstrings now say a fresh look is **required** after any movement.

Two edges worth keeping: a bare `S` gets **no** note (it moves nothing, and a
false alarm burns a 10–40 s look), and a sequence interrupted during its **first**
step **does** get one (the base still drove part of the way, and "completed:
nothing" would hide that). The test for the second is what caught it — the first
version keyed off `at > 0` and was wrong.

Pi 5 repo `86e0b08`. 184 tests pass.

**Live in Studio now. NOT live for Telegram/voice** until `agent_node` restarts,
which needs a password: `ssh 192.168.1.16 'sudo systemctl restart langrobo-brain'`.

**Still open:** this is an instruction to the model, not an enforcement. If the
12B model ignores the note and answers from the old photo anyway, the next step
is #8-style verification or an epoch tag the model must reconcile — but measure
whether it actually ignores it before building that.

### Reopened 2026-09-10 evening — read PERCEPTION_STATE.md before touching this

The user reported the symptom again. Two findings, neither of which is "the
12B model ignored the note":

1. **The fix has never executed.** `langrobo_core` is editable, so the source
   looks live, but Python imports at process start. The `agent_node` that could
   have served the test started **13:20:01** — 57 min before the 14:17 commit.
   The current one (after the 14:33 reboot) has it and has served **zero** turns.
   Restart and reproduce before judging. Same trap as the rotation fix.

2. **`86e0b08` never touched `prompts.py`.** `LOCAL_AGENT_PROMPT` rules 2 and 3
   still say "Do NOT call look() again" and gate looking on "the last view is
   stale" — the one condition the model cannot evaluate. A sentence in a
   `ToolMessage` does not outrank a numbered system-prompt rule, and the note is
   written during a **navigate** turn while the question is answered by
   **local_agent**. Fix the prompt first; it is ~5 lines and costs no cache.

The design that replaces the note — stamp the frame with the pose it was taken
from, frame each turn with the current pose, and gate on the difference — is in
**[PERCEPTION_STATE.md](PERCEPTION_STATE.md)**, along with the KV-cache rules
any fix must obey and a correction to this entry's cache reasoning.

---

## 1. ⬜ Test whether the chassis can pivot in place

**Do this first.** It is five minutes and it decides #2.

Every approach goal carries a yaw facing the object (`pixel_to_goal.py`
`compute_standoff_goal`). But nav2 here runs with **rotate-to-heading and spin
recovery disabled**, because TODO 14 says this rover cannot turn in place. If
that is true, the yaw in every approach goal is a requirement nav2 can never
satisfy: `follow_path` aborts, `backup` runs instead, and the BT eventually
gives up — which is exactly TODO 34's run A.

Two claims in this repo contradict each other:

- TODO 14: the rover cannot turn in place.
- `teleop_web.py`: `left`/`right` "pivot in place (both wheels counter-rotate)".

**One of them is stale, and nobody has checked which.** Both comments are old.

**How:** `logs/wzsweep.py` finds the commanded `wz` that actually rotates this
chassis; `logs/yaw_crosscheck.py` does a 360° spin against all four yaw sources.
Drive a pivot from teleop first and just watch it.

**Decide:** whether TODO 14 is still true. Everything about #2 follows.

---

## 2. ⬜ Make the approach goal yaw-agnostic — ONLY if #1 says it cannot pivot

**Depends on #1.**

If the rover genuinely cannot pivot, stop asking nav2 for a heading it cannot
reach. Two options:

- a goal checker that ignores orientation, or
- set the goal yaw to the robot's **current** heading rather than the bearing to
  the object.

The standoff *position* is what matters. Facing the object is a nicety — and the
camera has an 87° FOV, so being roughly pointed at it is usually enough.

**If #1 says it CAN pivot:** re-enable rotate-to-heading instead, and this item
becomes a config change rather than a geometry change.

**Risk:** if the goal yaw stops facing the object, the follow-up look in #8 may
not have the object in frame. The two interact — decide them together.

---

## 3. ⬜ Costmap-check the standoff goal before sending it

`compute_standoff_goal` is **pure geometry**. It never asks whether the point it
produced is somewhere the robot can actually be.

A goal 0.45 m in front of a chair can land inside that chair's inflation radius.
And the map is ~74% unknown — a goal in unknown space is unplannable. Either way
nav2 aborts, and it reads as a planner fault rather than a bad goal. TODO 34
suspects exactly this for run B.

**Fix:** after computing the goal, sample the costmap or the nvblox occupancy
grid at that cell. If it is lethal, inflated or unknown, walk the standoff
outward — 0.45 → 0.6 → 0.8 → 1.0 m — and take the first free one. Put the
standoff actually used into the reply JSON so the brain can say "I stopped a bit
further back, there was no room".

**Cost:** `pixel_to_goal` gains a costmap subscription. It currently subscribes
to camera_info, aligned depth and the query topic only.

**Watch:** the TF lookup in `_on_query` deliberately has **no timeout** because
it runs inside the single-threaded executor callback. Any new blocking lookup
added here has the same trap.

---

## 4. ⬜ Downscale the frame sent to the VLM

**The biggest cheap win in the whole path.** The look is 10–40 s and dominates
every approach; nothing else in the flow is above ~50 ms except the drive.

Today `image_bridge` sends a full **896×504** JPEG at quality 80 (~40 KB) for a
task whose entire output is a coarse centre point.

**The contract already supports this for free.** The VLM answers in **0–1000
normalised coordinates**, and `_vlm_locate` scales by the frame's *own* width and
height. So a half-size image yields the **identical** final pixel. Nothing
downstream changes — not `pixel_to_goal`, not the intrinsics, not the depth
lookup, because the pixel is rescaled to the colour frame's real size before it
is ever sent.

Halving to 448×252 cuts vision prefill roughly 4× in tokens.

**Fix:** one `cv2.resize` in `image_bridge.py` before `cv2.imencode`.

**Risk:** accuracy on small or distant objects. Test on real targets — a bottle
across the room is the case that would break first. Keep `look()`'s scene
description in mind too: the same topic feeds it, and a scene description may
want more pixels than a pointing task. **If they disagree, that is an argument
for two topics, not for skipping this.**

---

## 5. ⬜ Retry once when the VLM's reply will not parse

If the model returns malformed JSON, a missing key, or a coordinate outside
0–1000, `_vlm_locate` returns `None` — indistinguishable from "not found". The
search then **rotates 90° away from an object that may be in plain view**.

One retry on the *same* frame, before rotating, costs one look and saves a
wasted rotation plus a look. LLM JSON compliance is not perfect and this is the
cheapest possible mitigation.

**Effort:** trivial. **Risk:** none. Bound it at one retry so a broken model
cannot double the whole search.

---

## 6. ⬜ Guard the stale-pixel race

The VLM's pixel comes from a frame captured at time **T**. The answer arrives at
**T+10 to T+40 s**. `pixel_to_goal` then grounds that pixel using depth and TF
from **T+40**, freshness-gated to 1.5 s.

**The pixel and the geometry come from different moments, and nothing checks
that the robot did not move in between.**

It is safe today only because the robot is stationary during the look: the
search rotation completes before `_fresh_frame()` grabs anything, and nothing
else drives during the call. That is an accident of the current design, not an
invariant anyone enforces.

If anything ever moves the base during the VLM call — a nav goal that has not
fully cancelled, teleop, or a future "look while driving" — the result is a
**confidently wrong goal**, not an error. The robot drives somewhere plausible
and reports success.

**Fix:** capture the odom pose at frame-grab time, send it in
`/vision/pixel_query`, and have `pixel_to_goal` reply
`{"ok": false, "reason": "robot_moved"}` if the current pose differs by more than
a few cm / degrees. The brain can then simply look again.

**Note:** using the TF *at the capture timestamp* instead does **not** work — the
TF buffer holds 10 s and the VLM can take 40 s.

**Effort:** small — two fields and one comparison. **Risk:** very low.

---

## 7. ⬜ Sanity-check the object's height

The 3D point is computed in full, then only `(x, y)` survive — `rot_z` is
calculated and thrown away. The goal is 2D.

Mostly right for a ground robot. But if the VLM points at a clock, a picture, a
switch, or a cup on a high shelf, the robot computes a floor position beneath it,
drives there, and reports **success**. The geometry is not wrong; it is
answering a question that does not make sense.

**Fix:** keep `z`. If the object sits above ~1.2–1.5 m, refuse with something
honest — "I can see it, but it's up on the wall, I can't drive to it" — or at
minimum return the height so the brain can say something sensible.

**Effort:** small, the number is already computed. **Risk:** none. Pick the
threshold against the camera mount height, which per TODO 6 has **never been
measured** — so either measure it or choose the threshold conservatively.

---

## 8. ⬜ Verify arrival by looking again

**Arguable — decide the latency trade explicitly.**

On nav2 success the robot says "I've arrived — I'm right by the chair." It never
looks again. Success means **nav2 reached a coordinate**, not **the chair is
there**. If the chair moved, or the depth median landed on a hole and the goal
was off by half a metre, the robot still reports arrival confidently.

This is the same class of dishonesty as the silent cuVSLAM fallback — a green
result that is not evidence of the thing it implies. That one cost 21 m of
driving and two rounds of fixes (TODO 21, 30).

**Fix:** on success, one more `_fresh_frame` + `_vlm_locate`. Found → "I'm here,
it's right in front of me." Not found → say that instead.

**Cost:** one extra VLM round-trip on **every** approach. At today's 10–40 s that
is a real tax; **after #4 it may be 5–15 s**, which is why this sits after it.

**Interacts with #2:** if the goal stops facing the object, the object may not be
in frame on arrival and this check would produce false negatives.

---

## 9. ⬜ Fix the Studio / agent_node conflict properly

Today `agent_node._warn_if_competing_bridge()` logs an ERROR when
`/studio_bridge` appears and leaves it to a human to act.

Studio runs the same graph, polls the same Telegram bot, and publishes the same
`/cmd_vel`. Only `agent_node` registers a nav-done callback, so a navigation
Studio picks up finishes and reports to nobody (`listener=NONE`). Measured
2026-09-10; it cost two sessions of hunting a "missing" arrival message.

**A warning that depends on someone reading logs is not a fix.** Options:

- Studio's bridge registers a nav-done callback too, so at least the report
  lands somewhere; or
- whichever bridge starts **second** refuses to poll Telegram, so exactly one
  process owns the conversation.

The second is the honest one — the two are alternatives, not additions, and the
code should enforce what STARTUP.md §6 currently only asks of the operator.

**Note:** the guard's earlier `*_bridge` wildcard fired on `image_bridge` (the
Jetson's JPEG publisher, a required part of the `vlm` layer). That is already
fixed to a named set — do not reintroduce a wildcard.

---

## 10. ⬜ Delete the Pi 5's dead copy of `compute_standoff_goal`

Checked 2026-09-10: `langrobo_core/tools/approach.py:49` defines
`compute_standoff_goal`, and the **only** thing that calls it is
`tests/test_approach.py`. The production path uses `res["goal"]` straight from
the Jetson's reply.

So there are two implementations, in two repos, in different units (degrees vs
radians), kept in sync by hand and by a shared `LANGROBO_STANDOFF_M` — and **the
one with unit tests is the one that never runs.** That is worse than no test,
because it reads as coverage of a path it does not cover.

**Fix:** delete it and move those tests against the Jetson's copy, or keep it and
mark it reference-only in the docstring. Either is fine; the current state is
the one that misleads.

---

## 11. ⬜ Power telemetry into `/rover_diag`

Already READINESS.md #2, repeated here because it keeps being the cheapest large
win available and it is not part of any glamorous feature.

The rover has run itself flat with every gate green. Nothing can refuse a
mission on low battery, return to charge, or explain a stall. Two more floats in
a `Vector3` that already exists turns a silent failure into a readable state.

---

## 12. ⬜ Search-step overlap, and a progress message

Two small things in the same loop:

- **Overlap.** The camera FOV is 87° and the search steps 90°, so consecutive
  looks leave a small blind gap. In practice odometry under-rotates and the gap
  probably closes — but that is an accident being relied on, not a margin.
  ~75° steps would give real overlap, at the cost of more looks.
- **Silence.** Five steps at up to 40 s each is roughly **four minutes** during
  which the user gets nothing at all, then a failure. A "still looking…" message
  after step 2 costs nothing.

Both get much cheaper if #4 lands first.

---

## 13. ⬜ Commit the flow walkthrough as `docs/FLOW_APPROACH.md`

The full input→output walkthrough of this path — every stage, both machines,
real intrinsics, worked numbers, and the failure message each stage produces —
currently exists only in a terminal session.

Given how much of this system's usable knowledge lives in `TODO.md` and
`STARTUP.md`, it belongs in the repo. Include the worked example, which uses the
camera's real values:

```
fx 450.26  fy 449.71  cx 440.17  cy 248.46   frame 896x504
VLM (520, 470)/1000  ->  pixel (465.9, 236.9)
depth median 1.19 m  ->  cam (right +0.068, down -0.031, fwd 1.190)
base_link            ->  forward 1.360, left -0.068
OBJECT odom (1.360, -0.068)   GOAL odom (0.911, -0.046) yaw -2.86 deg
```

Note `depth 1.19 -> forward 1.36`: `base_link` sits 0.17 m behind the camera.
Both numbers are right and they are not interchangeable.

---

## Already tracked elsewhere — not repeated above

- **TODO 34** — nav2 fails to reach anything. Items #1, #2 and #3 are concrete
  attacks on it; the entry itself stays the record.
- **TODO 14** — rotate-to-heading disabled. #1 tests its premise.
- **TODO 6** — camera mount pitch never measured. #7 wants that number.
- **TODO 31** — frame gaps of 1.4–1.9 s under full load. Relevant to #4: less
  work in `image_bridge` is less load on the same GPU.
- **TODO 32** — the four things the brain asks for that this rover never
  answers. The `minimal` brain removed the tools; the gap is dormant, not fixed.
- **READINESS** — camera needs hands, power is invisible, nothing looks down,
  no memory between sessions. None of those are touched by anything above.

---

## Suggested first three, if the list is too long to start

1. **#1** — five minutes, and it gates the only 🔴 in this path.
2. **#4** — one line, possibly 2–3× faster approaches, and the normalised
   coordinate contract makes it safe by construction.
3. **#3** — turns a plausible-looking nav2 abort into a goal that works.

**#6** and **#7** are the two to add next. Both are small, and both close a
*confidently wrong* hole rather than a *throws an error* one — which is the
failure mode this rover keeps getting caught by.
