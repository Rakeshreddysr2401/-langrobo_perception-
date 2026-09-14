# System audit — 2026-09-14, both boxes read as one system

Read live: `~/rover` on the Jetson (this repo, branch `rover-v1.1.2-refactor`,
HEAD `7d432c5`) and `~/ros2_ws` on the Pi 5 over SSH (branch
`dev-1.3.3-minimal`, HEAD `8d02651`), plus `READINESS.md`, `INTEGRATION_GAPS.md`
(Pi 5 repo) and both `TODO.md`s. Every item below was checked against current
code or a current commit, not just quoted from a doc — several things the docs
still call "open" turned out fixed, and one thing the docs call "closed" turned
out not actually running by default. That gap — a doc going stale relative to
the other repo — is itself the biggest recurring failure mode on this project,
so it gets its own item (§6).

Numbered for interview: pull up one number at a time. 🔴 blocks something now ·
🟠 real, worked around · 🟡 unverified · ⚪ accepted/deferred by you already.

---

## Part A — Open items, ranked by what unblocks the most

### 1. 🔴 The single biggest unanswered question: does Nav2 actually pivot, or does it curve?

Two different subsystems turn this chassis at two different, conflicting
speeds, and nobody has run the one test that would settle which (if either) is
right:

- **`move_robot()` on the Pi 5** (`langrobo_core/tools/movement.py`) commands
  turns directly on `/cmd_vel` at **5.0 rad/s** — matching `teleop_web.py`'s
  measured clean pivot speed.
- **Nav2 on the Jetson** (`phase3/config/nav2.yaml`) caps all rotation —
  `velocity_smoother`, the new `Spin` recovery added in TODO §40 — at
  **1.5 rad/s**.
- Teleop's own measurement says 2.0 rad/s (47% duty) **cannot break the four
  tyres loose sideways at all** — a commanded pivot becomes a forward/backward
  curve. 1.5 is slower still. If that holds, every Nav2-driven rotation
  (including the brand-new Spin recovery meant to get it out of tight spots)
  is silently curving, not turning in place.
- This is a live, three-way cross-reference: Jetson TODO §40 (2026-09-11), Pi 5
  `INTEGRATION_GAPS.md` §3 (updated 2026-09-11 to match), and Jetson TODO §37
  (rotation-induced x,y drift, still unverified) are all pointing at the same
  unrun test.
- **The test that settles it, already written**: command a pure rotation
  through Nav2 and log `/odom` x **and** y, not just yaw — a curve and a pivot
  produce identical yaw-rate but different x,y. `logs/yaw_crosscheck.py` and
  `./rover compare --spin 360` exist for this. Nobody has run it since the
  1.5 rad/s Spin config went in.
- **Why it matters beyond Spin**: if turns curve, it's a candidate root cause
  for both §37 (rotation drift) and §36 (timed turns undershooting) at once —
  one test, two open bugs possibly explained.

### 2. 🔴 The camera needs hands — unchanged since 2026-08-26

D555 drops out in three distinct ways, all requiring a physical PoE reseat; it
**pings while its DDS server is dead**, so no software check catches it and no
software restart recovers it. This is the standing reason a human has to watch
every run, and nothing has moved on it. Real options worth discussing: a
switchable PoE port/relay under agent or watchdog control, or accepting
supervised-only operation as the design point for now given the "agentic dev
on Pi5" direction rather than full autonomy.

### 3. 🟠 A fix that's "done" but not actually running by default

Jetson TODO §32 declares the vision seam **closed and verified on hardware**
2026-09-09: `phase4/nodes/detections_3d.py` (YOLOv8n + D555 depth → odom-frame
JSON) now publishes `/vision/detections_3d`, which lights up `approach_object`,
`where_is`, `list_known_objects`, `scan_surroundings`, and the whole
`world_model` service on the Pi 5 side.

**But `detect` is not in `./rover up`'s bring-up sequence** — the script's own
comment names it explicitly: `camera → pose → fused → map → nav → vlm →
wheels → Pi5 → Studio → view`. No `detect`. So on any normal bring-up, those
five brain-side tools are back to their pre-fix failure mode — searching,
finding nothing, answering "I haven't seen one before either" about an object
in plain view — unless someone remembers to run `./rover detect` by hand.
**Decide: fold it into `up`, or is there a reason (GPU/CPU budget alongside
nvblox+nav2, per JETSON_LOAD.md) it's deliberately manual?**

### 4. 🟠 The two repos' own gap-tracking docs have drifted from each other again

Pi 5's `INTEGRATION_GAPS.md` §1 still lists `/vision/detections_3d`,
`/vision/target` and `/vision/target_result` as **⬜ open** — the exact three
topics Jetson TODO §32 says were closed and verified five days earlier
(2026-09-09). `INTEGRATION_GAPS.md` was touched again on 2026-09-11 (the nav2
pivot addendum in §3) without anyone revisiting §1. This is precisely the
failure class that file was written to catch — a topic one side thinks exists
and the other doesn't know about — and it just happened to the doc itself.
Worth a five-minute pass reconciling §1 against Jetson TODO §32, and worth
asking whether one of these two files should be the single source of truth
instead of two that can silently diverge.

### 5. 🟠 No power telemetry on the ESP32 link — cheap, still not done

`/rover_diag` is a `Vector3` carrying loop Hz and free heap; two floats are
sitting unused in a message that already exists and already round-trips at
20 Hz. On 2026-08-26 the rover decayed from 0.77 rad/s of rotation to **no
motion at all** while every ROS topic stayed green — completely invisible to
software. This is flagged as the "cheapest high-value fix" in `READINESS.md`
and nothing has changed since.

### 6. 🟡 Camera QoS mismatch risk — unconfirmed either way, silent-failure-shaped

`/camera/color/image_raw/compressed` is RELIABLE on both the Jetson publisher
and (by omission — ROS2 default) the Pi 5 subscriber, over WiFi. Reliable
large messages retransmit and can head-of-line block; sensor streams are
conventionally BEST_EFFORT. An incompatible-QoS pairing (one side reliable,
other best-effort) fails **completely silently** in ROS2 — exactly the failure
mode that has bitten this project before (§32's whole seam, the frame-id bugs).
Neither side has been changed. If changed, both sides must move together in
one commit.

### 7. 🟡 Two stale TODO entries on the Pi 5, each with an unmet "delete when done"

- **"Mac Mini llama.cpp returns Compute error on every request"** — dated
  2026-07-10. `FLEET_STATUS.md` (2026-09-06) shows the same Mac Mini answering
  `primary_available: true` and serving real completions. This entry almost
  certainly should have been deleted two months ago and wasn't — worth a
  30-second `curl` to confirm and close it, since a stale "known broken"
  entry is worse than no entry (it teaches people to ignore real ones).
- **"Pi5↔Jetson ethernet cable is carrier-only, no data"** — also 2026-07-10,
  matches `NETWORKING.md`'s own last-updated note on the same date. Genuinely
  unknown whether this was ever fixed; `NETWORKING.md` says WiFi remains the
  fallback path either way, so it's low urgency, but it's exactly the kind of
  thing worth a five-minute physical check (reseat the cable, check
  `rx_packets` on the Jetson NIC) next time you're at the machines.

### 8. 🟡 `nav2.yaml`'s own comments disagree with its own numbers

`READINESS.md` already flagged this and it's still true: the smoother's
angular cap is `1.5` commanded, while the comment beside a related constant
reasons about `2.5`. Small, but it's the kind of inconsistency that makes the
config harder to trust the next time someone has to touch it — worth a
one-line reconciliation pass while §1 is being worked anyway.

### 9. 🟡 Studio's CORS fix lives outside both repos

Patched by hand in `~/.local/lib/python3.12/site-packages/langgraph_api/
server.py` on the Pi 5 — not in git, not in a commit, gone on the next
`pip install -U langgraph*` or a fresh SD card. The durable fix is a
`langgraph` version bump, deliberately deferred because it also touches the
graph runtime `agent_node` depends on. Worth scheduling deliberately rather
than rediscovering the blank-page bug cold next reflash.

### 10. 🟡 Rotation-induced x,y drift — reported, never run against a controlled test

You deferred this yourself 2026-09-10 ("minor issue, will see it later"). It's
resurfaced here because it may share a root cause with item 1 above — if Nav2
turns are curving, that alone could produce exactly this symptom. Recommend
folding this into item 1's test rather than treating it as separate: run
`./rover compare --spin 360` and `--return` once, and both items get an answer
from the same drive.

### 11. 🟡 VLM pixel coordinates spoken aloud once, unreproduced

Reported once ("x:477, y:647" spoken by the robot); mechanism is understood
(no filter between a tool reply and TTS) but not caught in any journal entry
that day, so which code path emitted it is still unknown. Not actionable
without a repro — flagging so it's not forgotten, not because there's a
decision to make yet.

### 12. 🟡 Voice first-sentence latency is structural, not yet decided

Local Kokoro TTS has RTF ~1.8 on the Pi 5 (a 3 s sentence takes 5.4 s to
synthesize), and only sentences 2..N get pipelined — the first sentence, the
one the user is actually waiting on, can't be hidden. `sarvam_translate` is
already the sub-second default and Kokoro is the offline fallback, so this
may already be adequately handled — worth confirming that's the intended
resting state rather than a still-open problem.

### 13. ⚪ Known, deliberate, worth re-confirming given the new direction

- **No persistent map / no relocalization** — matches your stated preference
  (fresh start each session), but it also means Nav2's global frame stays
  `odom`-only, permanently. Worth a explicit "yes, still the right call" now
  that the roadmap says "agentic dev on the Pi 5" — a persistent map changes
  what an agent can be asked to do ("go back to the kitchen" vs. only
  relative moves).
- **`/servo_pan`, `/servo_tilt`, `/audio/music_*`** — gated off, no hardware
  exists for either. Purely a "still true, still fine" check.
- **`agent_node` runs a SingleThreadedExecutor** — nothing measured broken
  today, but `/voice/user_input` shares a thread with a 5 Hz image callback.
  Flagged as a latent latency risk if voice-under-load ever gets reported.

---

## Part B — Design ideas worth a deliberate conversation (not bugs)

- **Fold `./rover detect` into `./rover up`**, or explicitly document why not
  (see §3) — this is the one with the most leverage per line changed.
- **MiniLM entry classifier for intent routing** — already designed in
  `INTENT_ROUTING_PLAN.md` on the Pi 5, not built. Good candidate for "what to
  build next" now that hardware bugs are thinning out.
- **A `map`-framed twin of `/fusion/path`** — loop closure corrections are
  currently invisible in RViz by construction (`/fusion/path` is stamped
  `odom`, Fixed Frame is `odom`). Cheap visibility win.
- **MultiThreadedExecutor for `agent_node`** — pre-emptive answer to §13's
  last bullet if voice-under-load ever becomes a real complaint.
- **Switchable PoE / camera watchdog power-cycle** — the actual unlock for
  unattended operation (§2). Bigger lift, biggest payoff.
- **A single shared seam doc instead of two that can drift** (§4) — either
  generate `INTEGRATION_GAPS.md`'s topic table from a script that greps both
  repos, or designate one file as canonical and have the other link to it.

---

## What's already fine and doesn't need interview time

Encoders (calibrated, steady 20 Hz), fusion's core logic (CPU-bound but
working), the discovery-server networking scheme (NETWORKING.md's design is
sound and running), the wake-word/noise-gate/barge-in voice fixes (all
verified 2026-09-05/06), the rotation *speed* calibration in `movement.py`
(§3 of `INTEGRATION_GAPS.md`, correctly diagnosed and fixed against the v2
firmware), and the nav2 doorway/padding/recovery fix in TODO §40 (sound
reasoning, just not yet floor-tested — see item 1).
