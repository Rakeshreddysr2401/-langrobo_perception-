# ThingsTodo — the rebuild plan

**Written 2026-08-10.** This replaces the free-form note that used to be here (kept
verbatim at the bottom, §Appendix — it is the requirement, and nothing in it is dropped).

## The complaint, restated

> *"If I ask it to go near the bottle it moves differently, and I don't even have any idea
> what is going on, what is integrated, and how it is working."*

That is **two** problems, and they got solved in the wrong order last time:

| | |
|---|---|
| **Problem A** | The rover misbehaves. |
| **Problem B** | You cannot *see* what it believes, so you cannot tell *why* it misbehaves. |

Every past session attacked A while B was unsolved — which is why `SESSION_2026-08-10.md`
ends with **seven independent silent failures**, none of which logged an error. They were
invisible, not subtle. So this plan fixes **B first**, and never advances a phase until the
current one is *visible on a screen and passing a stated gate*.

**Rule for the whole plan: no phase is "done" because the code exists. It is done when you
watched it work and it passed the gate.**

---

## What already exists (do NOT rebuild it)

You have far more than the note assumes. Rebuilding from zero would throw away weeks of
measured, hard-won fixes. Concretely, already built and proven:

- **cuVSLAM** full SLAM — `map→odom` live, loop closure, `/slam/save_map`, `/slam/localize`
- **nvblox** 3D map + ESDF slice → costmap
- **nav2** planner + MPPI + a deliberately safe BT (no blind Spin/BackUp)
- **EKF fusion** (cuVSLAM pose + D555 gyro + wheel vx) — `config/ekf.yaml`
- **`/odom/health`** — the rover can already tell you when its own pose is lying
- **safety_guard** — 3 forward bumper layers, verified live
- **Laptop RViz config** — `config/laptop_view.rviz` (TF, nvblox walls, `/plan`, odom trail,
  bumper marker, camera image)
- **Phone teleop** — `http://192.168.1.16:8091`, hold-to-drive

So this is a **restructure and a verification campaign**, not a from-scratch build. The
missing pieces are narrower than they feel: **exploration**, **place memory**, **map reload
across sessions**, and above all **the ability to watch any of it**.

---

## Findings from tonight's system sweep

These are measured right now, and two of them are why "look at RViz" has been failing.

| # | Finding | Impact |
|---|---|---|
| **F1** | **The laptop is at `192.168.1.10`, not `192.168.1.12`.** `192.168.1.12` is now the **ESP32** (`rover-esp32.local` resolves there; ssh refused; different MAC). DHCP reshuffled them. | README §7 and `skills/rover-start/SKILL.md:72` both send you to the ESP32. **This alone breaks "show me RViz on my other laptop."** |
| **F2** | **The laptop is sitting at the GDM login screen** (`loginctl`: only `gdm` holds seat0). | RViz launched over ssh renders **invisibly**. You must physically log in first. |
| **F3** | **D555 ethernet link is UP again** — `enP8p1s0` has carrier, camera answers at `192.168.11.55`. Yesterday it was `NO-CARRIER`. | The blocker at the end of `SESSION_2026-08-10.md` is **cleared**. |
| **F4** | Container `orin_nav` is **exited**; nothing is running. Orin is idle (load 0.7, 4.2 GB free). | Clean slate for Phase 0. |
| **F5** | **No exploration code exists anywhere** in the repo (`standalone/run_stereo_explore.py` is a *rerun web viewer*, not a frontier explorer). | Want #4 is genuinely new work, not a fix. |
| **F6** | **No place memory on the Jetson.** `save_location` lives in the Pi5 brain (Qdrant) and is not wired to the Jetson map frame. | Want #5 is new work. |
| **F7** | Saved map in `maps/current` is from **Jul 19** — the stationary 15 s capture README §13 says is too sparse to relocalize against. | Treat as empty. Phase 4 makes the first real one. |
| **F8** | Doc drift: README §10 still says **"IR emitter ON"** (stack runs it **OFF** — that was the ×4 scale bug), and §8 still quotes speed caps from the **retired** `cmd_vel_deadband.py`. | Docs actively mislead. Fixed in Phase 0. |

`~/orin-nav-stack` is a **symlink** to this repo, and `run_stack.sh up` mounts it read-only
over the baked image — so **config/node edits take effect on the next restart, no rebuild**.
That is the loop you will use all plan long.

---

## The plan — 7 phases

Each phase states: **Goal · What you will learn · Gate · Known trap**.
Do not skip. Do not run two phases at once. **The gate is the whole point.**

---

### Phase 0 — Instruments before anything moves ✅ **DONE 2026-08-10**

*Nothing here touched a motor.*

**Gate result — PASSED.** With perception up (`./run_stack.sh up`), measured:

```
  ok camera IR left       29.5 Hz   >=15 Hz
  ok camera depth         25.8 Hz   >=10 Hz
  ok cuVSLAM odom         28.5 Hz   >=10 Hz
  ok nvblox slice          9.5 Hz    >=5 Hz
  !! ESP32 encoders        1.0 Hz   >=15 Hz   LOW — still on OLD firmware, reflash it
  ok TF map->odom        0.06s old
  ok TF odom->base_link  0.06s old
  ok orin load1      2.3
```

RViz is live on the laptop; from the laptop `/odom` measures **30.1 Hz** and 73 topics are
visible, so the network path is genuinely working — not just "rviz2 is running".

**What the new instrument caught on its very first run:** the ESP32 at **exactly 1.0 Hz** —
the Phase 3 blocker — which the old publisher-count status reported as a healthy `1`.

Delivered: `nodes/stack_status.py`, `./run_stack.sh view [start]`, rewritten `status`,
and the doc corrections below. Two bugs found and fixed while building it: the status node
enumerated the topic graph *before* DDS discovery finished (reported "no publisher" for a
fully healthy stack), and it counted messages before subscriptions had matched publishers
(every rate read low).

**Goal.** One screen that shows the truth, one command that reports health, and docs that
are not lying to you.

**Tasks**
1. **Fix F1/F2** — correct the laptop IP to `192.168.1.10` in `README.md` §7 and
   `skills/rover-start/SKILL.md`; add the "log in at the laptop physically first" step as a
   precondition, not a footnote.
2. **Fix F8** — correct README §10 (emitter OFF, with the ×4 reason) and §8 (deadband
   retired; real caps live in `nav2.yaml`).
3. **Add `./run_stack.sh view`** — prints the laptop-side command, checks the laptop is
   logged in over ssh, and tells you plainly if it is not. No more silent invisible RViz.
4. **Upgrade `./run_stack.sh status`** into the single source of truth: camera Hz, cuVSLAM
   Hz, `map→odom` age, `/odom/health` trust, wheel link, nav2 lifecycle, **Orin load**.
   Today it prints publisher counts; you need **rates**, because every failure so far was a
   rate collapse, not an absence.

**What you will learn.** The TF tree (`map → odom → base_link → camera0_link`) and what each
frame *means* — `map` jumps on loop closure, `odom` is smooth but drifts. Nearly every
confusing behaviour in this robot is a frame or a rate problem.

**Gate.** RViz on the laptop shows a live TF tree and camera image, and `status` prints
non-zero rates for camera and cuVSLAM. **Nothing has moved yet.**

**Trap.** `pgrep -f rviz2` over ssh matches your own ssh command line. Use `pgrep -x rviz2`.

---

### Phase 1 — "Where am I, and which way did I go?" *(want #3)*

**Goal.** Drive by hand and watch the pose trail be *correct*.

**Tasks**
- `./run_stack.sh up`, then `fuse` (EKF owns `odom→base_link`).
- Watch the `/odom` arrow trail in RViz while you push the rover **by hand** (motors off —
  this isolates perception from the drivetrain entirely).
- Tape-measure check: push exactly 1.00 m, confirm the trail says ~1.00 m.
- **Tape-measure the camera height** and fix the `base_link→camera0_link` static TF in
  `run_stack.sh` (currently z=0.20; floor deprojection implies **~0.16**, see `nvblox.yaml`).

**What you will learn.** What visual odometry actually is, why the emitter must stay **OFF**
(ON → cuVSLAM under-read 100 cm as 26 cm), and how to read `/odom/health`.

**Gate.** 1.00 m push reads 0.95–1.05 m; `/odom/health` says `trust: true`; the trail is a
clean line, not a scribble.

**Trap.** Pushing fast (>0.5 m/s) explodes cuVSLAM tracking. Push **slowly**.

---

### Phase 2 — "Build a map as I drive it" *(wants #1, #2)*

**Goal.** Manual drive → walls appear in RViz → saved at end of session.

**Tasks**
- Phone teleop (`http://192.168.1.16:8091`, MANUAL, hold-to-move) — motors on for the first
  time in this plan, but **you** are the controller.
- `./run_stack.sh remap` for a fresh origin, then drive slowly and watch nvblox fill in.
- `ros2 service call /slam/save_map std_srvs/srv/Trigger` at the end.

**What you will learn.** Why the map is a **forward cone only (~87°)** — no head servo, so
it is blind to the sides and behind (gap G2). This is the single biggest reason the map looks
"wrong" and it is a *hardware* gap, not a bug.

**Gate.** Drive a full loop of one room; RViz shows recognisable walls; the save produces a
`data.mdb` + `.nvblx` newer than Jul 19 (F7).

**Trap.** Teleop publishes **straight to `/cmd_vel`, bypassing safety_guard**. Drive by
sight. Short taps for turns — fast pivots break tracking.

---

### Phase 3 — Trust the wheels *(want #6)*

**Goal.** Encoders genuinely fused, so the pose survives cuVSLAM dropouts.

**Tasks**
- **Reflash the ESP32** — it still emits `/wheel_state` at **1 Hz**; HEAD has published at
  20 Hz since `f4be55a`. The **source needs no change**. Flash `rover-esp32.local` (pick it
  from the Arduino IDE network port list — do not type an IP, per F1). `git pull` the
  firmware; do not copy-paste (last attempt flashed a truncated file that could not compile).
- Start the Pi5 relay: `ros2 run langrobo_ros wheel_odom_relay`, and **give it a systemd
  unit** — it currently dies on every reboot.
- Validate `/odometry/filtered` on a straight line and on a pivot.

**What you will learn.** What each sensor contributes: cuVSLAM absolute x/y/yaw, gyro yaw
rate, encoders forward velocity — and why the EKF needs all three.

**Gate.** `ros2 topic hz /wheel_state` ≈ 20 Hz; `/odom/health` reports `VO_SCALE_OFF` never
during a slow straight drive.

---

### Phase 4 — Remember places *(wants #2, #5)*

**Goal.** Name a spot, come back to it in a **later session**.

**Tasks**
- Wire **nvblox `global_frame: map`** + reload (gap G7 — currently anchored to `odom`, which
  is different every boot, so saved walls never line up).
- New node **`place_memory.py`**: `save <name>` snapshots the current `map` pose to a JSON in
  `maps/`; `list`; `goto <name>` → `navigate_to_pose`. Publishes markers so **saved places
  are visible in RViz**.
- Prove relocalization: save a map, restart the stack, `/slam/localize`, confirm the walls
  land on the old walls.

**Gate.** Save "kitchen", full stack restart, `goto kitchen` arrives within ~30 cm.

**Trap.** `remap` and `fuse` **both wipe the nvblox map**. Save before either.

---

### Phase 5 — Explore on its own *(want #4)*

**Goal.** No manual driving — it maps a room by itself.

**Tasks**
- New node **`explorer.py`**: read the nvblox occupancy grid → find frontiers (known-free
  cells adjacent to unknown) → send the nearest reachable one as a nav2 goal → repeat until
  none remain. Publish frontier markers so **you can watch it think**.
- Hard limits from day one: max radius from start, wall-clock timeout, and **abort the moment
  `/odom/health` says `trust: false`**.

**What you will learn.** Why exploration is mostly a *decision* problem, not a driving one —
nav2 already handles the driving.

**Gate.** Rover maps a room unattended for 5 minutes without a human touching it and without
a safety trip.

**Traps.** The ~87° forward cone means frontiers behind it are never observed — expect it to
need to turn in place to "look". And **CPU is a safety property**: the full stack already
sits at load ~9 on 6 cores. Check `ir_rate_hz` in `/odom/health` before and after adding
this node — if IR drops below 10 Hz, cuVSLAM freezes silently and never recovers.

---

### Phase 6 — "Go near the bottle" (the original complaint)

**Goal.** Re-run the flagship command, now on a foundation you can see.

**Tasks**
- `python3 nodes/approach_object.py bottle 0.40 90`
- Then the real upgrade (**gap G3**): make the Pi5 brain prefer the **metric 3D map pose**
  from `/vision/detections_3d` → `navigate_to_pose` (obstacle-aware), instead of today's
  **blind 2D mono visual servo** with no obstacle avoidance and no distance.

**Gate.** Two consecutive successful approaches, both with `/odom/health trust: true`, both
watched in RViz.

**Note.** By this point, if it misbehaves you will *see which stage* misbehaved. That is the
entire purpose of Phases 0–5.

---

## Order of work, and why

```
Phase 0  instruments ──► 1  pose ──► 2  map ──► 3  wheels ──► 4  memory ──► 5  explore ──► 6  objects
         (no motion)      (hand)     (teleop)   (reflash)      (persist)     (autonomous)   (flagship)
```

Each phase depends on the one before it being *verified*, not merely *coded*:
you cannot map without a trustworthy pose (2←1); you cannot return to a place without a map
that persists (4←2); you cannot explore without goals that reliably complete (5←4); and the
object approach needs all of it (6←5).

---

## Standing rules

1. **Never advance on an unverified gate.** Silent failure is this system's defining
   characteristic — seven of them at once, none logging an error.
2. **Check `/odom/health` before every move.** `trust: false` ⇒ do not drive, do not believe
   any "arrived".
3. **CPU is a safety property.** Confirm `ir_rate_hz` > 20 before adding any node.
4. **Save the map before `remap` or `fuse`.** Both wipe it.
5. **One change at a time**, then re-run the gate. The seven-bug session is what happens
   otherwise.
6. **When a doc disagrees with the machine, the machine is right** — fix the doc immediately
   (F1 cost you a whole "why can't I see RViz" session).

---

## Appendix — the original note, verbatim

> Hey Previously I tried with you to fix the code and move accordingly but it not worked  properly
> Like If Ask to Go Near Bottle It Moves Differntly And I Don't Even Have Idea of what going on and whjat all things integrated and how it working
> So Now Am Thinking to Have A New Setup or start complete Restructuring one by one doing all the things
>
> Example First:
> 1.will integrate cuVslam and nnblox and nav2 and see weather visually a map using Rviz(using my other laptop find docs)
> 2.Need to check like if i move vehicle manually it needs to build the map as it sees and also need to save for that sesion
> 3.Track of its movement like where it is and in which dircetion it moved all
> 4.It Also Like If I won't move maually also can capable of building and exploring
> 5.Remembering or note downing places with coordinates
> 6.Can use Encoders, IMUs what ever you need
>
> IMPORTANT: Thinking First To Have Proper Vision and Logs of what happening (use Other Laptop Already in DoCS and some setup)
> And Will check learn and do things. LEARNING is Also Important like what exactly happening and will implement one by one have a plan

**Coverage:** #1 → Phases 0+2 · #2 → Phases 2+4 · #3 → Phase 1 · #4 → Phase 5 ·
#5 → Phase 4 · #6 → Phase 3 · "vision and logs first" → **Phase 0, which gates everything.**
