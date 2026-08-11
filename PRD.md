# PRD — what we are building

## The one sentence

**Click a goal in RViz, and the rover drives there, on a map it built itself.**

If a change does not move that sentence closer, it does not belong in this repo.

---

## Why this repo exists

The rover already "works" in the sense that all the pieces run. The problem is
different: **the system is bigger than what one person can hold in their head**,
so when something goes wrong there is no way to tell which of eight interacting
layers did it.

Two concrete symptoms of that:

- On 2026-08-11 the stack ran for an hour with **two nvblox nodes** competing,
  each building its own map and both publishing to the same topic. Nobody
  noticed, because everything starts at once and "it looks like it's running".
- The map came out as a solid blob and it took a full day of measurement to
  establish the cause was not in nvblox at all.

So the goal of this repo is not new capability. It is **legibility**: every
layer can be run alone, proven alone, and understood alone.

---

## Definition of done

Seven things, in order. Each is a task, each has a pass/fail gate.

1. The camera streams, alone, stably, for five minutes.
2. cuVSLAM reports a **tape-measured 2.00 m push within 5%**.
3. The fused pose survives the camera being briefly blinded.
4. Driving a room produces walls that are **lines, not blobs**.
5. That map can be **saved and reloaded**, and the rover finds itself on it.
6. Obstacles are detected without the floor being called an obstacle.
7. A goal clicked in RViz makes the rover drive there and stop.

---

## Explicitly NOT in scope

These were in the old system. They are not deleted — they live in the parts bin
at `langrobo_perception/` — but they are **out of this repo** until item 7 above
passes.

| Out | Why |
|---|---|
| "Go near the bottle" / object approach | Needs YOLO + detections_3d + a working pose. It was chasing objects on an unverified pose, which is how you get confident nonsense. |
| Voice / Telegram / the Pi 5 brain | A separate machine and a separate problem. The rover must work when driven from RViz first. |
| Place memory ("go to the kitchen") | Needs item 5 (a map that persists) to even mean anything. |
| YOLO object detection | Costs Orin CPU, and CPU starvation is a **safety** property here (see FACTS §1). Not until the core is stable. |
| A URDF | A marker gives the same visual result for a box on two wheels, without joint states to drift out of sync. |

---

## Constraints that shape every decision

| Constraint | Consequence |
|---|---|
| The camera looks **forward only, ~87°** | A single viewpoint can only ever produce a wedge. Rooms require driving. There is no fixing this in software. |
| The D555 dies if you poke it | No ad-hoc subscribers on raw camera topics. Minimise restarts. See FACTS §1. |
| The wifi link is nearly full | ~290 kB/s of map already. Anything added to the laptop view must be measured. |
| CPU load is a safety property | Above ~8 load on 6 cores the camera starves and **cuVSLAM freezes silently**, and the stack then navigates on a frozen pose. |
| Everything is DHCP | Resolve names. Never trust a hardcoded IP. |
| The rover is tethered | No blind recovery behaviours. The behaviour tree has no Spin or BackUp. |

---

## The rules

These exist because breaking each of them has already cost a session.

1. **Never advance past a gate you have not actually verified.** "It looked
   fine" is not a gate.
2. **One layer at a time.** If two things changed, you learn nothing from the
   result.
3. **The machine beats the doc.** If this repo disagrees with a measurement,
   the measurement wins — fix the doc in the same commit.
4. **Measure rates, not existence.** Every failure here has been a rate
   collapsing, not a topic vanishing.
5. **Check pose trust before believing anything spatial.** A frozen cuVSLAM
   still reports `slam_pose_ok: true`.
6. **Nothing enters `src/` until it has been run, tested and understood.**
   That is the promotion rule, and it is the whole point of this repo.
