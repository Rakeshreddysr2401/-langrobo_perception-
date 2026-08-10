# PRD — what we are building

## The problem this solves

> *"If I ask it to go near the bottle it moves differently, and I don't even
> have any idea what is going on, what is integrated, and how it is working."*

That sentence contains **two** separate problems, and past sessions kept
attacking the first one while the second was unsolved:

| | Problem | Status |
|---|---|---|
| **A** | The rover misbehaves | Mostly fixed — see `ARCHITECTURE.md` |
| **B** | You cannot *see* what it believes, so you cannot tell *why* | **This is what `learn/` fixes** |

Problem B is the real one. A robot you cannot observe is a robot you cannot
debug, and every hour spent on A while B is unsolved is guesswork.

## What we are building

A rover you can **watch, understand, and trust**, delivered as nine steps that
each end in something visible on your laptop screen.

### Definition of done

You can do all of this, in one sitting, without help:

1. Bring the stack up and read its health in one command.
2. See the robot's own position update live in RViz as you push it.
3. Prove that position is metrically correct against a tape measure.
4. Drive the rover by phone and watch the walls of a real room appear.
5. Save that map, and have it still be there next session.
6. Click **2D Goal Pose** anywhere in RViz, and watch the rover plan a path
   around the furniture and drive to it.
7. **Explain out loud what each stage is doing** — that is the actual product.

### Explicitly out of scope

These are real goals, but not *these* goals. They come after.

| Later | Why not now |
|---|---|
| "Go near the bottle" | Pulls in the Pi5 brain + Mac VLM — 3 machines, which is where the confusing failures live |
| Voice / "Hey Chotu" | The Orin has no CPU budget for STT/TTS; needs a home first |
| Remembering places by name | Needs map persistence (issue 05) working first |
| Pan/tilt camera head | The ESP32 firmware has no servo driver yet |
| Following a person | Needs everything above |

## Success criteria per issue

Each issue has a **gate** — a specific, measurable thing that must be true.
Gates are numbers, not opinions. Examples of what a gate looks like:

- `cuVSLAM odom ≥ 10 Hz` — a rate, read from `./run_stack.sh status`
- `push 1.00 m → reads 0.95–1.05 m` — a measurement against a tape
- `trust = True` — the robot's own honest assessment of its pose

If a gate fails, that is the *system working correctly* — it caught something.
Fix it, then re-run the gate. Do not lower the gate to make it pass.

## Constraints that shape everything

| Constraint | Consequence |
|---|---|
| **Jetson Orin Nano 8 GB, 6 cores** | The stack already runs at load ~2.5 idle and ~9 with everything on. CPU is a safety property, not a performance concern — starve the camera and cuVSLAM freezes silently. |
| **Camera sees forward only, ~87°** | No head servo. The rover is blind to its sides and behind. Maps will look "wrong" for this reason, and it is hardware, not a bug. |
| **D555 is ethernet/PoE, not USB** | It answers ping even when its DDS server is dead. Ping is not a health check. Only a physical power-cycle recovers it. |
| **Tethered rover, indoor, flat floor** | Speeds are capped low (0.30 m/s). 2D assumptions are safe (`two_d_mode` in the EKF). |
| **You are learning robotics** | Every issue explains the concept before the commands. Slower on purpose. |

## Where this is heading

The long-term product is a household robot that can see, understand, and move in
sync with its environment — the full picture is in
[`../orin-nav-stack/SYSTEM_INTEGRATION.md`](../orin-nav-stack/SYSTEM_INTEGRATION.md),
which describes all four machines (Jetson, Pi 5, Mac mini, ESP32) and the gaps
between here and there.

`learn/` is the foundation that makes the rest debuggable.
