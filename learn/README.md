# learn — build the rover one piece at a time, and understand each piece

This is the only folder you need to read.

**The goal of all nine issues:** you open RViz on your laptop, drive the rover
around a room by phone, watch walls appear, save the map, then click a goal
anywhere in RViz and the rover plans a path around the obstacles and drives
there — and you can point at any step and say what it is doing and why.

Not included: voice, Telegram, "go near the bottle", remembering places by name.
Those come after, and they are easy once this works.

---

## How to use this

Do **one issue at a time**, in number order. Do not skip. Do not do two at once.

```
    read the issue  →  run the commands  →  watch it in RViz
                              ↓
                       does it pass the GATE?
                    ↓                        ↓
                   yes                       no
                    ↓                        ↓
            commit, next issue        fix it, re-run the gate
```

**An issue is not done because the code exists. It is done when you watched it
work and it passed the gate.** This rule exists because every failure this robot
has had so far was *silent* — the code ran, nothing logged an error, and the
answer was simply wrong. On 2026-08-10 there were seven of those at once.

## The issues

| # | File | Question it answers | Motors move? |
|---|---|---|---|
| 00 | [`00-setup.md`](00-setup.md) | Can I see anything at all? | no |
| 01 | [`01-cuvslam.md`](01-cuvslam.md) | Where am I? | no |
| 02 | [`02-odometry.md`](02-odometry.md) | Is my position *correct*? | no (hand push) |
| 03 | [`03-imu.md`](03-imu.md) | Can I survive the camera blinking? | no (hand push) |
| 04 | [`04-nvblox.md`](04-nvblox.md) | What does the world look like? | no |
| 05 | [`05-map-on-the-fly.md`](05-map-on-the-fly.md) | Build a map as I drive, and keep it | **yes** (you drive) |
| 06 | [`06-obstacle-detection.md`](06-obstacle-detection.md) | What is in my way? | no |
| 07 | [`07-nav2.md`](07-nav2.md) | Drive to a goal by itself | **yes** (robot drives) |
| 08 | [`08-path-planning.md`](08-path-planning.md) | Why did it pick *that* path? | **yes** |

Issues 00–02 are written in full. **03–08 are stubs on purpose** — their gates
depend on numbers you have not measured yet, and a made-up number in a gate is
worse than no number, because it sends you off "fixing" something that was fine.
Each one gets written properly when you reach it.

## The other files

| File | What it is |
|---|---|
| [`PRD.md`](PRD.md) | What we are building and what counts as done |
| [`ARCHITECTURE.md`](ARCHITECTURE.md) | How the pieces connect + every measurement we have |
| [`PROGRESS.md`](PROGRESS.md) | Where you are right now. Update it as you go. |

## Standing rules

1. **Never advance on an unverified gate.**
2. **Check pose trust before every move.** `trust=false` ⇒ do not drive, and do
   not believe the robot when it says it arrived.
3. **CPU is a safety property.** If the Orin load goes above ~8 the camera drops
   frames, and cuVSLAM freezes *silently* and never recovers. Check `status`
   after adding anything.
4. **Save the map before `remap` or `fuse`.** Both wipe it.
5. **One change at a time**, then re-run the gate.
6. **When a doc disagrees with the machine, the machine is right.** Fix the doc
   immediately. A stale laptop IP in a README cost a whole session once.
