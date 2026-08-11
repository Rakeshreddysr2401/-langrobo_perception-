# Task 01 — the camera, alone

**Needs:** nothing. This is the start.
**Moves the robot:** no.
**Command:** `./rover.sh l1`

---

## Learn first

Read [`05-qos-dds-and-rates.md`](../knowledge/05-qos-dds-and-rates.md) — why a topic can exist and still deliver nothing, why the D555 is a DDS device, and why we measure rates instead of publisher counts.

Not to memorise. Just so the words in this task mean something.

## What you are proving

That the camera streams, on its own, **stably**, with nothing else running.

This is its own task because on this rig the camera is the fragile part, and
every layer above it dies in a confusing way when it sags. If you cannot hold
15 Hz for five minutes with nothing else competing, nothing above this is worth
debugging.

## What to understand first

**The D555 is a network device, not a webcam.** It speaks DDS over ethernet at
`192.168.11.55`. That single fact explains its whole failure mode:

- It **answers ping while completely dead.** Ping is never a health check.
- It can go `device is offline` at the DDS level, and then **no software restart
  recovers it** — only unplugging the PoE cable.
- Its topics are **lazily published**: nothing streams until something
  subscribes.

That last point is why there is a hard rule: **never attach an ad-hoc subscriber
to a raw camera topic.** Doing so has taken the camera offline twice. To look at
imagery, use a long-lived subscriber, never `ros2 topic echo` on an image.

## Do this

```bash
cd ~/rover
./rover.sh l1
```

It launches only the camera, then waits up to 40 s for `infra1` to have a live
publisher — a real check, not a blind sleep. If the camera never streams it
aborts and tells you to power-cycle rather than starting SLAM blind.

## Gate

```
[ ] camera IR left  >= 15 Hz
[ ] camera depth    >= 10 Hz
[ ] both still true after FIVE MINUTES  (re-run ./rover.sh status)
[ ] you can say why ping does not prove the camera is alive
```

The five-minute part is the real test. A camera that starts at 26 Hz and sags to
9 Hz is the failure that took a whole day on 2026-08-11, and it looks perfectly
healthy for the first thirty seconds.

## If it fails

| Symptom | Meaning | Fix |
|---|---|---|
| aborts, log says "No RealSense devices were found" | the on-camera DDS server is dead | **Physically power-cycle** — unplug PoE ~5 s, replug, wait ~15 s |
| starts fine then sags toward 10 Hz | the camera is dying, or something is competing | check nothing else is running: `docker ps`. Then power-cycle. |
| `ros2 topic hz` on depth says "not published yet" | NITROS type negotiation | **not a fault** — ignore it, trust `./rover.sh status` |

## Commit

```bash
git add -A && git commit -m "task 01: camera holds 26 Hz IR / 20 Hz depth for 5 min"
```

Record the **actual numbers** you saw, not "worked".

**Next:** [`02-cuvslam.md`](02-cuvslam.md)

---

## What I learned doing it

*Fill this in while you still remember being confused — that is the valuable
part, and it evaporates within a day.*

**What surprised me**

>

**What I got wrong first**

>

**Numbers I measured**

>

**Codebase things worth remembering** — a file, a parameter, a line that turned
out to matter more than it looked

>

**Still don't understand**

>

> Housekeeping when you finish: a measured number belongs in `FACTS.md`, a
> broken or unverified thing belongs in `TODO.md`, and anything you learned about
> the *concept* rather than about today should be promoted into
> [`knowledge/`](../knowledge/).
