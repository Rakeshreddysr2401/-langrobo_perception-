# 02 — Odometry: is my position *correct*?

**Needs:** issue 01 passed (cuVSLAM running, trail clean).
**Motors move:** no. You push the rover by hand, with a tape measure.
**Time:** ~45 minutes. Needs about 2 m of clear floor.

---

## What this is (read first)

Issue 01 proved cuVSLAM produces a *smooth, plausible* position. This issue
proves it produces a **true** one.

Those are completely different things, and the gap between them is where this
robot has hurt you most. With the IR emitter on, cuVSLAM produced a beautifully
smooth trail that was **wrong by a factor of four** — 100 cm read as 26 cm.
Every rate was healthy. Nothing logged an error. The rover simply drove to the
wrong place, confidently.

> **A robot that is confidently wrong is more dangerous than one that is
> obviously broken.** This issue is where you install the habit of checking.

### The only real instrument is a tape measure

Every sensor on this robot can be fooled. A tape measure cannot. So the test is
deliberately, almost insultingly simple:

```
     start                                    end
       │◄───────── tape: exactly 1.00 m ─────►│
       │                                      │
     [rover] ──── push slowly by hand ───► [rover]

     Then ask: what does /odom claim?
```

If it says 0.95–1.05, cuVSLAM is metrically honest. If it says 0.26, you have
just personally reproduced the emitter bug.

### What "scale error" means, and why it's sneaky

A scale error means every distance is multiplied by some constant. The robot
still turns correctly, still tracks smoothly, still looks perfect in RViz — but
every distance is proportionally wrong.

Downstream, this means: the map comes out the wrong size, "drive 2 m forward"
goes somewhere else, and object approach stops in the wrong place. It corrupts
everything after it, which is why this check comes before any mapping.

### The tool

`nodes/odom_ruler.py` exists for exactly this. It zeroes at the start, shows a
live readout as you push, and on Ctrl-C grades the result:

| Result | Meaning |
|---|---|
| within 5% | pass |
| within 15% | marginal — investigate before continuing |
| worse | fail — do not proceed to mapping |

It reports **two** numbers, and the difference matters:

- **`straight`** — distance from where you started, as the crow flies. This is
  what a tape measure gives you.
- **`path`** — total distance travelled, including any wandering.

On a straight push these should be close. If `path` is much larger than
`straight`, the pose is wandering sideways even though you pushed in a line.

---

## Do this

### 1. Set up

Clear about 2 m of floor. Put a tape measure down. Mark the start and a point
**exactly 1.00 m** away. Line up the same part of the rover with each mark — the
front edge or a wheel axle, but be consistent, since a few cm of inconsistency is
a few percent of your answer.

### 2. Check the stack is healthy first

```bash
cd ~/langrobo_perception/orin-nav-stack
./run_stack.sh status
```

`trust=True` and cuVSLAM ≥ 10 Hz, or the measurement is meaningless.

### 3. Start the ruler

```bash
docker exec -it orin_nav bash -lc '
  source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0
  python3 /opt/orin-nav/nodes/odom_ruler.py --expect 1.00'
```

It zeroes and starts a live readout.

### 4. Push

Push the rover slowly and straight, from the start mark to the 1.00 m mark. Take
your time — about 5 seconds. Stop cleanly.

> ✔ Hand-pushing is safe (the PID coasts at zero — see issue 00).
> ⚠️ Push **slowly**. Fast motion breaks tracking and you'll measure the wrong
> failure.

Press **Ctrl-C**. It prints the comparison.

### 5. Repeat it three times

Once is an anecdote. Do it three times and write down all three. If they disagree
with each other by more than a few percent, the problem is repeatability, not
scale — and that's a different fix.

### 6. Independently check the camera height

The camera height is currently **0.163 m** in TF. That number came from
`floor_probe.py`, which infers it from where the floor deprojects — a clever
instrument, but still the camera checking its own work.

Take an actual tape measure from the floor to the camera lens centre.

```bash
docker exec -it orin_nav bash -lc '
  source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0
  python3 /opt/orin-nav/nodes/floor_probe.py'
```

Open floor should read **z ≈ 0.000** (that's the check — `base_link`'s origin is
on the ground, so the floor must be at zero). If your tape and the probe disagree
by more than about 1 cm, **the tape wins** and the discrepancy needs explaining
before you map anything.

Why this matters so much: when this was wrong by 4 cm, the floor deprojected to
+0.037 m, nvblox mapped the floor itself as an obstacle, and the rover crawled
because it thought it was surrounded.

---

## See it in RViz

While pushing, watch the `/odom` trail. You are looking for:

- A **straight line**, not a curve. A consistent curve on a straight push means a
  yaw bias.
- Length that **looks** like 1 m against the RViz grid (default squares are 1 m).
- No jumps or teleports mid-push.

---

## GATE — all four

```
  [ ] 1.00 m tape push reads 0.95 - 1.05 m on /odom  (3 runs, all pass)
  [ ] /odom/health reports trust: true throughout
  [ ] path and straight are within ~10% of each other on a straight push
  [ ] floor_probe reads z ~ 0.000, and a real tape agrees with 0.163 m within 1 cm
```

Write the three numbers into `PROGRESS.md`. You will want them later — if
mapping looks wrong in issue 05, the first question is whether scale was ever
right, and "it passed" won't answer it.

---

## If it fails

| Reading | Meaning | Fix |
|---|---|---|
| ~0.25 m for a 1 m push | The emitter-scale bug | Confirm the IR emitter is OFF. Check `run_stack.sh` didn't get reverted. |
| 0.85–0.95 m | Mild under-read | Often pushing too fast, or a low-texture floor. Retry slower, aim the camera at a textured wall. |
| Wildly different each run | Tracking is unstable, not mis-scaled | Check `status` for camera rate dips and `orin load1`. Add texture to the scene. |
| Curves on a straight push | Yaw bias | Note it and continue — issue 03 fuses the gyro, which is what fixes heading. Don't chase it here. |
| `floor_probe` z far from 0 | Camera height TF is wrong again | Measure with a tape, update the TF in `run_stack.sh`, restart, re-probe. |

---

## What you have now

A pose you can **trust metrically**, not just one that looks smooth. Everything
after this — the map, the costmap, the path, the goal — is built on top of this
number. That is why it comes before all of them.

## Commit

```bash
git add learn/PROGRESS.md
git commit -m "issue 02: odometry verified against tape — 1.00 m read 0.98/0.97/0.99, floor z=0.000"
```

**Next:** [`03-imu.md`](03-imu.md) — surviving the moments cuVSLAM can't see.
