# Task 02 — cuVSLAM: where am I?

**Needs:** task 01 passed.
**Moves the robot:** no — you push it by hand.
**Command:** `./rover.sh l2`

---

## Learn first

Read [`02-visual-odometry.md`](../knowledge/02-visual-odometry.md) then [`01-frames-and-tf.md`](../knowledge/01-frames-and-tf.md) — how stereo VO turns two IR images into motion, why rotation is its weak case, and who publishes which transform.

Not to memorise. Just so the words in this task mean something.

## What you are proving

That the rover's sense of distance is **metrically honest**. Not that cuVSLAM is
running — that it is *right*.

**This is the most important gate in the whole repo.** Everything above it —
the map, the costmap, the planner — is built on this number, and a wrong pose
produces confident nonsense rather than an error.

## What to understand first

Stereo visual odometry, in one paragraph: find distinctive features in the left
IR image, find the same features in the right image, and the horizontal offset
between them gives depth by triangulation. Track those features across
successive frames; how they move tells you how the camera moved. Keep some
frames as "keyframes" so that when you return to a place you recognise it and
can correct accumulated drift — that is **loop closure**.

Now the trap that makes this task exist. **cuVSLAM has been wrong here by 4×
without saying so.**

| IR emitter | real push | `/odom` said | error |
|---|---|---|---|
| ON | 100 cm | **26 cm** | ~4× under |
| OFF | 100 cm | **97 cm** | 3% |

The emitter is bolted to the camera, so its dot pattern is repainted from the
camera's own viewpoint every frame. On a low-texture floor the dots barely move
in the image even as the rig translates — so the tracker sees almost no motion.

It reported `slam_pose_ok: true` throughout. **A tape measure is the only honest
referee.**

## Do this

```bash
./rover.sh l2
```

This starts the static TF first (the pose must land in a complete frame tree),
then cuVSLAM, then the rover body and trail markers so you can see it in RViz.

Then the measurement that actually matters:

```bash
./rover.sh measure 2.00
```

1. Put the rover on the floor and **mark where the wheels are**.
2. Start the command.
3. Push it **2.00 m in a straight line**, by hand.
4. Ctrl-C.

> ✔ Hand-pushing is safe. The firmware's PID returns zero below 0.01 target
> speed, and zero duty pulls both motor pins low — that is *coast*, not brake.
> The wheels free-wheel and the motors will not fight you.

## Gate

```
[ ] /odom publishes at >= 10 Hz
[ ] a 2.00 m tape-measured push reads 2.00 m +- 5%   (1.90 - 2.10)
[ ] path length is not much larger than straight-line distance
[ ] TF map -> odom -> base_link all present in RViz
[ ] pushing the rover moves base_link across the grid
```

## Reading the result

| Result | Meaning |
|---|---|
| within 5% | good — this is the gate |
| reads **short** | the classic low-texture under-read. Check the emitter really is off, and try a floor with more visual texture. |
| reads **long** | scale over-estimate — same class of problem |
| path ≫ straight | it is jittering in place rather than tracking |
| 0.000 throughout | cuVSLAM has **frozen**. It will still claim `slam_pose_ok: true`. Restart the layer. |

**If this gate fails, stop.** Do not go to mapping. A wrong number here is the
leading explanation for the smeared map (`TODO.md §1`), and no amount of nvblox
tuning fixes it.

## Commit

```bash
git add -A && git commit -m "task 02: cuVSLAM reads 1.97 m on a 2.00 m push (1.5% under)"
```

Put the real number in the message.

**Next:** [`03-fusion.md`](03-fusion.md)

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
