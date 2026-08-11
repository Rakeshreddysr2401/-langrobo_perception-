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

### Then the second measurement: out and back

The one-way push above measures the **scale factor** — does one metre of world
equal one metre of pose. That is the number the emitter fix was about, and it is
necessary but not sufficient.

```bash
./rover.sh measure        # no --expect this time
```

1. Start from the same mark.
2. Push **2.00 m out, then 2.00 m back to the mark**, in one continuous run.
3. Ctrl-C.

Now read a different line of the output. `straight` is distance *from where you
started* — and you finished where you started, so the honest answer is **zero**.
Whatever it reads instead is accumulated error, in metres, over 4 m of travel.
`path` should read ~4.00 m, confirming you really did travel out and back.

**Why this is the number that matters for the map.** nvblox writes depth wherever
the pose says the robot is. A perfect scale factor with 15 cm of accumulated
error still smears a wall by 15 cm — three 5 cm voxels — and it is smear, not
scale, that produced the 1.66 blob in `TODO.md §1`. A one-way push cannot see
this at all, because there is nothing to compare the endpoint against.

## Gate

```
[ ] /odom publishes at >= 10 Hz
[ ] a 2.00 m tape-measured push reads 2.00 m +- 5%   (1.90 - 2.10)
[ ] path length is not much larger than straight-line distance
[ ] OUT AND BACK: after 2.00 m out + 2.00 m back, `straight` <= 0.10 m
    (and `path` reads ~4.00 m, i.e. you really did drive both legs)
[ ] TF map -> odom -> base_link all present in RViz
[ ] pushing the rover moves base_link across the grid
```

The 0.10 m bound is 2 voxels of smear at 5 cm resolution. It is a starting
threshold, not a measured one — if the rig comes in far under it, tighten it and
say so in `FACTS.md`. If it comes in over, that is the smear cause found, and
`TODO.md §1` closes here rather than in task 04.

## Reading the result

| Result | Meaning |
|---|---|
| within 5% | good — this is the gate |
| reads **short** | the classic low-texture under-read. Check the emitter really is off, and try a floor with more visual texture. |
| reads **long** | scale over-estimate — same class of problem |
| path ≫ straight | it is jittering in place rather than tracking |
| 0.000 throughout | cuVSLAM has **frozen**. It will still claim `slam_pose_ok: true`. Restart the layer. |
| one-way passes, out-and-back fails | **the interesting case.** Scale is right; error accumulates as you move. This is the leading explanation for `TODO.md §1` and it is what would have been missed by measuring one way only. Note whether the residual points consistently in one direction (a bias) or wanders (noise). |

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
