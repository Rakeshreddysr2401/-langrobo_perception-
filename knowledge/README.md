# knowledge — the concepts, one place each

Tasks are **what to do**. This is **what it means**.

Concepts live here rather than inside task files because they span tasks — TF
matters in 02, 03, 04, 06, 07 and 08; QoS bites in every one. Written once, they
get deeper over time instead of drifting apart in six copies.

Each file explains the idea from first principles and then grounds it in **this**
codebase — real file names, real config values, real measured numbers.

| # | Topic | Used by |
|---|---|---|
| 01 | [Frames and TF](01-frames-and-tf.md) | 02, 03, 04, 06, 07, 08 |
| 02 | [Visual odometry and SLAM](02-visual-odometry.md) | 02, 03, 04, 08 |
| 03 | [Sensor fusion and the EKF](03-sensor-fusion-ekf.md) | 03 |
| 04 | [Voxels, TSDF, ESDF and the 2D slice](04-tsdf-esdf-voxels.md) | 04, 05, 06, 08 |
| 05 | [QoS, DDS and why we measure rates](05-qos-dds-and-rates.md) | all |
| 06 | [Costmaps and inflation](06-costmaps-and-inflation.md) | 06, 07, 08 |
| 07 | [Planners, controllers and behaviour trees](07-planners-and-controllers.md) | 07, 08 |
| 08 | [Frontier exploration](08-frontier-exploration.md) | 08 |

---

## How to use these

**Before a task** — read the topics its "Learn first" section lists. Not to
memorise; just so the words in the task file mean something.

**During a task** — when something surprises you, that surprise is the valuable
part. Write it in the task's **"What I learned doing it"** section while you
still remember being confused.

**After a task** — if you learned something about the *concept* rather than about
that day's work, promote it up into the topic file here. That is how these get
better.

---

## Where different things belong

| Kind of thing | Goes in |
|---|---|
| how something works, in general | `knowledge/` (here) |
| a number you measured | `FACTS.md` |
| something broken or unverified | `TODO.md` |
| what happened while doing a task | that task's "What I learned doing it" |

Keeping these separate is what stops the repo turning back into fourteen
overlapping documents that disagree with each other.

---

## If you only read one

[`01-frames-and-tf.md`](01-frames-and-tf.md). Nearly every confusing thing a
robot does is a frame problem, and the 3.7 cm error described in it broke this
robot in six different ways at once.
