# Task 04 — mapping: walls must be lines, not blobs

**Needs:** task 03 passed (z is exactly 0.000).
**Moves the robot:** no — you push it around the room.
**Command:** `./rover.sh l4`

---

## Learn first

Read [`04-tsdf-esdf-voxels.md`](../knowledge/04-tsdf-esdf-voxels.md) — what a voxel, a TSDF and an ESDF are, why the 3D map gets flattened into a 2D slice, and how to judge a map by counting cells instead of squinting at it.

Not to memorise. Just so the words in this task mean something.

## What you are proving

That driving a room produces a **room** — thin wall lines with open floor
between them — and not a solid blob.

**This is the task that has failed before**, and the whole repo is arranged so
that when it fails you know it is not nvblox's fault.

## What to understand first

nvblox does **not see a room**. It is handed a depth image and a pose, and it
writes the depth into a 3D grid *at wherever the pose says the robot is*. It
trusts the pose completely.

So if the pose under-reads travel, the wall you saw at the start is written in
one place, and the same wall after you push is written somewhere else. Drive a
room like that and every wall becomes a smear. **That is why task 02 comes
first.**

Three stages inside nvblox:

1. **TSDF** — each depth pixel becomes a 3D point using the pose, stored in 5 cm
   voxels holding "distance to the nearest surface".
2. **ESDF** — a second grid holding *distance to the nearest obstacle*, which a
   planner can query far more cheaply than raw geometry.
3. **2D slice** — nav2 cannot use 3D, so the band from 0.12 m to 0.40 m is
   flattened into a 2D occupancy grid.

### The forward-cone problem, which is not a bug

The camera sees **~87°, forwards only**. There is no pan/tilt head. So:

> A wedge of map in front of the rover **is the correct picture** from one spot.
> A room only appears if you drive around it.

Most "the map is wrong" confusion is this. Prove it to yourself deliberately:
put a chair beside the rover and watch it never appear.

### The map used to delete itself

nvblox's defaults were erasing the map after ~33 s (TSDF decay) and beyond 7 m
(clearing radius). Both are disabled in `config/nvblox.yaml`, with the arithmetic
written out there.

The cost, which you should watch for: **the map now never forgets.** A person
walking through leaves a permanent ghost.

### This task owns map quality — and the first knob is NOT decay

`TODO.md §5` is assigned here. The tempting fix for a thick map is to re-enable a
slow TSDF decay, and it does not work on this rig: decay removes whatever is not
re-observed, and with a forward-only camera the room *behind you* is never
re-observed. Any decay fast enough to clear noise is fast enough to delete the
room. That is measured, not theorised (`FACTS.md §4`).

**The first knob to reach for instead is
`projective_integrator_max_integration_distance_m`, set to about 3 m.** Stereo
depth error grows with range², so the far field is where a thin wall becomes a
thick one — and the far field is also the part you were going to have to drive
closer to anyway. One parameter, an effect you can measure directly in occ/free,
and nothing else in the stack changes.

⚠️ **Not before tasks 02 and 03 pass.** `TODO.md §1` says it in bold: do not tune
nvblox before the pose is proven. Tuning the map to compensate for a bad pose
bakes the pose error into the config, where it will quietly survive the pose being
fixed later. And it may not be needed at all — if an honest pose alone takes
driven occ/free from 1.66 to under 0.3, every hour spent on nvblox was wasted.

So the order inside this task is: drive the room, **measure occ/free first**, and
only reach for a parameter if the number says you have to.

## Do this

```bash
./rover.sh l4
./rover.sh view start
```

Then **push the rover slowly around the room by hand**. Slowly matters —
cuVSLAM tracks features between frames, and fast motion is when it loses them.

Watch two things as you go:

- the map **behind** you: does it stay, or vanish?
- the walls: are they **lines**, or thickening blobs?

## Gate

```
[ ] /nvblox_node/static_occupancy_grid >= 5 Hz
[ ] EXACTLY ONE nvblox node is running   (rover.sh l4 checks this)
[ ] map behind the rover PERSISTS as you drive away
[ ] walls come out as LINES roughly one or two cells thick
[ ] occupied/free ratio < 0.3 after driving a room
[ ] you can explain why the chair beside the rover is not on the map
```

### How to check the ratio honestly

Eyeballing a map is unreliable — the 1.66 blob and the 0.46 clean map look
similar at a glance. Reference numbers from 2026-08-11:

| | free | occupied | occ/free |
|---|---|---|---|
| parked, healthy | 1112 | 520 | **0.46** |
| after driving, bad | 1721 | 2850 | **1.66** ← the failure |

Arithmetic worth doing once: a wall 4 m away across an 87° cone is an arc ~6.1 m
long. One 5 cm cell thick, that is ~**122** occupied cells. The bad map had
**2850** — walls about a metre thick.

## If it fails

| Symptom | Likely cause |
|---|---|
| walls are thick blobs after driving | **the pose, not nvblox.** Go back and re-run `./rover.sh measure 2.00`. See `TODO.md §1`. |
| map vanishes behind you | decay is back on — check `config/nvblox.yaml` |
| two nvblox nodes | the L4 gate catches this; just run `./rover.sh l4` again |
| map is a wedge and never a room | correct behaviour — you have not driven far enough |
| floor appears as obstacle | re-run `./rover.sh floor`; it must read ~0.000 |

## Commit

```bash
git add -A && git commit -m "task 04: drove the room, walls are lines, occ/free 0.22"
```

**Next:** [`05-persistence.md`](05-persistence.md)

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
