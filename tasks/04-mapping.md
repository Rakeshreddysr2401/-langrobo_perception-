# Task 04 — mapping: walls must be lines, not blobs

**Needs:** task 03 passed (z is exactly 0.000).
**Moves the robot:** no — you push it around the room.
**Command:** `./rover.sh l4`

---

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
