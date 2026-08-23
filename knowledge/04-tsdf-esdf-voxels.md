# Voxels, TSDF, ESDF and the 2D slice

**Used by:** Phase 2 (mapping) and Phase 3 (the costmap layer).
**Code:** `phase2/launch/nvblox.launch.py`

> **Measured on this rig (Phase 2):** two settings decide whether the map is any
> use, and neither default suits a static room.
>
> **Decay.** nvblox multiplies every voxel's weight by 0.95 at 5 Hz and
> deallocates blocks that fall below threshold — a **2.7 second half-life**. A
> full room loop ended with less map than it started with. Set
> `decay_tsdf_rate_hz: 0.0` for surveying; leave it on where things move.
>
> **The ESDF slice is the ROBOT's height, not the camera's range.** The 2D map is
> a horizontal slice through the TSDF, and anything inside the band is an
> obstacle. Ours runs 0.10–0.24 m: above the floor (which the camera's 1.3° down
> pitch would otherwise read as an obstacle at range) and up to the rover's own
> 22 cm height. A 29 cm table is then correctly ignored while its legs are not.
>
> See [PHASE2.md](../PHASE2.md).


---

## The one thing to remember

> **nvblox does not see a room.** It is handed a depth image and a pose, and it
> writes the depth into a grid *at wherever the pose says the robot is*. It
> trusts the pose completely and has no way to check it.

Every mapping problem on this rig has turned out to be upstream of nvblox.

---

## Voxels

A **voxel** is a 3D pixel — a small cube of space. Ours are **5 cm**
(`voxel_size: 0.05`).

Voxel size is the central trade-off in mapping: halving it gives 8× the memory
and compute, for finer detail. At 5 cm, a chair leg is one or two voxels — enough
to avoid, not enough to describe.

## TSDF — Truncated Signed Distance Field

Each voxel does **not** store "full or empty". It stores **the signed distance to
the nearest surface**:

- **negative** — behind the surface (inside the wall)
- **zero** — exactly on the surface
- **positive** — in front, in free space
- **truncated** — beyond a few voxels of the surface we stop caring

Why bother, instead of just marking cells full/empty? Because it **averages
cleanly**. Ten noisy depth readings of one wall produce ten slightly different
distances, and averaging those converges on the true surface. Binary occupancy
cannot average — it flickers.

Each voxel also carries a **weight**: how much evidence has accumulated. That
weight is what decay attacks (below).

## ESDF — Euclidean Signed Distance Field

A second grid, holding **distance to the nearest obstacle** — not to the nearest
*surface we happened to see*, but to the nearest obstacle in any direction.

This exists purely for the planner. Asking "how close is this path to hitting
something?" against raw geometry is expensive; against an ESDF it is one lookup.
`esdf_mode: "2d"` in our config.

## The 2D slice

nav2's costmaps are 2D. So nvblox takes a horizontal band of the 3D world and
flattens it:

```yaml
esdf_slice_min_height: 0.10   # bottom of the band
esdf_slice_max_height: 0.22   # top — the rover's own height
esdf_slice_height:     0.16   # the representative height
```

Heights are **above the floor**, because `base_link`'s origin is on the ground.

The band is a judgement call, and ours is set to **the rover's own height**.
That is the honest definition of an obstacle for a ground vehicle: something
occupying the space the rover would occupy. Anything above 22 cm it drives
*under*, and the 29 cm table in this room is the case that forced the issue —
with a taller band the tabletop was integrated and the whole area came back as
solid obstacle, so the rover could not plan under a table it fits beneath.

### The slice height sets your useful depth range

This is the non-obvious consequence, and it cost us a day's confusion.

The band is only **12 cm tall**. Stereo depth error grows with the square of
range: at 5 m this camera's error is ~11.8 cm — *the entire height of the band*.
So a distant wall has its points smeared vertically right out of the slice.
Most miss it, the wall is never marked solid, and the ray therefore passes
**through** it and writes free space beyond.

| range | depth error | vs a 12 cm band |
|---|---|---|
| 2 m | 1.9 cm | well inside |
| 3 m | 4.2 cm | about a third — usable |
| 5 m | 11.8 cm | the whole band |

Observed 2026-08-22 with the limit at 5 m: an 18 × 17 m blob, walls scattered
through the middle and none at the edges, 150 m² of "free floor" for one room.
Both symptoms, one cause. `max_integration_distance_m` is now 3.0 m.

**The rule: the useful depth range is set by the SLICE HEIGHT, not by the voxel
size and not by the camera's spec range.** Want to see further? You need a
taller band — and a taller band re-integrates the tabletop. That is a genuine
trade, not a setting to tune freely. Change `esdf_slice_min/max_height` and you
must recheck the integration distance in the same breath.

**Slice heights are read at init.** `ros2 param set` reports success and changes
nothing — verified by getting a byte-identical map back. Edit the YAML and
restart.

---

## Decay: why the map used to delete itself

nvblox's defaults are tuned for dynamic scenes, where stale geometry (a person
who walked away) **should** be forgotten. Two mechanisms:

**TSDF decay.** Every voxel's weight is multiplied by `tsdf_decay_factor` at
`decay_tsdf_rate_hz`, and when it falls below a threshold the block is freed.
With the defaults:

```
ln(0.001 / 5) / ln(0.95) = 166 steps, at 5 Hz  =  ~33 seconds
```

**Clearing radius.** `map_clearing_radius_m: 7.0` deletes everything beyond 7 m
of the robot, once a second.

With an **87° forward-only camera**, those two guarantee a torch beam and never a
room: anything you look away from is gone in half a minute.

Both are neutralised in our config. Note `tsdf_decay_factor` must be **strictly
< 1.0** — `tsdf_decay_integrator.cu:27` asserts it and hard-aborts the node on
exactly 1.0. `0.999999` takes ~20 days, i.e. never.

### The cost, which is real

The map now **never forgets**. A person walking through leaves a permanent ghost,
pose drift smears walls instead of letting them fade, and memory grows without
bound.

The obvious fix — a gentler decay like 0.9999 — **does not work here**, and it is
worth understanding why. Decay removes what is not re-observed. With a
forward-only camera, the room *behind you* is never re-observed. So any decay
fast enough to clear noise is also fast enough to delete the room. The real lever
is integration weighting and depth outlier rejection, not decay.

---

## How to judge a map honestly

Eyeballing does not work — a good map and a bad one look similar at a glance.
Count cells:

| | free | occupied | occ/free |
|---|---|---|---|
| parked, healthy | 1112 | 520 | **0.46** |
| after driving, bad | 1721 | 2850 | **1.66** |

And do the arithmetic once, because it makes "too thick" concrete:

> A wall 4 m away across an 87° cone is an arc ~6.1 m long. One 5 cm cell thick,
> that is **~122 occupied cells**. The bad map had **2850** — walls about a metre
> thick.

Free should comfortably outnumber occupied. If occupied wins, something upstream
is wrong.

---

## The forward cone is not a bug

87°, forwards only, no working pan/tilt head.

> A wedge of map in front of the rover **is the correct picture** from one spot.

Most "the map is wrong" confusion is this. Prove it deliberately: put a chair
beside the rover and watch it never appear.

---

**See also:** [`01-frames-and-tf.md`](01-frames-and-tf.md) (why the slice band is
height-above-floor), [`06-costmaps-and-inflation.md`](06-costmaps-and-inflation.md)
(what nav2 does with the slice).
