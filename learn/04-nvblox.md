# 04 — nvblox: what does the world look like?

> **STUB.** Written in full when you get here.

**Needs:** issue 03 passed (pose survives dropouts).
**Motors move:** no — the rover stays put, you look at what it already sees.

---

## Goal

Understand the 3D map the robot builds from depth images, and see it in RViz —
without driving yet.

## What you will learn

- What a **voxel** is, and how depth images become a 3D model
- What an **ESDF** is — a grid storing *distance to the nearest obstacle*, which
  is much cheaper for a planner to query than raw geometry
- Why nav2 can't use 3D directly, so nvblox flattens a horizontal **slice** of it
  into a 2D grid
- Why the map is a **forward cone only (~87°)** — there is no pan/tilt head, so
  the rover is blind to its sides and behind. This is the single biggest reason
  the map "looks wrong", and it is hardware, not a bug.

## Known state going in

- nvblox is running and healthy — the ESDF slice publishes at ~9.2 Hz.
- Current slice band: `esdf_slice_min_height: 0.12`, `max: 0.40`
  (`config/nvblox.yaml:50-51`).
- **The 0.12 is a workaround, not a design choice.** It was raised from 0.05 to
  dodge a 4 cm camera-height error that made the floor deprojected as an
  obstacle. That error is now fixed (floor reads 0.000 ±0.004), so the band can
  come down — but that's issue 06, because it changes what nav2 calls an
  obstacle and must be validated while actually driving.
- `global_frame: "odom"` (`config/nvblox.yaml:9`) — which is why saved maps don't
  line up across sessions. That's issue 05.
- **Map accumulation was broken until 2026-08-11 and is now fixed.** nvblox's
  defaults deleted the map behind the rover — TSDF decay freed any block unseen
  for ~33 s, and `map_clearing_radius_m: 7.0` deleted everything beyond 7 m. The
  result was a torch beam that never became a room. Both are now disabled in
  `config/nvblox.yaml`, with the arithmetic written out there.
  - **Understand the trade-off in this issue:** the map now *never forgets*. A
    person who walks through leaves a permanent ghost, and pose drift smears
    walls instead of letting them fade. If ghosts become a problem, the fix is a
    slow decay (a factor like 0.9999), not the aggressive default.

## The open question this issue must answer (measured 2026-08-11)

The 2D occupancy map does **not** look like a room. Snapshot taken with the rover
standing still, looking forward:

```
grid      : 128 x 240 @ 0.05 m/cell
unknown   : 85.1 %
free      :  5.6 %
occupied  :  9.3 %      <-- MORE OCCUPIED THAN FREE. That is backwards.
known bbox: 5.3 x 9.3 m  (= exactly the 87 deg FOV cone at the 5 m
                            projective_integrator_max_integration_distance_m,
                            so the EXTENT is correct — the fill is not)
```

Rendered, it is a solid black wedge: a small white free triangle near the rover,
then everything from ~1.5 m to ~4.5 m painted as obstacle, then grey unknown.

Do the arithmetic, because it is the whole point. A far wall at 4 m across an
87° cone is an arc ~6.1 m long. One 5 cm cell thick, that is ~122 occupied
cells. **We measured 2850** — over 20x too many, i.e. the "wall" averages about
a metre thick. That is not a wall, it is a volume being filled in.

Candidate causes, none yet confirmed:

1. **The floor entering the slice band.** The band is 0.12–0.40 m in odom z. A
   small pitch error, or depth noise that grows with range, lifts far floor into
   the band — and the black region starting only at ~1.5 m fits that shape well.
2. **Depth noise accumulating.** Stereo depth error grows roughly with range²,
   and the map now **never forgets** (decay disabled, see below). A stationary
   rover re-integrates the same noisy surface indefinitely and it thickens.
3. **Unobserved space behind the surface being read as occupied** by the ESDF
   slice rather than left unknown.

**How to test (1):** raise `esdf_slice_min_height` to ~0.35 in
`config/nvblox.yaml` and restart. If the wedge collapses to thin walls, it is
the floor. Note that `ros2 param set` does **not** work for this — nvblox reads
the slice heights at init, verified 2026-08-11 (the map came back byte-identical).

> ⚠️ Restart with `./run_stack.sh fuse`, never by killing `nvblox_node` alone.
> And do **not** attach ad-hoc subscribers to the raw camera topics while
> investigating — that is what took the D555 offline mid-experiment.

## Likely work

Mostly looking, not changing: enable the nvblox mesh in RViz, walk objects in
front of the camera and watch them appear, and deliberately demonstrate the
forward-cone blindness so it stops being surprising later.

## Gate

*To be written when we get here.* Roughly: the mesh appears in RViz, you can
place an object and watch it show up, and you can state what the slice band means
and why the rover cannot see the chair beside it.
