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

## Likely work

Mostly looking, not changing: enable the nvblox mesh in RViz, walk objects in
front of the camera and watch them appear, and deliberately demonstrate the
forward-cone blindness so it stops being surprising later.

## Gate

*To be written when we get here.* Roughly: the mesh appears in RViz, you can
place an object and watch it show up, and you can state what the slice band means
and why the rover cannot see the chair beside it.
