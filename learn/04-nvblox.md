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

### What was tested on 2026-08-11, and what it ruled out

Three candidates. Two are now **eliminated by measurement**, which is the useful
part — it leaves exactly one place to look.

**❌ 1. The floor entering the slice band.** *Ruled out.* Same stationary
viewpoint, two slice bands, freshly restarted each time:

```
band 0.12-0.40 : free 1112, occupied 520, occ/free 0.47, covers 5.0 x 6.9 m
band 0.35-0.60 : free  653, occupied 315, occ/free 0.48, covers 2.5 x 4.4 m
```

The thickness **ratio is unchanged**. Lifting the band 23 cm clear of the floor
changed nothing except how much of the room is visible at all. If the floor were
filling the map, that ratio would have collapsed.

> Note: `ros2 param set` does **not** change the slice heights — nvblox reads
> them at init. Verified by getting a byte-identical map back afterwards. You
> must edit `config/nvblox.yaml` and restart with `./run_stack.sh fuse`.

**❌ 2. Noise accumulating because decay is disabled.** *Real, but far too small.*
Parked for 5.5 minutes with nothing in the room moving:

```
 t(s)     free  occupied  occ/free
    1     1122       520      0.46
  332     1116       549      0.49      +29 occupied cells, free flat
```

About **5 cells per minute**. Getting from 520 to the 2850 of the bad map would
take over seven hours at that rate. It is a slow leak worth knowing about, not
the cause.

**❌ 2b. Pose wandering while parked.** *Ruled out completely.* The rover is not
moving, so every reported change is error:

```
 t(s)         x         y         z   yaw_deg   drift_m
     0    0.0000    0.0000    0.0000     0.025    0.0000
   160    0.0000    0.0000    0.0000     0.014    0.0000
```

Zero translation drift over 160 s; yaw wobbles ±0.03°. The fused pose is rock
solid **standing still**.

**✅ 3. Pose error while DRIVING — the one candidate left.** Everything above was
measured from a stationary rover, and a stationary rover produces a *healthy*
map: `occ/free ≈ 0.46`, stable, thin edges. The 1.66 blob only ever appeared
after the room had been driven.

That points at the pose being wrong **while moving**, which smears each wall
across several cells as the rover travels. There is already strong prior
evidence for exactly this failure on this rig:

- cuVSLAM **under-reads translation ~4x** on low-texture floors with the IR
  emitter on (which is why the emitter is off — `run_stack.sh`).
- cuVSLAM can **freeze silently** if the camera is starved, and keep reporting
  `slam_pose_ok: true` while doing it.

**This is why the learn plan puts cuVSLAM (01) and odometry (02) BEFORE nvblox
(04) and mapping (05).** The map cannot be better than the pose it is built on.
Do not tune nvblox to compensate for a pose error — verify the pose first.

**The test to run next (it belongs to issue 02):** put the rover on the floor,
mark the start, push it a **measured 2.00 m** in a straight line, and compare
against `/odometry/filtered`. If it reports 1.5 m or 2.6 m, that is your blob.

> ⚠️ Restart with `./run_stack.sh fuse`, never by killing `nvblox_node` alone —
> it comes back with no depth input and logs "Last view not set for sensor type".
> And never attach ad-hoc subscribers to the raw camera topics while
> investigating: that took the D555 offline twice in one day.

## Likely work

Mostly looking, not changing: enable the nvblox mesh in RViz, walk objects in
front of the camera and watch them appear, and deliberately demonstrate the
forward-cone blindness so it stops being surprising later.

## Gate

*To be written when we get here.* Roughly: the mesh appears in RViz, you can
place an object and watch it show up, and you can state what the slice band means
and why the rover cannot see the chair beside it.
