# Phase 2 — Mapping

**Goal: build the room as the rover drives, and see it.**

Phase 1 answered *where am I*. Phase 2 answers *what is around me*, and writes it
down. Everything here stands on the fused pose — nvblox writes depth wherever the
pose claims the robot is, so a bad pose does not produce a slightly wrong map, it
produces a permanently smeared one.

Divided into four parts. **2a and 2b are done**; 2c and 2d were deliberately
deferred.

| | what | status |
|---|---|---|
| **2a** | see it — RViz, the pose, the track it draws | ✅ |
| **2b** | build it — nvblox turns depth + pose into a map | ✅ |
| 2c | keep it — save and reload across a power cycle | deferred |
| 2d | localize in it — recognise a room you mapped before | deferred |

---

## 1. Why 2b and 2d are different

They sound alike and are opposites.

- **Mapping** assumes you *know where you are* and records what you see. It
  trusts `/odom`.
- **Localization** is handed a map and works out *where you are in it*.

That shows up in the frame chain:

```
map ──(localization, 2d)──► odom ──(Phase 1b)──► base_link
```

Phase 1 produced `odom → base_link`. **`odom` drifts and resets to zero on every
restart.** A map that survives needs `map → odom`, and producing that transform
is what localization does. Until then everything lives in `odom`, which is enough
to map a room and navigate it in one session — and not enough to recognise it
tomorrow.

**`global_frame` is `odom` everywhere in Phase 2 and 3.** Pointing anything at a
`map` frame nothing publishes gives an empty screen and a confusing hunt.

---

## 2a — Seeing it

### The track it draws

`fusion_node` publishes **`/fusion/path`** (`nav_msgs/Path`) — the line the rover
has driven.

Two decisions in a small feature, both learned by getting them wrong:

- **Appended in 2 cm chords, not per pulse.** At 20 Hz a stationary rover would
  otherwise add 20 identical poses a second until the message is megabytes and
  RViz stutters. Capped at 5000 poses, about 100 m.
- **Published on a timer and latched**, not only when it moves. Appending is
  about travel; publishing is about viewers. The first version conflated them, so
  RViz showed nothing once the rover stopped and nothing at all if it started
  after a drive — which reads as a broken publisher rather than a design choice.

### RViz, on a laptop

Full setup in [phase2/LAPTOP.md](phase2/LAPTOP.md). The rover changes nothing —
RViz only subscribes.

**Two settings fail silently** and are the first thing to check when a display is
blank:

- `ROS_DOMAIN_ID=0` and an **unset** `ROS_DISCOVERY_SERVER`. A laptop pointed at
  a stale discovery server sees an empty graph and looks exactly like a network
  fault.
- **QoS durability**, and the two sources here are opposite:

| topic | durability |
|---|---|
| `nvblox .../static_occupancy_grid` | **Volatile** |
| `/global_costmap/costmap` | **Transient Local** |
| `/fusion/path` | **Transient Local** |

RViz renders a QoS mismatch **exactly like a dead publisher** — empty screen, no
error.

The laptop has ROS 2 Jazzy but **not `nvblox_rviz_plugin`**, so the 3D mesh
cannot render there. The config uses only standard message types, and the 2D map
is what nav2 plans on anyway.

### Seeing it without RViz

`logs/map_view.py` draws the same occupancy grid as text, with the rover on it:

```
                    .............###
                  ............###
                ...........#####
 > ................#########
   ................#########
     ..######.....##########
```

`#` wall · `.` floor seen · blank unknown · `> < ^ v` the rover and its heading.

---

## 2b — Building it

### The technique

**nvblox**, on the GPU. Depth images plus a pose become a **TSDF** — a truncated
signed distance field, a grid of voxels each storing how far it is to the nearest
surface. From that comes a mesh to look at, and an **ESDF** slice at floor height
which is a 2D occupancy grid: the blueprint, and what nav2 plans on.

Confirmed running on this Orin before use, because `isaac_ros_visual_slam` in the
same image is a Thor build that will not run here. nvblox allocated GPU hash
tables and built TSDF/Color/Feature/Freespace/Occupancy/ESDF layers at 5 cm.

### The settings that matter, and why

| setting | value | why |
|---|---|---|
| `global_frame` | `odom` | no map frame exists yet — see §1 |
| `voxel_size` | 0.05 m | small enough for a chair leg, large enough that a room fits in GPU memory. Phase 1's 10 cm drift gate was set as two voxels for this reason |
| `mapping_type` | `static_tsdf` | the dynamic modes track moving objects and cost more; the room is not moving |
| `esdf_slice_min_height` | **0.10 m** | **the camera sits 16.3 cm up and pitches down 1.3°.** A slice at exactly 0 clips the floor itself and fills the map with phantom obstacles. That mount pitch came from Phase 1's gravity measurement and earns its keep here |
| `esdf_slice_max_height` | **0.24 m** | **the rover's own height, tape-measured** — see below |
| `max_integration_distance` | **3.0 m** | set by the SLICE HEIGHT, not the camera's range — see below |
| `decay_tsdf_rate_hz` | **0.0** | **the map forgets otherwise** — see below |
| `use_color` | false | colour is disabled on the camera: with `enable_sync:=false`, enabling it gates the IR pair behind colour alignment and starves the stereo cuVSLAM needs |

### The slice is the rover's height, not the camera's range

The 2D map is a **horizontal slice** through nvblox's 3D model, and anything
inside the band becomes an obstacle. The band must be set by **what the rover
collides with**:

| height | |
|---|---|
| camera | 18 cm |
| **rover** | **24 cm** — tape-measured 2026-08-23, the number that matters |
| gate bar / table | 29 cm — 5 cm of clearance above the rover |

| band | |
|---|---|
| 0 – 10 cm | **skipped.** The camera pitches down 1.3°, so at 3 m the floor itself reads up to 6.8 cm high and would fill the map with phantom obstacles. **This is a real blind spot** — a low rail or threshold is invisible and will still stop the wheels |
| **10 – 24 cm** | **the obstacle band.** Anything here, the rover hits |
| above 24 cm | **ignored.** The rover drives under it |

**The band was 0.10–0.22 for weeks, set to "the rover's height" before anyone
measured it.** The rover is 24 cm, so the top 2 cm of it was driving through
space nothing checked: an obstacle between 22 and 24 cm was invisible to the map
and could still hit the body. Set the top of the band from a tape measure, not
from memory.

The principle is exact and worth stating plainly: **the band should cover
precisely what can strike the rover — no more, no less.** Nothing above 24 cm can
hit a 24 cm rover, which is what lets it drive under both the table and the gate.
Nothing below the band can be seen, which is the cost.

With the original 0.60 m ceiling, a 29 cm table was inside the band — so driving
under one painted obstacles in **every direction**, and the rover was surrounded
by a tabletop it could comfortably fit beneath. Table *legs* span 0–29 cm and
stay flagged, which is what actually needs avoiding.

Heights are in the `odom` frame, whose z = 0 is where `base_link` started —
ground level. **Raise this if the rover grows a mast; lower it and it will drive
into things it cannot clear.**

### The slice height also sets your useful depth range

This is the non-obvious consequence, and it cost a day.

The band is only **14 cm tall**. Stereo depth error grows with the square of
range — at 5 m this camera's error is ~11.8 cm, most of the band. A distant wall
has its points smeared vertically right out of the slice, so it is never marked
solid, the ray passes **through** it, and free space is written beyond.

| range | depth error | vs a 14 cm band |
|---|---|---|
| 2 m | 1.9 cm | well inside |
| 3 m | 4.2 cm | usable |
| 5 m | 11.8 cm | most of the band |

Observed with the limit at 5 m: an 18 × 17 m blob, walls scattered through the
middle and none at the edges, 150 m² of "free floor" for one room. Both symptoms,
one cause. `max_integration_distance` is now **3.0 m**.

**The rule: the useful depth range is set by the SLICE HEIGHT, not the voxel size
and not the camera's spec range.** Want to see further? You need a taller band —
and a taller band re-integrates the tabletop. That is a genuine trade, not a knob
to turn freely. Change the band and recheck the integration distance in the same
breath.

### Measured

Standing still, then after one hand-driven loop:

| | parked | after driving |
|---|---|---|
| grid | 4.4 × 8.4 m | 5.3 × 8.0 m |
| floor seen | 1.63 m² | **3.55 m²** |
| obstacle | 1.77 m² | **2.48 m²** |
| unknown | 90.8% | **85.6%** |

The grid itself expanded and its origin moved, so nvblox allocated new blocks to
cover ground the rover drove into. `vo_dropped: 0` with 195 landmarks throughout —
the map was built on an honest pose.

### The map was forgetting

nvblox decays the TSDF **by default**. Every voxel's weight is multiplied by
`tsdf_decay_factor` (0.95) at `decay_tsdf_rate_hz` (5 Hz), and once it falls low
enough the block is **deallocated — deleted**.

That is a **2.7 second half-life**:

| not looked at for | weight remaining |
|---|---|
| 3 s | 50% |
| 10 s | 7.7% |
| 30 s | 0.05% — effectively erased |

Measured 2026-08-22: a **full room loop ended with less map than before it** —
8.74 m² of floor down to **3.04 m²** — because everything seen early in the loop
had been deleted by the time the rover came back round. The map looked good
locally the whole time, which is what made it hard to notice.

It is the right default for a scene full of moving things, where a stale
observation is worse than none. It is exactly wrong for surveying a static room.
Set to **0.0**. The cost is that a moving object leaves a permanent ghost; the
room does not move.

### Two traps, both silent

**`static_map_slice` is not an occupancy grid.** It carries
`nvblox_msgs/DistanceMapSlice`. The `nav_msgs/OccupancyGrid` that RViz and nav2
want is **`static_occupancy_grid`**. Subscribing to the wrong one receives
nothing while `ros2 topic hz` still cheerfully reports 5 Hz — because `hz` does
not check the type.

**nvblox publishes VOLATILE.** A `transient_local` subscription — the obvious
choice for something called a map — receives nothing and looks exactly like a
dead node.

A third, less subtle: **nvblox allocates GPU hash tables before it publishes**, so
a health check run 12 s after launch reads 0 Hz and looks broken. The gate now
waits 25 s and reports how much is mapped rather than only that a topic exists.

---

## 3. What Phase 2 does not do

- **The map dies when `fused` restarts.** The pose origin resets, so the old
  geometry would land in the wrong place. That is exactly what 2c fixes.
- **No relocalization.** The rover cannot recognise a room it mapped yesterday.
- **A teleport is unrecoverable.** nvblox has no way to un-write geometry
  committed at a wrong pose. Watch `jumps` in `/fusion/status`; if one happens,
  restart the map.
- **Only what the camera has seen is mapped.** A wall behind the rover stays
  unknown until it turns toward it — and turning is currently limited to wide
  arcs, see [TODO](TODO.md) §14.

---

## 4. Commands

```bash
./rover camera && ./rover pose && ./rover fused   # Phase 1
./rover map                                       # nvblox
./rover rviz                                      # on the Jetson; prefer a laptop
```

```bash
# how much is mapped, as numbers
python3 -u /logs/map_stats.py

# the map as text, with the rover on it -- live, or --once
python3 -u /logs/map_view.py

# drive a loop and see whether the line closes
python3 -u /logs/loop_test.py
```

Each runs inside the container after
`source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0`.

### Measured with `loop_test.py`

Driven under teleop, out and back through several turns:

```
back at the start, /odom is 6.8 cm away   -> PASS (want <= 10 cm)
heading is +1.2 deg off                   -> PASS (want <= 10 deg)
0 teleports, 0 frames dropped, landmarks 162
```

That run also corrected a Phase 1 "fact": it peaked at **42 cm/s** with zero
teleports, against a documented ~25 cm/s limit. See [TODO](TODO.md) §3 —
teleports track **texture**, not speed.
