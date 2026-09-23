# description/ — the rover's geometry: CAD and URDF from one file

```
params.yaml   every measurement, raw, with status + source + date   ← edit this
build         regenerate everything below and check the repo for drift
build.py      the derivation (params -> frames), URDF writer, CAD, drift check
rover.urdf    GENERATED: frames + boxes/cylinders, no <inertial> yet
cad/          GENERATED: rover.step (open in FreeCAD / Onshape) + one .stl per part
```

## Workflow — "as we go"

1. Measure something (see ROVER_BUILD_PLAN.md §1).
2. Put it in `params.yaml` the way it was measured (from the nose, from the
   floor...). Unknown stays `value: null`. **No guesses, no placeholder masses.**
3. `description/build`: rewrites `rover.urdf` and `cad/`, prints the derived frames,
   and checks all 22 other copies of these numbers (`./rover`, nav2 footprint,
   `vo_node`, `rover_marker`, firmware, pivot safety corners). Exit 1 = a copy
   disagrees; the output says what it should be.
4. Fix the copies it names, rerun until `drift: N/N`, commit.

A new part (battery, Jetson, Pi 5, motors, drivers...) goes under `components:`
as a box with `size` and `centre` in base_link. It shows up in the URDF and the
CAD on the next build. `mass` stays null until it is weighed.

The CAD needs build123d; `build` makes `~/.venvs/cad` on its first run.

## Frames

base_link: **on the floor, midway between the four wheel contact patches** —
x forward, y left, z up. The nose is at +0.182 (axles 6.3 / 30.1 cm behind it).

| frame | x | y | z | note |
|---|---|---|---|---|
| wheels | ±0.119 | ±0.170 | 0.041 | fixed joints until /joint_states exists |
| camera0_link | 0.173 | −0.0475 | 0.175 | the D555's LEFT IR imager, at the front glass; the driver publishes everything below it |
| laser | 0.1342 | 0 | 0.2498 | yaw 1.5463 rad, calibrated by driving |

Two things that look like mistakes and are not:

- **camera y is −4.75 cm** although the camera is centred: `camera0_link` is the
  left imager, half the 95 mm baseline from the case centre. The driver's own
  TF puts infra2 at +0.095 from it, which fixes the sign.
- **The odometry wheel diameter (85 mm) differs from the tyre (83 mm).** It stays
  until a taped 1 m drive says which one gives true distances.
  Likewise `effective_track` (0.5216) is a turning calibration, not the 0.34 track.

## Not yet (ROVER_BUILD_PLAN.md step 4)

The live TFs still come from `static_transform_publisher` calls in `./rover`. The
switch is: `robot_state_publisher` on `rover.urdf`, RViz on the URDF, and
`rover_marker.py` retired. Until then the drift check keeps the copies honest.
