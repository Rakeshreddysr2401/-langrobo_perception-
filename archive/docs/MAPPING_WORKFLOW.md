# Map-first navigation — how to make "go to a coordinate" actually work

*Written 2026-07-18. Companion to NAVIGATION_PIPELINE.md. Explains why a
freshly-started rover can only navigate ~0.15 m, and the procedure to grow that
into a real navigable area — without ever driving blind.*

---

## Why you can't just send a far coordinate

Nav2 here is deliberately **anti-crash**: the planner has `allow_unknown: false`
and the global costmap has `track_unknown_space: True`
(`config/nav2_real.yaml`). Together they mean:

> **Nav2 will only plan through floor the camera has actually seen as free.**

The D555 is **forward-only** and blind under ~0.4 m, so a stationary robot has
observed only a narrow cone of floor — plus a small free disc right around
itself. Everything else is `NO_INFORMATION`, and the planner refuses it. That is
the wall-strike protection working, not a bug. The cost is: **until you map an
area, you can't navigate into it.**

Check the current navigable radius any time (planning only, no motion):

```
scripts/nav_reachability.sh
```

A fresh/stationary robot reports ~0.15 m in each direction. After mapping it
grows to metres and **persists** (see step 4).

## What already got fixed (2026-07-18)

- **Unreachable goals now FAIL cleanly** (`ABORTED`, error 208) instead of the
  old fake "arrived". The NavFn `tolerance` was routing impossible goals to the
  robot's own cell → instant false SUCCESS with zero motion. Tightened to 0.15.
- **Arrival is position-based** (`SimpleGoalChecker`), so the pulsing
  deadband drive can't leave a goal hanging just because the rover wasn't
  perfectly stopped at the tolerance edge.

So the brain can now trust the result: `SUCCEEDED` = really there, `ABORTED` =
not reachable (map it first).

---

## The procedure

> Prerequisite: the drivetrain must physically drive on the floor. As of
> 2026-07-18 it is torque-limited (wheels spin in the air, stall on the ground)
> — see [[rover-swap-drivetrain-diagnosis]]. Do this once that is resolved.

### 1. Fill the blind donut — rotate in place
The area right around a stationary robot is unseen. A slow **in-place rotation**
sweeps the forward cone through 360°, so nvblox marks the surrounding floor
free. Do one full slow turn before expecting to plan sideways/behind.

### 2. Teleop-drive the area you want to navigate
Drive the rover slowly over every route you'll later ask it to take, keeping
walls/obstacles in the camera's view. nvblox fuses each observed cell as free;
the planner's reachable set grows behind you.

Drive **through the safety layer** so the collision monitor still guards you:

```
# publish to cmd_vel_smoothed → collision_monitor (SlowZone/SafeZone) → cmd_vel
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_smoothed
```

Keep speeds low (≤0.2 m/s). Re-run `scripts/nav_reachability.sh` and watch the
radius climb.

### 3. Verify before trusting it
`scripts/nav_reachability.sh` should now report the area you drove. Only send
autonomous goals inside that reachable envelope.

### 4. The map persists
RTAB-Map saves to `/data/rtabmap.db` and relocalises against it on boot, so a
mapped area **stays navigable across reboots** — map a room once, drive it by
coordinate forever. (`config/localization`.)

---

## Sending a coordinate goal (once mapped)

```
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.2, y: 0.3}, orientation: {w: 1.0}}}}"
```

`SUCCEEDED` = arrived. `ABORTED` = not reachable through known-free space — map
the corridor to it first (§1–2). The Pi5 brain reaches the same action via
`pi5/langrobo_client.py go x y`.

## The safety contract (unchanged)
- Plans only through seen-free space (`allow_unknown: false`).
- 0.35 m inflation buffer; MPPI respects the footprint.
- Collision monitor: 0.45 m slow ring, 0.25 m hard-stop ring.
- No blind reverse (BackUp removed from the tree).
- ESP32 500 ms watchdog.

None of this is relaxed by the map-first workflow — you are giving the planner
*more seen-free space to work with*, not letting it guess.
