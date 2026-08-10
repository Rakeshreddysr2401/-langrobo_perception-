# 05 — Map on the fly: build a map as I drive, and keep it

> **STUB.** Written in full when you get here.

**Needs:** issue 04 passed (you understand what nvblox is showing).
**Motors move:** **YES — first time in this plan.** You drive, by phone.

---

## Goal

Drive the rover around a real room by hand-held phone control, watch the walls
appear in RViz as it goes, save the map, and have it still be there next session.

## What you will learn

- The difference between the **cuVSLAM map** (feature landmarks, used to
  relocalize) and the **nvblox map** (occupancy geometry, used to plan) — two
  different maps, saved separately, and both needed
- What **relocalization** is: recognising a place you've been so a saved map
  lines up with where you are now
- Why driving slowly and turning gently produces a dramatically better map

## Known state going in

- Phone teleop is live at **`http://192.168.1.16:8091`** — hold-to-move.
  > ⚠️ Teleop publishes **straight to `/cmd_vel`, bypassing `safety_guard`**.
  > When you drive by phone, **you are the safety system.** Drive by sight, not
  > by RViz.
- `./run_stack.sh remap` gives a fresh map origin.
- `ros2 service call /slam/save_map std_srvs/srv/Trigger` saves the SLAM map.
- The saved map in `maps/current` is from **Jul 19** and is a 15-second stationary
  capture — far too sparse to relocalize against. **Treat it as empty.** This
  issue makes the first real one.
- **Open gap:** nvblox is anchored to `global_frame: odom`, which is different
  every boot, so saved walls never line up on reload. Wiring `global_frame: map`
  plus reload is real new work in this issue.

> ⚠️ **`remap` and `fuse` both wipe the nvblox map.** Save before either.

## Likely work

1. Fresh origin, drive a full loop of one room slowly, watch nvblox fill in.
2. Save both maps.
3. Wire `global_frame: map` + reload so the walls survive a restart.
4. Prove it: save, restart the whole stack, `/slam/localize`, confirm the old
   walls land on the real walls.

## Gate

*To be written when we get here.* Roughly: a recognisable room in RViz, a
`data.mdb` + `.nvblx` newer than Jul 19, and after a full restart the reloaded
walls line up with the real ones.
