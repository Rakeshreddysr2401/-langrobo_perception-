# TODO — known bugs and open questions

Everything I know to be wrong or unverified, in one place. Nothing here is
hidden in a code comment.

**Status key:** 🔴 blocks the goal · 🟠 real bug, worked around · 🟡 unverified ·
⚪ accepted limitation

---

## 🔴 1. The map smears while driving — cause not found

The headline problem. A **parked** rover builds a clean map (occ/free 0.46, thin
edges); a **driven** one produced a blob (1.66, walls ~1 m thick).

Ruled out by measurement on 2026-08-11 — see `FACTS.md §4`:

- the floor entering the slice band (ratio unchanged across two bands)
- noise accumulating with decay disabled (only ~5 cells/min)
- pose drift while parked (0.0000 m over 160 s)

**The only remaining suspect is pose error while moving.** Prior evidence
supports it: cuVSLAM under-reads translation ~4x on low-texture floors and can
freeze silently.

**Next action:** `./rover.sh measure 2.00` — push the rover a tape-measured
2.00 m and see what it reports. This is task 02's gate and it has never been
run since the emitter was turned off. **Do not tune nvblox before doing this.**

---

## 🔴 2. The ESP32 is not connected

`/wheel_state` reads 0–1 Hz. The firmware publishes unconditionally every 50 ms
once `AGENT_CONNECTED` (`rover_firmware_v2.ino:391`), with no motion gating, so a
low rate cannot mean "the rover was parked". The only other 1000 ms timer in the
file is the `WAITING_AGENT` ping at `:374`.

So this is "not connected", not "slow".

**First thing to check:** the **micro-ROS agent on the Pi 5**. Like the wheel-odom
relay, it has no systemd unit, so a reboot leaves the ESP32 with nothing to
connect to.

**Do not reflash.** That was the 2026-08-08 fix and it is done; 1 Hz means
something different now.

Consequence while this is broken: the EKF loses its independent speed
cross-check, so nothing contradicts cuVSLAM when it lies. That makes item 1
harder to diagnose, so fixing this first is defensible.

---

## 🟠 3. The collision monitor is silently unprotected

`depth_to_cloud.py` feeds `/perception/depth_points`, which is
`collision_monitor`'s only obstacle source. Measured 2026-08-10: it publishes at
**~0.5 Hz with gaps up to 7.3 s**, and stalled ~10 s during a live drive. The
monitor's `source_timeout` is **2.5 s**.

When a source is stale the monitor **does not brake — it ignores it**. So nav2's
collision layer effectively has no obstacle input, and nothing says so out loud.

This must be fixed or consciously accepted **before task 07 (nav2)** ever moves
the robot. `safety_guard.py` is the remaining protection.

---

## 🟠 4. `esdf_slice_min_height` is still at a stale workaround value

`config/nvblox.yaml` uses `0.12`. It was raised from 0.05 to dodge a 3.7 cm
camera-height error **which is now fixed** (floor reads 0.000). The 2026-08-11
slice experiment further showed the floor is not what fills the map.

So the last two reasons for keeping it are gone, and it costs every obstacle
shorter than 12 cm. It should come down toward ~0.06.

**Not changed yet on purpose:** it alters what nav2 calls an obstacle, so it must
be validated while driving. Belongs to task 06.

---

## 🟠 5. The map never forgets

`tsdf_decay_factor: 0.999999` and `map_clearing_radius_m: -1.0` disable nvblox's
two deletion mechanisms, because the defaults were erasing the room after ~33 s
(`FACTS.md §4`).

The cost is real: a person who walks through leaves a **permanent ghost**, and
pose drift smears walls instead of letting them fade. Memory also grows without
bound.

The tempting fix — a slow decay like 0.9999 — **does not work here**, and it is
worth understanding why: decay removes whatever is not re-observed, but with a
forward-only camera the room *behind you* is never re-observed. Any decay fast
enough to clear noise is fast enough to delete the room. The real lever is
integration weighting / depth outlier rejection, not decay.

---

## 🟡 6. "Subscribing kills the camera" is strong correlation, not proof

Two occurrences on 2026-08-11, both immediately after a new subscriber attached
to a raw camera topic, with a plausible mechanism (lazy publishing → stream
start/stop over the DDS control channel → `"id":"hwm"` timeout → `device is
offline`).

But `floor_probe.py` has subscribed to depth successfully in the past, so it is
not deterministic. The refined guess is that **repeated connect/disconnect** is
what does the damage, not a single long-lived subscription.

Treated as a hard rule anyway, because the cost of being wrong is a walk to the
rover and a full restart. If you want to settle it properly, that is a
deliberate experiment on a day when losing the camera does not matter.

---

## 🟡 7. Layer gates are written but never executed

`rover.sh l1`–`l5` and every gate in `tasks/` were **written, not run**. The
promoted code was working in the old repo, but this repo's wiring — the new
mount path, the new container name, the layer sequencing, the `assert_rate`
helper — has not been executed once.

Expect the first run to fail on something small. That is the plan: the whole
point of layering is that the failure will name its own layer.

---

## 🟡 8. `stack_status.py` still expects the old all-at-once stack

It reports layers as "not started" with hints like `run: fuse` and `run: vision`,
which are the **old** script's subcommands. Harmless but confusing, and it does
not know about the L1–L5 model.

Should be reworded to the layer names once the layers have actually been run
once and their real steady-state rates are known.

---

## ⚪ 9. The camera sees forward only

~87°, no pan/tilt head. The rover is blind to its sides and behind, so a map is
only ever as complete as the path you drove. Hardware limitation.

A pan/tilt head is the single biggest capability upgrade available, and it is
out of scope until the goal in `PRD.md` is met.

---

## ⚪ 10. The Pi 5 services do not survive a reboot

Neither the wheel-odom relay nor the micro-ROS agent has a systemd unit, so both
must be started by hand after every reboot. This is the likely cause of item 2.

Fixing it means working on the Pi 5, in the `pi5_ros2_ws` repo — **not here**.

---

## ⚪ 11. Nothing is verified about nav2 in this repo

`config/nav2.yaml` was copied across and its behaviour-tree path corrected, but
nav2 has not been launched from this repo at all. The costmaps use
`global_frame: odom` while `bt_navigator` uses `map` — deliberate (a costmap in
`map` jumps under the robot at every loop closure), but it means the map cannot
persist across sessions. That trade-off is task 05's subject.
