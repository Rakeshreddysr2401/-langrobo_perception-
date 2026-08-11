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

## 🟠 3. The collision monitor's source rate has never been measured

**Rewritten 2026-08-11 — the previous version of this item was wrong**, and wrong
in the reassuring direction. It said the monitor "does not brake, it ignores a
stale source", making this a silent loss of protection. Both the node it accuses
and the config it cites say the opposite:

| Previous claim | What the machine says |
|---|---|
| the source runs at ~0.5 Hz | that was nvblox's `back_projected_depth` **debug** topic, which `nodes/depth_to_cloud.py` was written on 2026-08-10 to replace (`depth_to_cloud.py:5-17`) |
| `source_timeout` is 2.5 s | `config/nav2.yaml:81` says **1.5**, with the 1.0 → 2.5 → 1.5 history written out |
| a stale source is ignored | a stale source logs `"Robot to stop due to invalid source"` and **holds the robot at zero** — observed live, `nav2.yaml:74-80` |

So the monitor is **fail-safe**, and the real risk is the opposite one:

> If `depth_to_cloud.py` cannot hold its 10 Hz against the 1.5 s timeout, the
> rover is **pinned at zero** by its own safety layer while nav2 plans happily,
> and every goal dies on "Failed to make progress" — which is exactly the
> confusing failure of 2026-08-10, wearing a different mask.

**Still unverified, because nothing in this repo has ever run (§7).** The node's
claimed cost is one strided numpy deprojection per frame (~7k points at stride 8),
but that is a design intent, not a measurement.

**Still a hard blocker for task 08**, with the gate changed from "add protection"
to **"measure the rate"**: `/perception/depth_points` must sustain ≥ 5 Hz with no
gap over 1.5 s, measured while driving, before anything explores by itself.

Related: §14, the second-subscriber problem inside the same node.

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

## 🟠 9. Rotation is now load-bearing, and it is untested

Task 08 fills in the 87° blind spot by **stopping and spinning in place** at each
waypoint. That makes rotation a core capability rather than a nice-to-have — and
rotation is what visual odometry handles worst, because features leave the frame
fast and there is little parallax.

Nothing has measured how well the pose survives a full 360° turn on this rig.
Task 03's gate now includes a hand-rotated 360° check specifically to find this
out early, at zero speed, before the robot is doing it under its own power.

If heading drifts badly in a turn, task 08 cannot work as designed and the
fallback options are the pan/tilt head (which fights cuVSLAM) or a 2D lidar.

---

## ⚪ 10. The camera sees forward only

~87°, no pan/tilt head. The rover is blind to its sides and behind, so a map is
only ever as complete as the path you drove. Hardware limitation.

The pan/tilt **servos are already wired** to GPIO18/19, but firmware v2 has no
servo driver and there is no Jetson node to model the pan in TF. So the hardware
gap is smaller than it looks — the software gap is not.

And it is not a free win: panning the camera while the body is still makes
**cuVSLAM think the robot moved**. Pan and visual SLAM on one camera fight each
other, and the pan would have to be compensated out of the SLAM input. That is
why task 08 spins the whole robot instead.

---

## ⚪ 11. The Pi 5 services do not survive a reboot

Neither the wheel-odom relay nor the micro-ROS agent has a systemd unit, so both
must be started by hand after every reboot. This is the likely cause of item 2.

Fixing it means working on the Pi 5, in the `pi5_ros2_ws` repo — **not here**.

---

## ⚪ 12. Nothing is verified about nav2 in this repo

`config/nav2.yaml` was copied across and its behaviour-tree path corrected, but
nav2 has not been launched from this repo at all. The costmaps use
`global_frame: odom` while `bt_navigator` uses `map` — deliberate (a costmap in
`map` jumps under the robot at every loop closure), but it means the map cannot
persist across sessions. That trade-off is task 05's subject.

**Decided on paper 2026-08-11** (`tasks/05-persistence.md`): nvblox and the
**global** costmap move to `map`, the **local** costmap stays in `odom`. The jump
objection applies to the local costmap — the one MPPI samples while moving — and
that one is not moving. A map anchored to `odom` cannot persist at all, so task
05's gate is unreachable otherwise. Precondition before touching either file:
measure how often and how far `map → odom` actually steps on this rig.

---

## 🟠 13. The container image cannot be rebuilt by ANYONE — there is no recipe

**Upgraded from ⚪ to 🟠 on 2026-08-11.** This item used to say the Dockerfile
lived in the old repo and copying it across would fix things. Investigating that
turned up something worse.

`docker/Dockerfile` has now been copied in (unbuilt, labelled — see
`docker/README.md`). It does **not** discharge this item, because it is fifteen
lines of `COPY` on top of `FROM isaac_ros:cuvslam-unified`, and the chain above
that has no recipe at all:

```
orin-nav:1.1                                 ← the only part with a Dockerfile
  └─ isaac_ros:cuvslam-unified   57.8 GB     ← no Dockerfile exists, anywhere
       └─ isaac_ros:langrobo-prod 54.4 GB    ← no Dockerfile exists, anywhere
```

`orin-nav:1.1` carries `com.docker.compose.project=robot` and
`...config_files=/home/rakhi24/robot/docker-compose.yml` labels — the fingerprint
of an image made by **`docker commit` on a running container**. It was never
built from a file, so it cannot be reproduced from one.

**Consequence:** if that image is deleted, the entire repo stops, and no amount
of source-code archaeology brings it back. Rebuilding means redoing the original
container work from scratch, starting from
`orin-nav-stack/standalone/Dockerfile.cuvslam-jp72` (which *is* a complete recipe,
but for a cuVSLAM-only image with no nvblox and no nav2).

**The only real insurance is an image export, not a Dockerfile:**

```bash
docker save orin-nav:1.1 | gzip > /path/to/external/orin-nav-1.1.tgz
```

⚠️ Not done yet, and it needs somewhere to go: `/` has **45 GB free of 227 GB**
and the export will be tens of GB, so this must target external storage, not the
Orin's own disk. Do it before the first session that touches Docker.

Also unchanged: `firmware/` stays in the old repo (and per an earlier decision,
rover firmware really belongs in the Pi 5 repo `pi5_ros2_ws`, not the Jetson one).
This repo cites it by line number but does not carry it.

---

## 🟠 14. `depth_to_cloud.py` is a second subscriber on the fragile depth stream

Found 2026-08-11 while rewriting §3. `FACTS.md §1` states plainly that **nvblox
is the only depth subscriber**, and that a new subscriber attaching to a raw
camera topic took the D555 offline twice in one day.

`depth_to_cloud.py:57` subscribes to `/camera/camera0/depth/image_rect_raw`. That
makes two — permanently, on the exact topic FACTS §1 says not to touch. The rule
and the code have contradicted each other since 2026-08-10 and neither file
mentioned it.

**Mitigated, not resolved.** `depth_to_cloud` moved from L5 to L4 on 2026-08-11,
so it starts in the same step as nvblox. Per §6 the suspected killer is repeated
connect/disconnect **transitions**, not a long-lived subscription — so this makes
L4 a single attach event instead of two, and, the real win, lets L5 (nav2) be
restarted as often as tuning needs without ever cycling the depth stream.

Still unknown: whether a second permanent subscriber changes the D555's streaming
behaviour at all. **The first `./rover.sh l4` after this change is the test, and
it is a camera-risk moment** — if the camera goes offline, only a physical PoE
power-cycle recovers it. Run it on a day when a walk to the rover is acceptable.
