# TODO — known problems, in one place

Status: 🔴 blocks a gate · 🟠 real, worked around · 🟡 unverified · ⚪ accepted

---

## 🟡 2. The drift gate has never been run — and it is no longer blocked

> **Unblocked 2026-08-23.** This said the gate could not pass without wheel
> data, and `/wheel_state` was stuck at 1 Hz. It now runs at 20.004 Hz (§1),
> so the stated blocker is gone. The gate itself has still never been run, which
> is why this stays open rather than closing.

2 m out-and-back, clean run (peak 16 cm/s, no teleports):

| | endpoint error |
|---|---|
| cuvslam alone | 28.4 cm |
| FUSED (gyro heading) | **12.2 cm** |
| gate | ≤ 10 cm |

Fusion fixed the heading half. The residual is **distance**: the return leg
registered 186.5 cm against the outbound 198.0, so reversing under-reads by ~6%.
cuVSLAM is the only translation source, so nothing can contradict it. Encoders
are exactly that missing measurement — see §1.

Note the 10 cm bar was **chosen, not measured** (2 voxels at 5 cm). If a re-run
lands near 12 cm again, that is the rig telling us encoders are required, not a
bug to chase.

---

## 🟡 3. Teleports are driven by TEXTURE, not speed — the 25 cm/s limit was wrong

Every run we have, cross-tabulated:

| run | peak speed | landmarks | teleports |
|---|---|---|---|
| clean 2 m push | 18.8 cm/s | healthy | 0 |
| failed out-and-back | 76.5 cm/s | — | 1 |
| hard drive, bare wall | 94 cm/s | **min 4** | **32** |
| hard drive #2 | 84 cm/s | **min 17** | **12** |
| **teleop loop, 2026-08-21** | **42 cm/s** | **162** | **0** |
| room loop, 2026-08-22 | 89 cm/s | 19 (at end) | **0** |
| room loop #2, 2026-08-22 | 84 cm/s | 40 (at end) | **0** |

**42 cm/s with good texture produced zero teleports.** The runs that failed were
not the fast ones, they were the blind ones. And the 84–94 cm/s figures are
themselves *inflated by* the teleports — peak speed is computed from cuVSLAM's
own position deltas — so the real driving speed in those runs was lower than it
looks, which weakens the speed correlation further.

The original entry claimed a ~25 cm/s limit from correlating teleports with peak
speed across two runs. That correlation was real and the causation was not:
landmarks were the confound, and they move together because a bare wall is both
featureless and where you tend to drive faster.

**Consequence for nav2:** do not cap `vx_max` at 25 cm/s on this evidence. The
honest rule is to **watch landmarks, not the speedometer** — `compare.py` and
`fusion_node` already drop cuVSLAM below 30 of them, and `/fusion/status`
publishes the count. A texture-aware speed limit would be better than a fixed
one, but this needs a deliberate test: drive the same textured route at
increasing speed until it breaks. That has not been done.

**2026-08-22 strengthens this considerably.** Two full room loops at 84 and
89 cm/s -- more than three times the supposed limit -- produced **zero**
teleports and zero dropped frames each. That is the fastest clean driving we
have recorded, and the 25 cm/s figure now looks not merely unproven but wrong
by a factor of three.

One caveat on those two rows: `loop_test` reports the landmark count *at the
end of the run*, whereas the failing rows above report the *minimum during* it.
They are not the same measurement and the low numbers are not comparable. The
teleport counts are directly comparable, and they are zero.

**Not yet ruled out:** that speed matters *at the margin*, when texture is
already thin. Nothing here separates those.

---

## 🟠 4. cuVSLAM is much worse in reverse

Same out-and-back, split by leg:

| leg | heading error vs gyro |
|---|---|
| outbound (0 → 198 cm) | −1.74°, tracking within 0.13° most of the way |
| return (198 → 11 cm) | **+7.68°** |

Inherent: driving forward, features expand outward from the image centre with
long track histories; reversing, they shrink toward the centre and new ones must
enter at the edges where they are worst observed.

Worked around by taking heading from the gyro. **Design consequence for Phase 3:**
prefer plans that turn and drive forward over plans that reverse.

---

## 🟡 5. The 360° spin gate has never been run

Rotation is visual odometry's weakest case and Phase 2 onward depends on it.
`./rover compare --spin 360`.

---

## 🟡 6. Camera mount pitch never measured

`cam_x` 17.0 cm, `cam_z` 16.3 cm and a 2.06° mount **yaw** are all measured. Any
**pitch** (nose up/down) is not. It does not affect Phase 1's numbers much but it
will move the ground plane in Phase 2's mapping.

---

## 🟠 7. The D555 drops out constantly, in three distinct ways

It went down **four times on 2026-08-15 alone**. For an autonomous rover this is
the least solved thing on the vehicle — a robot that needs a human to reseat a
cable is not autonomous. The three failures look similar and are not:

| symptom | what is true | fix |
|---|---|---|
| `No RealSense devices were found` **and** `ethtool` says `Link detected: no` | no electrical link at all — cable or PoE injector | reseat the cable; nothing on the camera can help |
| `dds-device.cpp:46 device is offline`, driver alive, 0 publishers | link fine, on-camera DDS server dead | unplug 5 s, replug |
| streams at 30 Hz but `ros2 param set` times out | **half-alive**: image traffic works, option traffic does not | wait ~30 s after power-up, then retry |

That third one is new and dangerous. Seen 2026-08-15: infra1 streaming at
30 Hz with the emitter stuck ON, and every attempt returning
`timeout waiting for reply: {"id":"set-option","option-name":"Emitter Enabled"}`.
Everything looks healthy and the pose is quietly worthless — this is the failure
that made a 60 cm push read 1.1 cm. **The camera answers image traffic before it
answers option traffic**, so attaching the driver too soon after a power-cycle
lands here. `./rover camera` now retries the emitter set for ~30 s and refuses to
continue if it never takes.

**Ping is not a health check** in any of the three. Nor is a live image stream.
The only proof is the emitter read-back.

---

## ⚪ 8. The container image cannot be rebuilt

`orin-nav:1.1` was made by `docker commit`, not from a Dockerfile, and neither it
nor its 57.8 GB base has a recipe. If it is deleted, everything stops. The only
insurance is `docker save` to external storage. `/` has ~45 GB free of 227 GB, so
this needs somewhere else to go.

---

## 🟡 15. The estimate has never actually been graded — the loop test measured the wrong thing

`loop_test.py` printed **FAIL** whenever `/odom` did not finish within 10 cm of
where it started. That grades the *driving*. Nobody parks a rover back onto a
tape mark to 10 cm by eye down a phone screen, so it reported failure on runs
where the estimate was excellent and merely said so honestly.

Two room loops on 2026-08-22, both graded FAIL:

| run | /odom says | tape says | estimate error | driven |
|---|---|---|---|---|
| 1 | 120.6 cm | ~100 cm | **~20 cm = 1.6%** | 12.9 m |
| 2 | 71.2 cm | *never measured* | unknown | ~12 m |

Run 1 is **good stereo VO** and the verdict called it a failure. We went looking
for a regression that did not exist.

The pose error is the DIFFERENCE between what odometry claims and what a tape
says, and only one of those two numbers is available inside the program. The
test now asks for the tape reading and grades the difference as a percentage of
distance driven (PASS ≤ 2%, OK ≤ 5%).

**Still open, and this is the real item:** we have exactly one measured data
point, and it was eyeballed as "around 1 metre". Run 2's **−22° heading error**
is the part worth chasing — twice run 1's 11°, and heading error is what smears
a map into a blob. Neither has a tape reading behind it.

**Next run:** measure the gap with a tape when the test asks. Until then the
odometry is *probably* around 1.6% and we cannot say more than that.

---

## 🟠 16. The SLAM pose diverged 91 m — and the frame split contained it

First real outing for loop closure, 2026-08-22, straight after a room loop:

```
odom -> base_link   [-0.828, 0.876,  0.000]     z exactly 0 -- healthy
map  -> odom        [-10.5,  -22.2, 87.685]     87 metres UP
correction_m        91.045
```

The SLAM optimiser diverged exactly as the odometry pose once did (§ the z-gate
work), and **`planar_constraints = True` did not prevent it** — which is worth
recording on its own, because that setting was added specifically to make this
impossible and it did not.

A correction is accumulated drift made visible. On a ground rover over a room
that is centimetres to a metre or two; it is never tens of metres and never
vertical. So `_publish_correction` now applies the same plausibility test the
odometry gets (`MAX_CORRECTION_Z` 0.30 m, `MAX_CORRECTION_M` 5.0 m) and
discards rather than publishes.

**What did NOT happen is the point.** `odom -> base_link` stayed perfectly
healthy throughout, because the correction lives in a different frame. Keeping
loop closure out of the odom frame was argued for on the grounds that a closure
would look like a teleport to the fusion guard; its first real outing showed
the other half of the argument, which is that a diverged optimiser cannot drag
down the frame nav2 and nvblox consume.

**Open:** loop closure has therefore still never usefully fired. The gate stops
the damage; it does not make closure work. Why the optimiser diverges — and
whether `planar_constraints` is being applied at all — is unexamined.

---



## 🔴 21. cuVSLAM diverges repeatedly, and the fallback is SILENT

Third divergence, 2026-08-22 19:09, found only because a pose reading looked
1.5 m off and I went looking:

```
vo_z              -40.087 m     forty metres underground
vo_implausible      99394       and climbing at ~30/s
dead_reckoned        3341
dr_metres           21.409 m    travelled on dead reckoning alone
```

At ~30 rejections/second, 99394 is about 55 minutes — and `fusion_node` had
been up 57 minutes. **It had been diverged for essentially the entire session**,
including both autonomous goal drives.

### The gate works. That is not the problem.

`VO_MAX_Z` caught it and the fused pose never followed cuVSLAM underground.
Restarting `vo_node` recovered it completely: `vo_z` −40.087 → −0.005,
`vo_implausible` steady at 0, 110 landmarks, 0 jumps.

**The problem is that nothing said so.** The rover degraded to gyro-plus-wheels
dead reckoning and carried on looking healthy: `/odom` at 20 Hz, `ready: true`,
`vo_alive: true`, gates passing, nav2 planning and driving. Every check we run
routinely was green while the pose quietly ran open-loop for 21 metres.

Worth noting what dead reckoning actually managed: the 1 m autonomous goal read
95.8 cm and the tape said 93–95. Over a metre it is fine. Over 21 it is not, and
nothing distinguishes the two on screen.

### What needs doing

- **`./rover fused` does NOT reset cuVSLAM.** It restarts `fusion_node` only.
  Recovering a diverged tracker needs `./rover pose` first. Easy to get wrong
  while debugging, and it silently leaves the divergence in place.
- **Surface it.** `vo_implausible` climbing should be loud — in the `fused` gate,
  in `./rover status`, and ideally as a red line in `compare.py`. A counter you
  have to go and ask for is not a warning.
- **Find out why it diverges.** `planar_constraints = True` was set specifically
  to make vertical drift impossible and has now failed three times (−21.7 m,
  +87.7 m, −40.1 m). Either the flag does not do what its name says on this
  build, or something upstream — extrinsics, the IR emitter, frame conjugation —
  is feeding it a rotation it cannot reconcile. Nothing here has been examined.
- **Consider whether the gate is too blunt.** It rejects on absolute z. At the
  moment of divergence the x/y estimate may still be usable, and throwing the
  whole pose away forces dead reckoning. Rejecting on z *rate* rather than z
  *value* would keep more good data — but this is a design change, not a tweak.

### Cost this session

The map and the odom origin were both discarded to recover, because resetting
the pose invalidates a map built in the old frame.

---

## ✅ 17. The slice height caps the useful depth range — FIXED, and re-measured since

> **Closed 2026-08-23.** Integration distance is 3.0 m and the band is now
> 0.10–0.24 m, set from a tape measure: the rover is 24 cm tall, so the old
> 0.22 m top left 2 cm of it unprotected. The rule below still holds — the
> useful depth range is set by the SLICE HEIGHT, so re-check the integration
> distance whenever the band changes.

A real room loop produced an 18 × 17 m blob with walls scattered through the
**middle** and none at the edges, claiming 150 m² of free floor for a room
nothing like that size. Both symptoms are one cause.

The obstacle band is 0.10–0.22 m — 12 cm, set to the rover's own height so it
can drive under the 29 cm table. Stereo depth error grows with range squared:

| range | depth error | vs the 12 cm band |
|---|---|---|
| 2 m | 1.9 cm | well inside |
| 3 m | 4.2 cm | about a third — usable |
| 5 m | 11.8 cm | **the entire band** |

A wall 5 m away has its points smeared vertically across the whole band. Most
miss it, the wall is never marked solid, the ray passes **through**, and free
space is written beyond it — which is exactly walls-only-near-the-rover plus an
absurd free area.

`projective_integrator_max_integration_distance_m` is now **3.0 m**, down from
the 5.0 m an earlier commit *the same day* had raised it to in order to reach
far walls. It reached them with data too noisy for the band to catch.

**The general rule, worth remembering:** the useful depth range is set by the
SLICE HEIGHT, not by the voxel size or the camera's spec range. Any change to
`esdf_slice_min/max_height` should be followed by rechecking this distance.

**Not yet verified:** the 3 m setting has not been driven. The map was cleared
when it was applied and no loop has been run since.

---
## ✅ 18. nav2 would not plan past ~0.9 m — CAUSE FOUND IN §19, not the map

> **Resolved 2026-08-23, and the diagnosis below was WRONG.** This entry blamed
> map noise. Rebuilding the map fixed every map metric (§17) and nav2 still could
> not plan a metre. The real cause was the costmap layer double-counting a
> distance gradient — see §19. Kept because the measurements are sound and only
> the conclusion was not; it is a good example of a plausible cause that survived
> two sessions because it fit the symptoms.

First goal ever sent to nav2, 2026-08-22. All four servers active, `/cmd_vel`
free, route clear. `compute_path_to_pose` returned **NO_VALID_PATH (208)**,
"Failed to create plan with tolerance of: 0.250000".

Bisecting the distance, straight ahead:

| goal | result |
|---|---|
| 0.3 – 0.9 m | OK (path length saturates at 18 poses) |
| **1.0 m** | **NO_VALID_PATH** |

And it fails at 1.0 m in *every* direction — +x, −x, +y, −y. Not an obstacle,
not a heading: a radius.

**The rover is sealed in a 1.5 × 0.9 m pocket of traversable costmap.** Drawn
with the correct thresholds, it is ringed by INSCRIBED/LETHAL cells on all
sides. Global costmap census: 57.7% unknown, 5.8% free, 9.4% traversable,
**21.8% INSCRIBED + 5.2% LETHAL**.

### Why: the ESDF slice says nowhere is far from an obstacle

The costmap layer consumes `/nvblox_node/static_map_slice` — distance to the
nearest obstacle per cell — not the occupancy grid:

```
known cells            23050 (49% of the slice)
median distance        0.35 m      <- in a room this should be ~1 m
further than 1.0 m     15% of known   -> these become FREE
closer  than 1.0 m     84% of known   -> these get cost
```

With `max_obstacle_distance: 1.0`, every obstacle cell projects a 1 m halo of
cost, and the inflation layer then marks everything within the inscribed radius
(~0.18 m) of a lethal cell as INSCRIBED, which NavFn treats as blocked. With
spurious obstacles scattered through the map, those halos merge and seal it.

**This is § 17 showing up downstream.** The same map reports 7.4 m² of wall for
a room whose perimeter at 5 cm thickness should be well under 1 m² — roughly
eight times too much obstacle. Every one of those false cells is a 1 m
no-go halo. The map is not merely ugly, it is unplannable.

### What has NOT been ruled out

- The 3 m integration fix (§17) landed but **the map has never been rebuilt by
  driving since**. The current map was accumulated largely from a stationary
  rover, whose distant observations are exactly the noisy ones.
- Whether `max_obstacle_distance: 1.0` is itself too generous for a 5 cm-voxel
  map in a small room. It is nvblox's common default; it has never been tuned
  here, and it multiplies the cost of every map error by a 1 m radius.
- Whether NavFn's saturation at 18 poses for 0.7/0.8/0.9 m goals means those
  "OK" paths were truncated at the tolerance rather than genuinely reaching the
  goal. They were never driven.

### Next

Rebuild the map by driving a lap with the 3 m limit in force, then re-run the
distance bisection. If the median ESDF distance does not rise well above 0.35 m,
the map is still too noisy and `max_obstacle_distance` needs revisiting before
autonomy is possible at all.

**Read the costmap topic correctly when checking this.** nav2 publishes through
a translation table: raw 253 (INSCRIBED) → **99**, raw 254 (LETHAL) → **100**,
raw 1–252 → 1–98, unknown → −1. A first pass at this read 99 as ordinary
inflation and concluded the route was clear; it was blocked.

---
## ✅ 1. `/wheel_state` at 1.000 Hz — FIXED by the firmware flash, verified 2026-08-23

**Confirmed fixed after a power cycle on 2026-08-23:** `/wheel_state` sustained
**20.004 Hz** over a 369-sample window, `/wheel_ticks` 20.5 Hz, `/cmd_vel`
subscription count 1. Sustained, not a burst — max inter-arrival 0.107 s, so
nothing like the old 1000.1 ms ± 2.9 ms metronome.

The fix was the firmware change that stops `rclc_executor_spin_some` blocking
(timeout 0), described below. This entry stayed red long after the flash landed
because nothing re-checked it; the diagnosis underneath is preserved because the
reasoning is the useful part.

**This unblocks §2** — the drift gate could not pass without wheel data at rate.
That gate has still never been run.

### Original diagnosis (kept — the method is the point)



**The cause is the best-effort output stream on the ESP32, not the firmware
logic and not the network.** Diagnosed 2026-08-15 afternoon. Several earlier
theories in this file were wrong and are recorded as dead at the bottom.

### What is ruled out, and by what

| ruled out | evidence |
|---|---|
| the flashed binary not matching source | the sweep below reproduces exactly what the source predicts once you account for the executor blocking |
| a second ESP32 | agent log names its client: `session established … address: 192.168.1.3:47138` |
| WiFi / power-save | ping to `.3` is 2.4–10 ms, −36 dBm, 866 Mbit/s |
| the Pi 5 → Jetson DDS hop | same best-effort listener, same instant: Pi 5 **1.000 Hz ±2.9 ms**, Jetson **1.000 Hz ±38.7 ms** |
| packet loss | ±2.9 ms on a 1000.1 ms gap is a timer; random loss cannot be that regular |
| burst-then-idle buffering | zero inter-arrival gaps under 100 ms — it is genuinely one message per second |
| dead encoders | see §10 — both sides verified by hand |

### What it actually is

**`loop()` runs once per inbound message, or once per second if none arrives.**
`rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5))` blocks until data is
available rather than returning after its 5 ms timeout. The telemetry publish
sits immediately after that call, so it can only fire as often as the call
returns.

Proven by sweeping the rate we publish TO the board, 12 s per step:

| `/cmd_vel` out | `/wheel_state` in | ratio |
|---|---|---|
| silent | 1.00 Hz | — |
| 2 Hz | 3.13 Hz | 1.56 |
| 5 Hz | 5.08 Hz | 1.02 |
| 10 Hz | 10.08 Hz | 1.01 |
| 20 Hz | 11.76 Hz | 0.59 |
| 40 Hz | 16.44 Hz | 0.41 |

It tracks 1:1 up to 10 Hz then saturates. The 2 Hz row is the giveaway: 2 from
arriving messages plus ~1 from the idle timeout is exactly the 3.13 observed.

### Workaround, in place now, no flash

Publish zero `/cmd_vel` at 40 Hz and the board keeps turning. `logs/keepalive.py`
does this and `./rover compare` starts it automatically. Measured **17.1 Hz**,
gaps 58 ms against the intended 50, no gap over 800 ms. `./rover wheels` passes
at 17.0 Hz.

All-zero commands cannot cause motion — `pidStep()` returns 0 for both sides —
they only keep `lastCmdMs` fresh.

**Never run the keepalive while teleoperating.** It publishes to `/cmd_vel`, so
it would interleave with real commands and make the rover stutter. Hand-pushed
measurement runs only.

### Proper fix, still to do

Stop `loop()` blocking on the executor. Options, cheapest first:

1. `rclc_executor_spin_some(&executor, 0)` — non-blocking poll, so `loop()` free-runs
   at its `delay(1)` rate and the 50 ms timer fires properly. One line, but
   untested: if the timeout is being ignored entirely, zero may block too.
2. Set a spin period on the executor (`rclc_executor_set_timeout`) explicitly.
3. Move telemetry into `controlTask`, which provably runs at 50 Hz on its own
   core. **Risky** — micro-ROS sessions are not thread-safe, so this needs the
   publish handed to `loop()` rather than called from the task.

Do this next time the board is being flashed anyway; the workaround holds until
autonomous driving, where nav2's own `/cmd_vel` stream keeps `loop()` fed.

### Confirmed not the cause

Reliable QoS was flashed on 2026-08-15 (`e580d5e`) on the theory that
best-effort messages sat unflushed in an output stream. `ros2 topic info -v`
confirms the publisher is now `RELIABLE` and the rate was **still exactly
1.000 Hz**. The theory was wrong. The change is harmless and has been kept, but
it is not the fix.

That flash also invalidated the round-trip probe that appeared to show `loop()`
consuming 19.71 cmd/s with zero lag: micro-ROS keeps a shallow input queue that
retains the newest message, so "lag 0" appears no matter how slowly `loop()`
runs. **The sweep above is the measurement to trust**, because it varies the
input rate instead of assuming queue semantics.

### Dead theories, kept so they are not re-litigated

- ~~"the flashed binary is not built from this source"~~ — the source predicts
  the observed behaviour exactly once the executor block is accounted for. Cost
  two pointless reflashes; the lesson is that "the binary must be wrong" is what
  you reach for when the real mechanism is in a library you did not read.
- ~~"`loop()` is blocked by `ArduinoOTA.handle()` or the WiFi stack"~~ — right
  that `loop()` was blocked, wrong about where.
- ~~"`controlTask` (prio 2, core 1) starves `loopTask` (prio 1, core 1)"~~ —
  plausible on inspection, but it blocks on `vTaskDelayUntil(20 ms)`, and
  starvation cannot explain the rate tracking inbound traffic 1:1.
- ~~"best-effort messages sit unflushed in an output stream"~~ — flashed
  reliable, publisher confirms `RELIABLE`, rate unchanged at 1.000 Hz.
- ~~"rate at the Pi 5 is 1 Hz, measured with `ros2 topic hz`"~~ — that tool
  defaulted to **reliable** QoS, incompatible with what was then a best-effort
  publisher, and silently received nothing. The number happened to be right;
  the method was not. Now moot (the publisher is reliable), but measure with
  `qos_profile_sensor_data` regardless.

**Consequence at 1 Hz, if the keepalive is ever not running:** the wheels are a
coarse sanity check, not a reference. `compare.py` integrates them trapezoidally
across gaps up to `WHEEL_MAX_DT`, but the firmware reports *instantaneous*
velocity measured over one 20 ms control period, so at 1 Hz we point-sample a
signal that updates 50× faster. At 17 Hz that objection largely goes away.

---

## ✅ 20. First autonomous goals driven — and the controller fixed after the first one

**2026-08-22: the rover navigated to a goal on its own for the first time.**

Goal 1 m straight ahead. It planned, checked, drove, and stopped 7.8 cm from the
goal against a 15 cm tolerance. Tape-confirmed: odometry read 95.8 cm of
displacement, measured on the floor as 93–95 cm.

### The first run got there badly

| symptom | measured |
|---|---|
| overshot the goal | out to 113 cm, then back to 95 |
| hunted around it | ~13 s of a 6 s journey |
| crawled | 0.05 m/s instead of 0.18 |
| turned at the limit | 1.00 rad/s, the velocity_smoother cap |
| ended rotated | −23 deg, having been asked to hold heading |
| declared success while still turning | last command vx 0.00, **wz +0.90** |

Two causes, both leftovers from the pivot fault:

- **`use_rotate_to_heading: false`** — the NO-PIVOT setting. With heading
  correctable only by driving an arc, the rover was buying heading with forward
  motion, because that was the only currency it had. The pivot fault is fixed
  and verified on the floor (§14), so the reason had expired.
- **`approach_velocity_scaling_dist: 0.6`** — it begins decelerating 60 cm out,
  which on a 1 m goal is most of the journey. Now 0.25.

Also `yaw_goal_tolerance` 0.5 → 0.25 rad. At 0.5 (29 deg) the goal was declared
reached while 23 deg off; heading can now be corrected in place, so the loose
tolerance is no longer needed to avoid a deadlock.

### Second run, same 1 m goal, after the fix

| | before | after |
|---|---|---|
| time | ~30 s | **4.9 s** |
| overshoot | to 113 cm then back | **none** |
| cruise speed | 0.05 m/s | **0.18 m/s throughout** |
| heading change | −23 deg | **+2.3 deg** |
| peak turn rate | 1.00 rad/s (at cap) | 0.26 rad/s |
| cmd_vel messages | 597 | 100 |
| stop | still commanding wz 0.90 | **clean, topic released** |

`rotate_to_heading_angular_vel` is 0.8 rad/s, kept under the 1.13 rad/s
(65 deg/s) the rover actually achieved on the floor, so a commanded in-place
rotation is one it can execute.

### Watch on the next runs

It stopped **14.7 cm** from the goal, just inside the 15 cm tolerance, having
travelled 85.7 cm of the 1.00 m asked. That is the goal checker firing as soon
as it is close enough, which is correct behaviour — but it means short goals
land systematically short. Worth watching whether it matters for §6's obstacle
test; tightening `xy_goal_tolerance` below the ~7 cm pose error would just make
it chase noise.

---

## ✅ 19. The costmap did not reflect its ESDF — FIXED, it was a double gradient

Step 1 of the autonomy road **passed**. Driving a lap with the 3 m integration
limit improved the map on every gate:

| | before the lap | after | wanted |
|---|---|---|---|
| median ESDF clearance | 0.21 m | **0.46 m** | above 0.35 |
| cells further than 1 m | 3% | **21%** | above 15% |
| wall as % of floor | 46% | **9%** | under 10% |

It also beats the pre-§17 map (0.35 m median, 7.4 m² of wall). The map is not
the problem any more.

**nav2 still cannot plan anywhere.** Every goal from 0.5 m to 3.0 m fails with
NO_VALID_PATH, in every direction — worse than before, when 0.3–0.9 m worked.

### The costmap disagrees with the ESDF it is derived from

Read at identical world points, 5 cm cells around the rover:

```
          costmap        ESDF clearance
 y ≥ -0.05    all 99       0.45 – 0.53 m
 y ≤ -0.10    all  0       0.45 – 0.61 m
```

ESDF 0.45 reads FREE, ESDF 0.47 reads INSCRIBED. The boundary is a sharp
axis-aligned line, not a distance contour — so the cost is **not a function of
the ESDF value at that cell**, which is the only thing that layer is supposed
to compute.

The layer is correctly wired: both costmaps subscribe to
`/nvblox_node/static_map_slice`, and the log confirms
`Name: nvblox_layer  Topic name: /nvblox_node/static_map_slice  Max obstacle
distance: 1`. The costmap is live, not frozen — 146 cells changed over 171 s
with the rover parked.

### What was ruled out

- **Not the start cell alone.** The rover does stand in an INSCRIBED cell, which
  by itself fails every goal. But planning with `use_start: true` from a free
  cell 20 cm away fails identically.
- **Not unknown-space fragmentation.** `allow_unknown: true` changed nothing —
  though see the caveat below.
- **Not a stale costmap.** It updates.

### The caveat that undermines two of those

**Runtime `ros2 param set` on costmap layers appears not to take effect.**
Disabling `inflation_layer.enabled` returned "Set parameter successful" and
produced a **byte-identical** costmap — the same init-only trap already
documented for nvblox's slice heights. So the inflation and allow_unknown tests
are inconclusive, not negative. Re-run both by editing the YAML and restarting.

### The cause: two layers both computing a distance falloff

`libnvblox_nav2.so` exports `nav2_costmap_2d::CostmapLayer::updateWithMax`. That
merge takes the per-cell **maximum** of the layer and the master grid, so it can
raise a cost but never lower one.

In gradient mode the nvblox layer writes a graduated cost to every cell within
`max_obstacle_distance` of an obstacle. Merged by max, those costs accumulated
and never released — which is exactly a sharp stale boundary rather than a
distance contour, and exactly why a better map did not produce a better costmap.

**`convert_to_binary_costmap: true` fixes it.** The layer then marks LETHAL only
where the ESDF distance is ≤ 0 — actually inside an obstacle — and FREE
everywhere else. Nothing accumulates because there is no gradient.

Immediately after the change, every goal planned:

| goal | before | after |
|---|---|---|
| 0.5 m | NO_VALID_PATH | OK, 25 poses |
| 1.0 m | NO_VALID_PATH | OK, 47 poses |
| 2.0 m | NO_VALID_PATH | OK, 91 poses |
| 3.0 m | NO_VALID_PATH | OK, 203 poses |
| 1.5 m in all four directions | NO_VALID_PATH | OK |

Path length now grows with distance instead of saturating at 18 poses, which
was the §18 gate.

And it is correct, not merely permissive. ESDF and costmap now agree:

```
clearance 0.47, 0.45, 0.46, 0.54 m  ->  0   FREE
clearance 0.15, 0.10 m              ->  99  INSCRIBED
clearance −0.10 m (inside)          ->  99  INSCRIBED
```

**The safety margin was never this layer's job.** `inflation_layer` owns it with
`inflation_radius: 0.35`. Having both layers compute a falloff was
double-counting, and the one that merges by max is the one that could not clear
itself.

`max_obstacle_distance` was restored to 1.0: it is the ramp length in gradient
mode and inert in binary mode, verified by identical planning results at 0.4 and
1.0. Left at stock so that turning binary mode off returns to stock behaviour.

### The lesson worth keeping

Two wrong hypotheses came from inferring the cost formula from its outputs.
The answer came from `strings` on the plugin `.so` and reading which nav2 merge
function it links against. When a layer misbehaves, look at how it MERGES before
theorising about what it computes.

---

## ✅ 14. A pivot needed ~2× the duty the teleop was sending — FIXED, verified on the floor

**On the floor**, a pivot command drove instead of turning: counter-rotating in
**0%** of samples, the rover reversing along a slight curve. It cost a mapping
session — 10.7 m of "room loop" driven inside a 1.8 m box, because every attempt
to turn just drove the rover back and forth.

**Lifted on blocks, the wheels counter-rotate perfectly and symmetrically:**

| commanded | velL | velR | |
|---|---|---|---|
| `wz −2.00` | **+0.411** | **−0.407** | counter-rotating |
| `wz +2.00` | **−0.398** | **+0.398** | counter-rotating |

So the wiring, `R_MOTOR_DIR` and the reverse PWM path are all **correct**. The
electrical side does exactly what it is told. It is **traction**: four tyres
scrubbing sideways need more torque than the rover was being given.

### The cause: a units mismatch in the teleop

`rover_firmware_v2.ino` reads `wz` as **rad/s** and computes each wheel as
`vx ± wz × 0.34/2`. The teleop was written for an earlier firmware
(`rover_sim contract_bridge.py`) where `wz` was a **PWM fraction** — and its own
comment still claims `WZ = 2.0` gives "~100% PWM per wheel".

| WZ | wheel target | duty |
|---|---|---|
| **2.0** (was) | 0.34 m/s | **48%** |
| 4.0 | 0.68 m/s | 87% |
| **5.0** (now) | 0.85 m/s | **100%** |

Full authority is `2 × 0.86 / 0.34 = 5.06 rad/s`, where both wheels reach maximum
speed in opposite directions. `WZ` is now 5.0 and `WZ_SLIGHT` 4.25, scaled by the
same factor.

### Verified on the floor, 2026-08-22

| | counter-rotating | peak turn |
|---|---|---|
| LEFT | **93%** | 63.5 °/s |
| RIGHT | **96%** | 65.9 °/s |

Against **0%** before the change. The rover pivots.

**It is working hard to do it.** The wheels reach ~0.28 m/s of the 0.85
commanded and the body turns at ~65 °/s rather than the ~290 °/s full duty would
give unloaded — so scrub is absorbing roughly two-thirds of the torque. Fine for
driving; worth remembering when nav2 plans a tight turn, and it is why the
`NO-PIVOT` settings in `phase3/config/nav2.yaml` should be relaxed carefully
rather than all at once.

### Superseded Unloaded counter-rotation proves the
duty is now available; it does not prove it is enough to break the scrub. If it
still will not pivot, the remaining candidates are current sag (two motors share
one BTS7960 per side, and a pivot is the highest-current manoeuvre) or simply
too much grip for these motors — in which case the answer is a wider turning
radius rather than more duty.

---

## ✅ 13. Skid-steer scrub — measured, and it changes what the wheels are for

This rover has four driven wheels and no steering, so a turn drags every tyre
sideways. Two consequences, both measured 2026-08-21.

**The turning geometry is not the physical track.** `wz = (vR - vL) / W` needs an
effective width that includes the scrub:

| turn | wheels read | truth | ratio | implied width | peak rate |
|---|---|---|---|---|---|
| 90 left | 146.49 | 90 | 1.628 | 0.5534 m | — |
| 360 left | 556.49 | 360 | 1.546 | 0.5256 m | 21.4 °/s |
| 360 left | 548.34 | 360 | 1.523 | 0.5179 m | 76.2 °/s |
| 360 left | 553.08 | 360 | 1.536 | 0.5224 m | 74.3 °/s |
| 360 left | 551.49 | 360 | 1.532 | 0.5209 m | 75.1 °/s |

`WHEEL_BASE_ROT_M = 0.5216`, the mean of the four 360s. Using the physical
0.34 m made the wheels **63% wrong on every turn**. Note the faster turns
over-read slightly less — scrub is not a constant, so re-measure on carpet.

**The wheels only agree with each other in a straight line.** Per-wheel counts,
same side, same BTS7960, mechanically obliged to sweep the same arc:

| | LEFT front/rear | RIGHT front/rear |
|---|---|---|
| straight (200 cm) | 1.00× | 1.03× |
| turning (360°) | **1.60×** | **1.29×** |

Repeated on the next run: LEFT 37.5%, RIGHT 23.0% against 37.6% and 22.5%. The
scrub is **reproducible to half a percent**, so it is a deterministic property of
this chassis and loading, not random slip. That is what makes it safe to freeze
the calibration through turns rather than trying to filter it.

So encoder distance is an excellent reference straight and a poor one mid-turn.
`compare.py` now **freezes the encoder/cuVSLAM scale calibration while turning**,
because folding those samples in would drag a good calibration off with scrub
that is not travel at all.

**Design consequence:** heading comes from the gyro (−0.2% to −0.9% across four
turns), distance from the encoders while straight, and neither trusts the wheels
to measure a rotation.

---

## ✅ 12. Yaw sign verified against REP-103 (2026-08-21)

A LEFT teleop command produces a **positive** yaw rate, as REP-103 requires.
Checked with `logs/check_yaw_sign.py` because it cannot be read off the source —
it depends on how the IMU is bolted in and how `gyro_node` re-frames it. An
inverted sign would have made nav2 steer away from every goal, and would not
have shown up until the rover was driving itself.

This also resolved an ambiguity: a 360° run reported as clockwise read
`+361.53°`. The convention being correct means it was a mislabelled left turn,
not a sign bug.

---

## ✅ 11. cuVSLAM under-reads distance by ~2.2%, consistently

Four independent measurements against a tape:

| pushed | cuVSLAM read | error |
|---|---|---|
| 200 cm | 197.3 cm | −1.35% |
| 200 cm | 195.1 cm | −2.45% |
| 120 cm | 116.9 cm | −2.6% |
| 200 cm | 195.4 cm | −2.3% |

That is a **systematic scale error, not noise** — it is the same sign and
roughly the same size every time. It passes the ±5% scale gate, so Phase 1 does
not care, but over a long route it compounds: 100 m of driving is 2.2 m short.

A constant scale factor points at the stereo baseline, which is what converts
disparity into metres. `−P[3]/P[0]` gives 9.49 cm; a true baseline of
9.49 × 1.022 = **9.70 cm** would remove the error exactly. Worth checking against
the physical lens spacing before anyone hard-codes a fudge factor.

The encoders are now the better distance reference (§10), so FUSED takes
magnitude from them and direction from cuVSLAM, which removes this from the
fused pose without touching the driver.

---

## ✅ 10. Encoders — all four verified good AND calibrated (2026-08-15)

`ENCODER_CPR = 1560` is **correct**. Measured against a 200 cm tape push:

| wheel | counts | implied CPR | vs configured |
|---|---|---|---|
| LF | 11434 | 1526.6 | 0.98× |
| LR | 11378 | 1519.2 | 0.97× |
| RF | 11623 | 1551.9 | 0.99× |
| RR | 11275 | 1505.4 | 0.97× |

All four within 3% of each other and of the configured value. No firmware
change needed. `logs/calibrate_encoders.py <cm>` repeats it.

**Two false alarms got here first, both mine, both worth remembering:**

1. *"LEFT encoders are dead"* — the four-wheel test printed its prompts through
   a buffered pipe, so the operator never saw them and nothing was spun at the
   right moment. A test that needs the human and the script to agree on WHEN is
   fragile; `logs/wheels_selfpaced.py` asks only for a total instead.
2. *"Encoders are 2× out and the rear reads 25% more than the front"* — that
   compared cumulative counts, which measure **arc length**, against cuVSLAM's
   `straight`, which is **displacement**. On a push that curves or doubles back
   those are different quantities. Calibrate on a straight forward push against
   a tape, and against nothing else.


### The earlier, narrower check that led here

Before the tape calibration above, each side was verified in isolation by
hand. Kept because it rules out something the calibration does not: crosstalk.

Rover lifted, each wheel spun by hand in isolation, watching `/wheel_state`
(`x = velL`, `y = velR`, computed in the 50 Hz control task on its own core):

| spun | velL | velR |
|---|---|---|
| LEFT wheel, right held still | **14 of 15 samples non-zero, peak 0.068** | 0.000 throughout |

Left drives the left channel, right drives the right, correct sign, no
crosstalk. So the encoders, `ENC_*_DIR`, `METRES_PER_COUNT` and the control task
are all sound, and §1 is purely a transport problem.

**Beware a false negative here.** A first attempt reported "LEFT encoder: NO
SIGNAL" simply because samples arrive once a second and the test's phase
boundaries did not line up with which wheel was being spun. Any hand test on
this rig must name one wheel and hold the others still —
`logs/spin_one.py` does that.

---

---

## ⚪ 9. The Pi 5 has no RTC battery

At boot its clock resumes at its last known value, so `systemctl status` reports
service start times hours or days wrong until NTP corrects it. Use `uptime -s`.
This nearly caused a misdiagnosis: a service that had started 40 seconds earlier
appeared to be "3 days old".
