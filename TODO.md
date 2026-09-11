# TODO — known problems, in one place

Status: 🔴 blocks a gate · 🟠 real, worked around · 🟡 unverified · ⚪ accepted

One problem per entry, in the order they were found. For the **cross-cutting**
view — each subsystem's measured values against what is missing, and which of
these actually block unattended operation — see **[READINESS.md](READINESS.md)**.

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

> **Fourth occurrence, 2026-08-23**, found the instant `./rover status` learned
> to check for it: `vo_z +7.232 m`, 117026 rejections, 17.84 m dead-reckoned —
> while every rate in the table above it read OK. It was diverged during the
> autonomous goal runs the same afternoon, which is a candidate explanation for
> the heading errors seen there and was invisible at the time.
>
> Signs so far: −21.7 m, +87.7 m, −40.1 m, +7.2 m. Both directions, no pattern
> in magnitude, `planar_constraints` set throughout.
>
> `./rover status` now reports it and says how to recover. That is detection,
> not a fix.

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
- ~~**Surface it.**~~ **DONE 2026-09-09.** `vo_implausible` climbing is now loud
  where it matters. `./rover status` already ran `health.py`; the gap was that
  the *bring-up path* did not, so the check lived only in a doc telling you to
  run it by hand after every start — which is how it got skipped. Now:
  - `./rover fused` runs `health.py` as part of its gate, because that is the
    layer that starts publishing `/odom` and so the layer that has to say
    whether `/odom` means anything. On divergence it warns and keeps going —
    the layer *is* up, it just cannot be trusted.
  - `./rover map` **refuses to start** on a dishonest pose. This is the
    irreversible one: nvblox integrates depth at whatever pose it is handed, so
    a diverged pose does not degrade the map, it corrupts it, and the only
    recovery is discarding the map and redriving the lap.
- ~~**The red line in `compare.py`.**~~ **DONE 2026-09-09.** `compare` is the
  instrument you are actually watching during a push, and it was reading
  `/vo/status` — which carries the landmark count but **not** `vo_z`,
  `vo_implausible` or `dr_metres`. Those live on `/fusion/status`. That is why
  the Phase 1 instrument stayed quiet through all four divergences. It now
  subscribes to both and prints a red banner live, and again in the verdict
  above the grade, latched so a run that diverged *and then recovered* is still
  not reported clean. `VO_Z_LIMIT` is defined once at 0.30 m so the instrument
  and `health.py` cannot drift apart. Divergence is detected on absolute z **or**
  a climbing rejection counter — the counter catches it before z has moved far
  enough to trip, and a steady non-zero counter is treated as history, not a
  fault. Verified against all four recorded signatures (−21.7, +87.7, −40.1,
  +7.2 m), the climbing case, the steady case, the latch, and a malformed
  payload.
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

## ✅ 22. The divergence guard measured FORWARD TRAVEL and called it height — FIXED 2026-08-26

> **This is not §21.** §21 is a real cuVSLAM divergence and is still open. This
> was the *recovery* firing on healthy tracking and doing the damage itself.

The operator reported the RViz track coming back **beside** the outbound leg on
the same path. It was not drift. It was the odom origin being reset roughly
every 50 cm of driving.

`vo_node`'s auto-recovery tested `wfr.translation[2]` — the **optical** z, taken
before the frame conjugation. `base_from_optical` maps optical **+z to base +x**:

| optical | base |
|---|---|
| +x (right) | −y |
| +y (down) | **−z ← height lives here** |
| +z (forward) | **+x ← the guard was reading this** |

So the guard read forward travel and called it altitude. Every ~50 cm tripped
`VO_MAX_Z_ODOM = 0.50`, and after the 3 s grace it re-created the tracker —
which **resets the odometry origin**, breaks odom continuity, and invalidates the
map. The recovery for a silent failure was itself a silent failure.

Measured 2026-08-26, one session, rover driving normally:

| | |
|---|---|
| tracker rebuilds | **15** |
| reported "z" at trip | ±0.50, ±0.51 — i.e. half a metre driven |
| landmarks at those trips | 197, 174, 167, 159, 152, 142 — **healthy** |
| real base-frame height, throughout | **0.038 m** |
| fusion `jumps` / `dr_metres` | 85 / 1.76 m |
| GPU / thermals | 3–32 %, 53 °C — not a resource problem |

The tell was that `/fusion/status` and `/vo/status` disagreed. Fusion's `vo_z`
reads the **published** `/vo/odom` z, which is correctly conjugated, and sat at
0.002–0.038 m the whole time. Only `vo_node`'s own guard saw metres.

**Fix:** conjugate into `base_link` *before* the guard and test `T[2,3]`. The
pose matrix was already being computed twenty lines further down; it just moved
up, so the guard and the published pose now reason about the same axis.

**Why nothing caught it:** `self_test()` covered the conjugation thoroughly —
six cases, including mount yaw against two real pushes — but never the guard.
`VO_MAX_Z_ODOM` appeared exactly twice in the file: its definition and that one
comparison. Test 7 now asserts that 4 m forward does **not** trip the guard and
4 m of altitude does.

Genuine divergence (§21) is unaffected and still open — this only stops the
recovery firing when nothing is wrong.

**Verified on the floor the same evening.** Driving the room after the fix:
resets fell from **15 to 3**, and the three that remain are the honest kind —
they tripped at 2, 4, 12, 13 and 14 landmarks, i.e. texture collapse (§3), not
at healthy 197. The false-positive class is gone.

---

## ✅ 23. Loop closure was being thrown away over a vertical it cannot have — FIXED 2026-08-26

The complaint that started this: driving a lap and coming back along the same
path drew a **second line beside the first** in RViz. §22 was most of it. This
is the rest, and it is why the two lines never converged.

`_publish_correction` rejected the whole `map -> odom` correction if its z
exceeded `MAX_CORRECTION_Z = 0.30`. Measured 2026-08-26 over one room drive:

| | |
|---|---|
| corrections accepted | 15 |
| corrections **rejected** | **291** |
| shape of every rejection | 0.4–0.5 m total, **z = ±0.4 m** |
| `MAX_CORRECTION_M` | 5.0 — never the binding constraint |

So all 291 failed on the vertical alone, and the x/y/**yaw** part went in the bin
with it. That planar part is exactly what pulls a return leg back onto its
outbound one, which is why loop closure was running (15 closures) and visibly
achieving nothing.

**Fix:** flatten the correction onto the floor *before* judging it — z to 0,
rotation reduced to yaw — then test the **planar** magnitude. This rover has
three degrees of freedom; a correction's z, roll and pitch are the optimiser's
drift, not information.

**It does not weaken the guard.** The 2026-08-22 divergence corrected
`[-10.5, -22.2, 87.7] m`; its planar magnitude is **24.6 m**, still far over
`MAX_CORRECTION_M`, so it is still rejected on horizontal evidence alone. The
vertical threshold was never what caught it. Verified against five cases
including that one and a planar-only blowup.

A large vertical is still the best early warning of optimiser divergence, so it
is now counted and logged as `correction_flattened` in `/vo/status` rather than
acted on.

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

---

## ✅ 24. Wheels went silent again — FIXED by a full power-cycle, verified 2026-09-06

Found while dry-running the plan for VLM-driven goals: "say 'go to the red
bottle', a VLM turns the color frame into coordinates, something publishes
`/goal_pose`, nav2 drives there." Enabled color on the D555
(`enable_color:=true`, `rgb_camera.color_profile:=424x240x15`) so the Pi 5 has
a frame to hand the VLM, then re-ran the whole stack to check nothing else
broke: cuVSLAM (28.6 Hz, 121 landmarks, 0 implausible), nvblox and nav2 all
came up clean — the new stream shares no topic with any of them.

Then published a 0.68 m / 0.15 m goal to `/goal_pose` to test the other half:
does a published topic actually move the rover? `bt_navigator` accepted it
instantly, `controller_server` issued real velocity commands (`linear.x 0.18`,
`angular.z -0.26`, replanning every cycle) — the nav2 plumbing this idea
depends on is correct. But the FUSED pose (cuVSLAM-backed, so this is ground
truth) never translated: x/y sat at 0.00/0.00 for the whole run, only yaw
drifted a couple of degrees. `controller_server` eventually hit "Failed to
make progress" and cycled into a retry loop; `bt_navigator` was killed to stop
it rather than let it spin uselessly.

`/wheel_state` was 0 Hz throughout — checked on **both** the Jetson and the
Pi 5, so it is not a DDS-domain quirk local to one machine. On the Pi 5,
`ros2 node list` shows only `/pi5_tts_node` — no ESP32 micro-ROS client node
exists. The board answers ping (192.168.1.3) and `MicroXRCEAgent` is running
on the Pi 5 (`:8888`), so the network and the agent process are both fine —
the micro-ROS session between them is what's missing.

**This is a regression, not a repeat of §1.** §1 was fixed and confirmed at
20.004 Hz on 2026-08-23. Sometime in the two weeks since, that session dropped
to nothing. Cause unexamined — the next step is the same one that fixed §1:
physical access to the board, starting with a plain power-cycle before
assuming a reflash is needed again.

**Blocks:** all driving, autonomous or VLM-triggered, until the ESP32 session
is re-established. nav2 itself is not the problem — it plans and commands
correctly. The wheels just do not turn.

**Fixed, same day.** All devices (Jetson, Pi 5, ESP32, camera) power-cycled
together. `/wheel_state` came back at ~20 Hz and `wheels_alive: true` on the
very next bring-up — no reflash needed, so whatever dropped the micro-ROS
session was recoverable by power alone. See §26 for the drive that confirms
it end to end.

---

## 🟡 25. `planner_server` segfaulted once on nav2 bring-up, self-recovered on retry (2026-09-06)

First `./rover nav` after the color change: `planner_server` died with exit
code -11 about 15 s after activating —

```
[ERROR] [planner_server-2]: process has died [pid 8517, exit code -11, ...]
[lifecycle_manager]: Have not received a heartbeat from planner_server.
[lifecycle_manager]: CRITICAL FAILURE: SERVER planner_server IS DOWN ...
```

— and the lifecycle manager tore down the rest of nav2 in response. Not
memory pressure: 2.6 GB available and the container at ~2.1 GB, both before
and after. A second `./rover nav` came up clean and both costmaps (global
0.8 Hz, local 1.7 Hz) stayed healthy for the rest of the session, including
through §24's drive test.

Only seen once so far, so this is logged unverified rather than diagnosed.
Leading guess is a nav2 lifecycle race at startup — the `nvblox_layer`
costmap plugin subscribing right as nvblox is still settling — but nothing
here confirms that. Worth checking for a pattern if `planner_server` dies
again on a future bring-up.

**Update 2026-09-06, later same day:** two more `./rover nav` bring-ups (after
the full power-cycle in §24, and again after killing `bt_navigator` in §26)
both came up clean. Three clean bring-ups against one crash — still not
enough to call this fixed, but no repeat yet.

---

## ✅ 26. First autonomous drive since the wheels came back — succeeded, after one stall (2026-09-06)

All devices power-cycled (Jetson, Pi 5, ESP32, D555). Full bring-up clean,
including §24's wheels and §25's nav2. Published a goal 0.7 m ahead:
`bt_navigator` accepted it, `controller_server` drove — real translation this
time, x: 0.00 → 0.42 → 0.53 m — but then stalled, oscillating between
0.53–0.58 m for ~25 s while yaw drifted 0° → −11.8° on a goal that needed no
turn. `controller_server` hit "Failed to make progress" and retried, stalling
the same way again. Killed `bt_navigator` to stop the loop and asked the user
to physically check the floor — **confirmed clear**, and nvblox's own map
agreed: no occupied cell anywhere in the corridor ahead (x 0.2–1.5 m,
y ±0.3 m). So not a mapped obstacle, and probably not an obstacle at all.

Relaunched nav2 and re-issued the remaining distance as a fresh goal (current
position → +0.7 m). This time it drove straight through with no stall —
`Reached the goal!` / `Goal succeeded` in ~7 s, arriving at x: 0.566 → 1.194 m
(0.63 of the 0.70 m requested, inside `general_goal_checker`'s tolerance).
`/fusion/status` right after: `vo_implausible: 0`, `jumps: 0`,
`dead_reckoned: 51` samples / `dr_metres: 0.068` — a brief, harmless VO
dropout covered by dead reckoning, not a divergence.

**Read together with §13 (skid-steer scrub) and §14 (pivot duty):** the yaw
drift during the stalled run, on a goal that was purely straight-line, looks
like the same per-side scrub already documented there rather than anything
new. Not reproduced on the second attempt, so logged as a one-off rather than
chased further — worth a second look if a future goal stalls the same way
(oscillating translation + unwanted yaw drift, no mapped obstacle).

---

## 🟠 27. Depth stream at 8.3 Hz, under its 10 Hz gate (2026-09-06)

`./rover status` during the fleet check:

```
camera  /camera/camera0/depth/image_rect_raw     8.3   want 10   LOW
camera  /camera/camera0/infra1/image_rect_raw   25.8   want 15   OK
```

IR is fine, so this is not the camera falling over — it is depth specifically.

**Nothing downstream is broken by it right now.** nvblox publishes at 4.8 Hz,
both costmaps pass, `/odom` is 20 Hz, cuVSLAM is tracking cleanly. Logged
because it is the first rate to sit under a gate in a while, and because of
what it feeds: `pixel_to_goal.py` reads `aligned_depth_to_color` to turn a
VLM pixel into a metric goal. A slow depth stream there means a staler depth
sample behind each goal, which is exactly the kind of thing that gets blamed
on the VLM later.

**Leading theory: the `align_depth.enable:=true` added the same day.** That
makes the driver reproject depth into the colour frame itself, which is real
per-frame work it was not doing before. Colour was also enabled that day
(`enable_color:=true`, 424x240x15). Both landed together, so neither is
isolated yet.

**Not yet done:** measure depth with `align_depth` off and colour still on, to
split the two. If alignment is the cost, the question is whether 8.3 Hz is
good enough for the VLM path (probably — goals are issued at human pace, not
at frame rate) or whether the colour profile should drop instead.

---

## 🔴 28. Voice cannot start — the Pi 5 has no audio hardware attached (2026-09-06)

Tried to bring up `pi5_voice_pkg`. **Everything in software passed:**

- `fleet_role.sh voice status` → `jetson voice: stopped`, no whisper/kokoro
  processes on the Jetson, so the "never run both" precondition is met.
- `SARVAM_API_KEY` and `SONIOX_API_KEY` both present in `~/ros2_ws/.env`.
- Config reads clean: `stt_provider: sarvam`, `tts_provider: sarvam_translate`,
  `wake_detector: openwakeword`, `wake_word: hey_jarvis`, `require_wake: true`.

**The hardware is the blocker.**

```
bluetoothctl info D6:AA:BB:59:EF:B6
    Paired: no    Bonded: no    Trusted: yes    Connected: no
bluetoothctl connect D6:AA:BB:59:EF:B6
    Failed to connect: org.bluez.Error.Failed br-connection-page-timeout
```

Two 15–18 s inquiry scans never saw the boAt Stone; it shows in
`bluetoothctl devices` only as a remembered pairing. **The Pi 5's own adapter
is healthy** and was ruled out: `Powered: yes`, `rfkill` neither soft nor hard
blocked, `hci0: UP RUNNING PSCAN`, `TX bytes:68432 errors:0` — those TX bytes
are connection pages going out unanswered. The Pi is calling; the Stone is not
picking up.

The documented wired fallback is not present either:

```
arecord -l                 ->  no capture hardware devices at all
pactl list short sinks     ->  auto_null       (PipeWire's placeholder)
```

So the Pi 5 currently has **neither a microphone nor a speaker**.

**`Paired: no` is the part that matters.** The bond is gone, not just the
connection — that is the post-reboot behaviour `PI5_VOICE.md` already
documents. `connect` cannot work against it because bluez holds no key, so
this needs a real re-pair with the speaker in **pairing mode** (not merely
powered on — powered on alone, it will chase its last host). Sequence:

```bash
bluetoothctl remove D6:AA:BB:59:EF:B6      # clear the stale record first
bluetoothctl --timeout 20 scan on | grep -i stone
bluetoothctl pair D6:AA:BB:59:EF:B6 && bluetoothctl trust D6:AA:BB:59:EF:B6 \
  && bluetoothctl connect D6:AA:BB:59:EF:B6
```

A USB headset in the Pi 5 skips all of it — `bt_audio.ensure()` falls back to
whatever device exists.

**The nodes were deliberately NOT launched.** `bt_audio.ensure()` never raises
by design; it logs a status dict and carries on with whatever device it had.
Against no device, both nodes would have come up looking green while being
completely deaf — a worse failure mode than not starting them.

**Two things to know before the first utterance, neither a fault:**
`tts_provider: sarvam_translate` means it answers in **Telugu** (set
`tts_provider: local` for English Kokoro); and the first turn can take
50–108 s cold (KV cache, or a stale HTTP connection to the Mac mini) against
~1.5 s warm — ask twice before concluding voice is broken.

**Telegram is unaffected and remains the working channel.**

---

## 🟡 29. SSH to the Pi 5 goes intermittently unresponsive (2026-09-06)

During the fleet check the first several `ssh rakhi24@192.168.1.16` sessions
ran fine, then every one hung and timed out — for several minutes — before
recovering on its own. Throughout the outage:

- `ping` succeeded (0% loss, though `rtt max` 51 ms with `mdev` 25 ms — jittery)
- TCP to port 22 **connected** — so sshd was listening and reachable
- the hang was after the TCP handshake, i.e. in the SSH session itself

Not diagnosed. **First suspect is Wi-Fi contention:** the Jetson reaches the
Pi 5 over `wlP1p1s0`, and that same radio is carrying the DDS traffic for the
whole stack — camera, VO, costmaps — while the stack is up. An SSH session is
small and latency-sensitive and would be the first thing to starve.

Worth caring about because remote bring-up of the Pi 5 (voice, the brain) goes
over exactly this link, and because it will look like "the Pi 5 crashed" when
it has not. If it recurs, worth checking whether the Jetson's wired interface
can carry the Pi 5 link instead of the Wi-Fi radio.

---

## ✅ 30. The cuVSLAM honesty gate reported "ok — tracking" while cuVSLAM was dead — FIXED, verified 2026-09-09

The check that exists specifically to catch a silent pose failure passed a pose
whose tracker was **not running at all**.

During bring-up, `vo_node` died after `./rover pose` had already gated green.
`./rover status` then printed both of these, in one output:

```
pose      /vo/odom                                    0.0     10 --
cuvslam   vo_z +0.014 m   implausible 0   dead-reckoned 0.00 m   landmarks 81
          ok — tracking, not dead reckoning
```

`ros2 node list` showed no cuVSLAM node. `ros2 topic hz /vo/odom` said
`does not appear to be published yet`. `./rover map` had already built on that
pose without refusing, which §3 of [STARTUP.md](STARTUP.md) says it will not do.

Only the contradiction between those two lines exposed it. Had the status sweep
sampled `/vo/odom` a second earlier or later, the whole table would have read OK.

### The cause: `vo_alive` is published, correct, and never read

`phase1/nodes/health.py` parses four fields out of `/fusion/status` — `vo_z`,
`vo_implausible`, `dr_metres`, `landmarks`. It never reads `vo_alive`.

The fusion node computes that field correctly and does publish it —
`phase1/nodes/fusion.py:377`:

```python
'vo_alive': self.vo_n > 0 and (now - self.vo_last_msg) <= COVER_STALE_S,
```

with `COVER_STALE_S = 0.5`, and `fusion_node.py:348` serialises the whole health
dict, so `vo_alive` is sitting in the very JSON payload `health.py` already
parses. The signal was present, correct, and unused.

### Why the four fields it does read cannot catch this

All four are last-known values that **freeze** when the publisher stops rather
than going stale-flagged or absent:

| field | on tracker death | why it stays quiet |
|---|---|---|
| `vo_z` | holds its final value | +0.014 m, well inside the 0.30 m limit |
| `landmarks` | holds its final count | 81 — above the 30 that would warn |
| `vo_implausible` | stops incrementing | `climbing` compares two frozen samples |
| `dr_metres` | **0.00** | the rover was stationary |

`dr_metres` is the one that would have moved, and it only moves once the rover
drives. **A stationary rover with a dead tracker is indistinguishable from a
stationary rover with a healthy one** on these four fields — which is exactly
the state every bring-up is in.

### How it was missed

`health.py`'s own docstring explains the silent fallback as one where
"`vo_alive` stays true". That is true of **divergence** — cuVSLAM keeps
publishing a confidently wrong pose, so freshness looks fine. It is false of
**death**, where publishing stops and `vo_alive` goes false within 0.5 s.

The check was written for the first failure mode and, in the process, dismissed
the one field that cleanly catches the second.

### The fix

`health.py` now reads `vo_alive` as its own failure, checked **before** the
divergence test — because once the tracker is dead, `vo_z` and
`vo_implausible` are frozen garbage and testing them first would report the
wrong fault.

It is sampled **across the window, not once**. A single false reading is the
~1 s dropout of §31, not a death, so:

| `vo_alive` across ~4 samples | verdict | rc |
|---|---|---|
| all false | `✗ NOT RUNNING` — frozen numbers, `./rover pose` THEN `./rover fused` | 1 |
| some false | `ok`, plus a dropout warning pointing at §31 | 0 |
| all true | `ok — tracking, not dead reckoning` | 0 |
| field absent | `ok`, plus a warning that this fusion_node cannot report it | 0 |

The low-landmark warning is now suppressed when the tracker is not alive — that
count is frozen too, and "drive where there is texture" is bad advice for a
process that is not running.

**Both gates inherit it**, because both propagate the exit code: `./rover fused`
prints the do-not-map warning (`rover:255`), and `./rover map` now **refuses**
(`rover:276`) on a dead tracker, which is exactly what it failed to do above.

### Verified

Against the live stack: `ok — tracking`, rc 0 — no regression. Then against a
synthetic `/fusion/status` on an isolated `ROS_DOMAIN_ID`, so the running stack
was never disturbed, all four rows of that table reproduced:

```
DEAD    (vo_alive 0)        ✗ NOT RUNNING — ... in any of 4 samples.    rc=1
FLICKER (vo_alive 1,1,0,1)  ok  + ! vo dropped out in 1 of 4 samples    rc=0
HEALTHY (vo_alive 1)        ok — tracking, not dead reckoning           rc=0
OLD     (field absent)      ok  + ! does not publish vo_alive           rc=0
```

### Follow-up: the latched red line quoted the wrong numbers (2026-09-09)

`compare.py` latches divergence on purpose — a push that diverged is not clean,
even if the tracker looks fine by the time you read the verdict. But
`_divergence_lines()` printed the **live** fields beside that latched verdict,
so once cuVSLAM was reset the red line refuted itself:

```
✗ cuVSLAM DIVERGED — vo_z +0.01 m, 0 poses rejected, 0.00 m dead reckoned.
```

A divergence warning quoting a z well inside the 0.30 m limit is one a reader
talks themselves out of, which defeats the point of latching it.

`vo_z` and `vo_implausible` are now snapshotted at the moment the latch trips
and quoted from there once it has recovered, with the second line saying so.
While it is *still* diverged the live values are quoted, because then they are
the truth. `dr_metres` stays live in both cases — it is cumulative, and the
distance carried open-loop does not un-happen.

```
still diverged  ✗ ... vo_z -1.40 m, 9 poses rejected, 5.00 m dead reckoned.
                  The pose is running open-loop on wheels+gyro. ...
recovered       ✗ ... vo_z -0.91 m, 7 poses rejected, 4.50 m dead reckoned.
                  Measured when it tripped. The live fields read clean now,
                  which does not make the run clean.
```

---

## 🟠 31. Frame gaps of 1.4–1.9 s under full load exceed cuVSLAM's `max_frame_delta_s` (2026-09-09)

With every layer up (camera, pose, fused, map, nav, vlm) on a Jetson at load
average ~4.2, the camera pipeline stalls long enough to starve `vo_node` past
the point where cuVSLAM resets tracking. From `/tmp/vo.log`, one session:

```
[WARN] [vo_node]: frame gap 1.92 s — past cuVSLAM max_frame_delta_s (1.0 s). Tracking may reset.
[WARN] [vo_node]: frame gap 1.40 s — past cuVSLAM max_frame_delta_s (1.0 s). Tracking may reset.
```

matched one-for-one, to the same second, by the fusion node:

```
[WARN] [fusion_node]: vo DOWN — carrying on with the rest
[INFO] [fusion_node]: all three sensors contributing
```

Each dropout recovered in ~1 s, so no rate gate ever saw it: `/odom` held 20 Hz
and `/vo/odom` averaged 23 Hz across the sweep.

**This revises the cost recorded in [STARTUP.md](STARTUP.md) §2.** That note has
the `vlm` layer taking depth from ~24 Hz to ~14 Hz, "worst case a 633 ms gap …
still over the 10 Hz gate". The rate gate is not the binding constraint —
`max_frame_delta_s` is, it sits at 1.0 s, and the gaps measured here are up to
**1.9 s, nearly double it**. A gap that clears the 10 Hz gate comfortably can
still reset the tracker.

Unproven but suspected: this is also what killed `vo_node` outright in §30. Its
log ends on `tracker up` with no traceback, no OOM in `dmesg`, and 2.7 GB free —
consistent with a tracking reset it did not survive rather than a crash.

Not yet diagnosed to a cause. The obvious suspects are consumer load on the
camera pipeline (`image_bridge` attaching a second consumer to the colour
stream) and CPU contention from nvblox + nav2. Worth measuring which, because
the workaround — not running `vlm` while mapping — costs the brain its eyes.

---

## 🟠 32. The Pi 5 brain asks this rover for four things it never answers (2026-09-06)

> **The two vision seams are closed as of 2026-09-09** — `/vision/detections_3d`,
> `/vision/target` and `/vision/target_result` are answered and verified on
> hardware. See "Closed here" below. The servo and music rows stay open, and
> the Nav2 rotation question below is still unmeasured.

Read both repos side by side as one system for the first time. The rover is
not the problem; the **seam** is. Four topics `pi5_ros2_ws`'s `ros2_bridge.py`
speaks have no counterpart here at all — not a stopped node, not a container
that is down: no code anywhere in this repo that publishes or subscribes them.

| topic | what it powers on the brain side |
|---|---|
| `/vision/detections_3d` | `approach_object`, `where_is`, `list_known_objects`, `scan_surroundings`'s object report, **the whole `world_model` service** |
| `/vision/target` + `/vision/target_result` | `navigate_to_visible_object` (the mono visual-servo fallback) |
| `/servo_pan`, `/servo_tilt` | `point_camera` — `rover_firmware_v2.ino` declares three subscriptions and none of them are servos, and there is no mount |
| `/audio/music_*` | every tool in `tools/music.py` |

This is the same shape as §21 and the `map`-frame fault: **it presents as
healthy.** Nothing errors. `approach_object("chair")` searches, finds nothing,
and answers *"I haven't seen one before either, so I have nowhere to go"* —
about a chair it is looking straight at.

The consequence worth acting on: **`approach_described_object` (VLM pixel →
`phase4/nodes/pixel_to_goal.py`) is the only working object-approach path on
this rover**, and it costs a VLM round-trip (~10–40 s) per look. A detector
publishing

```json
{"frame": "odom", "objects": [{"label": "chair", "x": 1.2, "y": -0.4, "z": 0.3, "conf": 0.8}]}
```

on `/vision/detections_3d` would light up five brain-side tools at once, at
roughly zero query cost. `"frame"` must be `odom` — the brain now drops (and
logs) anything else.

**Fixed here the same day:**

- `pixel_to_goal._on_query` called `lookup_transform(..., timeout=Duration(0.5))`
  **from inside the executor callback thread** — the same single thread the
  `TransformListener` needs in order to consume `/tf` and fill the buffer. It
  could not succeed: a guaranteed 0.5 s stall followed by the same failure.
  Latest-available (`Time()`) now, matching the `base_link` lookup right below
  it, which never had the timeout.
- `image_bridge` re-encoded 896×504 JPEGs at 5 Hz forever, whether or not
  anything was subscribed — on a box at 96–99 % GPU with 1.6 GB free
  (JETSON_LOAD.md), plus ~250–400 KB/s of Wi-Fi for an audience of zero. It
  checks `get_subscription_count()` first now.
- `teleop_web.py`'s docstring quoted caps (`vx<=0.22`, `wz<=0.90`) that its own
  constants and comment directly contradict — inherited from the open-loop
  firmware.

**Still open, and it needs one measurement here.** The brain's drive
calibration was written for the pre-`v2` open-loop firmware: it commanded
0.28 m/s while assuming the robot physically moved at 0.60 m/s, so every
`move_robot` distance came out **2.1× short** (`F:20` drove ~9 cm). Fixed on
that side. But it raises a question about *this* repo:

> `phase3/config/nav2.yaml`'s `velocity_smoother` caps angular at 1.5 rad/s.
> `phase1/teleop/teleop_web.py` measured that **2.0 rad/s cannot break this
> chassis's tyres loose sideways at all** — a pivot command becomes a
> forward/backward curve, which is how 10.7 m of "room loop" fitted inside a
> 1.8 m box on 2026-08-22. If that holds, **Nav2's in-place rotations are
> curving too**, and that would quietly explain heading error which survives a
> good position fix.
>
> One test settles it: command a pure rotation through Nav2 and watch whether
> the base translates.

### Closed here: the detector is back, in `odom` (2026-09-09)

The node was not missing, it was **stranded**. `orin-nav-stack/nodes/detections_3d.py`
— 459 lines, YOLOv8n + D555 depth + TF, already speaking all three vision
topics — was left behind in the superseded project when the layered stack was
built. `/tmp/detections_3d.log` in the container, dated July, was the clue.

Ported to `phase4/nodes/detections_3d.py` and wired in as **`./rover detect`**.
Almost all of the adaptation was configuration, not code — the node was written
against parameters:

| what | old stack | here | why |
|---|---|---|---|
| `target_frame` | `map` | **`odom`** | the brain drops any other frame. `ros2_bridge.NAV_FRAME` is `os.environ.get("LANGROBO_NAV_FRAME", "odom")` — read on the Pi 5, not assumed |
| `look_feed_rate` | 2.0 Hz | **0.0** | `image_bridge` (the `vlm` layer) owns `/camera/color/image_raw/compressed` now. Two publishers on one topic is the failure that reads as a flapping camera |
| model | `/models` in the old image | `models/` mounted `:ro` | weights are gitignored; `./rover detect` prints the one-line recovery |

Its TF handling needed nothing: it already takes capture-time TF and falls back
to latest-available with **no timeout**, and gives the `TransformListener` its
own thread (`spin_thread=True`). That is the bug §32 fixed in `pixel_to_goal`
above — this node never had it.

**Verified on hardware**, on the running stack:

```
GATE detect:
    detections_3d publishing       1        (want >= 1)   OK
    target hunt listening          1        (want >= 1)   OK
```

- 136 messages captured in 90 s, **every one `"frame": "odom"`**.
- Positions are steady: one object held x 2.51, y 1.21, z 1.10 across ten
  consecutive ticks, ~2 cm spread — the EMA in `_smooth()` doing its job.
- The visual-servo path answers without needing an object: setting
  `/vision/target` to `person` produced `{"target": "person", "found": false}`
  on `/vision/target_result` every tick at ~5 Hz, which is the contract (the
  brain reads silence there as a dead camera and halts).
- The node logged `YOLO active (demand)` with `Subscription count: 2` — the
  Pi 5 brain was already listening, across machines, with no change on that side.

Two honest caveats:

- **The labels are yolov8n-grade.** That capture was confidently 126×`knife` in
  a room with no knife in it, at `confidence 0.35`. The *geometry* is right and
  that is what `world_model` consumes, but "it can see" should not be read as
  "it can name". The 0.35 gate is deliberate and documented in the node — 0.45
  published nothing for objects filling the frame.
- **The gate is not a rate gate**, on purpose: the node publishes only when it
  sees something, and `on_detections` on the Pi 5 has no staleness timeout — it
  just observes into `world_model`. An empty room is not a fault, so the gate
  checks the node is up and listening and then reports what is in view.

**Not closed:** `/servo_pan` and `/servo_tilt` have no hardware to answer with —
`rover_firmware_v2.ino` declares three subscriptions and none are servos, and
there is no mount. `/audio/music_*` is Pi 5-side. Both are unchanged.

Full cross-repo writeup: **`pi5_ros2_ws/INTEGRATION_GAPS.md`**.

---

## 🟠 33. The ESP32 wheel link decayed to nothing mid-bring-up, then recovered on its own (2026-09-09)

The link came up healthy after the power cycle, **died during bring-up while
nothing was being driven, and came back by itself** with no power-cycle and no
intervention. Whole sequence measured on one bring-up:

| when | `/wheel_state` | `/cmd_vel` subs |
|---|---|---|
| after `./rover map` | 15.9 → 14.8 → 11.3 Hz, max gap 0.732 s | 1 |
| after `./rover vlm`, minutes later | "does not appear to be published yet" | 0 |
| confirming | Publisher count **0** | **0** |
| after `./rover status`, ~10 min later | **20.0 Hz**, 228-sample window, std dev 0.029 | **1** |

The three `hz` windows in row 1 are the interesting part: not a clean drop, a
**monotonic decay** across ~45 s before the session died outright.

### This is a transient, not the firmware-retry fault

Worth stating plainly because an earlier draft of this entry got it wrong and
said the rover could not drive and needed a physical power-cycle. **It did not.**
The documented ESP32 fault is that old firmware *never* retries the agent
connection — a link that is down stays down until the board is power-cycled.
This one restored its own micro-ROS session. So it is a **different failure**,
and power-cycling on sight of it is the wrong reflex: it would have "fixed"
something that was already fixing itself, and hidden the real behaviour.

### Why it still matters

The window is silent in the worst direction. nav2 keeps planning and publishing
`/cmd_vel` into a topic with **no subscriber**, every gate on the Jetson stays
green, and the rover simply never moves. That reads as a controller fault or as
teleop being in MANUAL, and it is neither. If a drive had been commanded in that
window it would have failed for a reason nothing on the Jetson reports.

`/cmd_vel` publisher count climbed across the same period — 2, then 6 — as nav2,
the brain and `/studio_bridge` attached. Whether that load on the DDS link is
*causal* or merely concurrent is **not established**, and should not be asserted
either way on one observation. It is the most obvious thing to test first.

### What is not known

- Whether it reproduces. **One observation.**
- What the recovery was triggered by, if anything — it was noticed at the next
  `./rover status`, so the true recovery time is somewhere in a ~10 min window.
- Whether DDS load from attaching publishers has anything to do with it.

### What was done

- `STARTUP.md` §4 gained a subsection: the §4 wheel check is a reading, not a
  guarantee — re-check **after the last layer is up** and again before driving.
- No code change, and **no power-cycle advice** — the correct response to seeing
  this is to re-check, not to reach for the board.

### Next time it happens

Capture the micro-ROS agent log *while it is down* — the session teardown and
re-establish reasons are the evidence this entry does not have. Do **not**
power-cycle first; that destroys the only interesting state.

**The agent runs on the Pi 5, not the Jetson** (found 2026-09-10 while looking
for something else): `langrobo-microros.service`, "LangRobo micro-ROS agent
(Pi5 <-> ESP32 WiFi UDP bridge)". So the log to read is

```bash
ssh 192.168.1.16 'journalctl -u langrobo-microros --since "10 min ago"'
```

That also reframes the whole entry: the link is **ESP32 -> Pi 5 over WiFi UDP**,
and `/wheel_state` only reaches the Jetson afterwards over the DDS discovery
server. A WiFi dropout on the Pi 5 side would produce exactly the decay-then-
recover signature observed, and would have nothing to do with Jetson DDS load or
with `/cmd_vel` publisher count. **That is now the first hypothesis to test**,
ahead of the publisher-count one above.

### It is REPRODUCIBLE, and driving is the trigger (2026-09-10)

Found while trying to calibrate the rotation constant (§36), which needed the
rover to pivot repeatedly. It is no longer a single bring-up observation.

**Idle it is fine. Under driving load it falls apart.** `/wheel_state`, measured
across one session, in the order the readings were taken:

| when | rate |
|---|---|
| after bring-up, idle | 19.99 Hz |
| idle, before driving | 18.8 Hz |
| during a 5 s pivot | 14.2 Hz |
| during three back-to-back pivots | **1 message in 3 s** |
| idle again, minutes later | 19.94 Hz |
| idle, confirmed | 19.86 Hz |

No intervention at any point. The recovery is the same self-healing already
recorded above; what is new is that **commanding motion is what breaks it.**

**Both directions of the link die together, which is the diagnostic.** During a
continuous 5 s pivot command, `/wheel_state` produced *no messages at all* for
1.5 s — not zeros, an absence — and the gyro confirms the body was not rotating
then either. Commands and telemetry share one micro-ROS WiFi UDP path, so a
single link stall stops the motors (firmware watchdog) and the telemetry at the
same instant. Nothing else explains both halves at once.

The visible signature at the robot is a **~1 s on, ~1 s off stutter**. Gyro rate
per 0.5 s bucket through one 5 s pivot, commanded continuously at 5.0 rad/s:

```
0.92  0.37  0.00  0.00  0.61  0.52  0.00  0.00  0.52  0.64
```

**It is not a current cliff.** Full duty was the obvious suspect — 5.0 rad/s
commands +/-0.85 m/s against `MAX_WHEEL_VEL` 0.86, and a pivot is the
highest-current manoeuvre this rover has. But re-running at **2.0 rad/s** was
*worse*, not better: the right side managed 11.7 deg in five seconds. A current
or thermal limit would ease off at lower duty. This does not.

**What this invalidates.** Any measurement taken while driving is suspect unless
`/wheel_state` continuity is checked for the same window. Three of the four
rotation measurements on 2026-09-10 had to be thrown away for this reason, and
one of them looked perfectly plausible (0.13 rad/s, 12% left/right asymmetry)
while being nothing but dropout.

**What to do next**, in order:

1. Capture `journalctl -u langrobo-microros` **during** a pivot — the trigger is
   now reproducible on demand, so the log that this entry has always lacked is
   finally obtainable. (Checked after the fact: the unit is `active` and logged
   nothing for the whole window, which is itself worth explaining.)
2. Watch the Pi 5's WiFi at the same time — signal, retries, tx errors. If the
   ESP32 browns out under motor current its WiFi drops first.
3. Only then consider firmware. A power-cycle still "fixes" it and still
   destroys the evidence.

**This outranks §36.** A calibration constant is worth nothing while the link
delivers commands intermittently, and it makes every motion measurement on this
rover untrustworthy until it is fixed.


---

## 🔴 34. Nav2 fails to reach anything — controller one run, planner the next (2026-09-10)

> **Explained 2026-09-11 — see §40.** Run A is RPP's collision sweep and the
> cost-regulation crawl; run B is the planner refusing a gap narrower than
> twice the inscribed radius. Both are fixed in config; neither is verified on
> the floor, so this stays 🔴 until §40's tests run.

Three consecutive `approach_described_object` attempts at a white bucket ~1.6 m
away. None arrived. **This is the thing actually blocking use of the robot** —
everything else fixed today was about being *told* what happened.

### Two different failures, not one

**Run A (01:09:44 → 01:10:19) — the controller.** The rover visibly drove up,
reversed, drove up, reversed, drove up, stopped:

```
follow_path Aborting ×4  →  Running backup  →  completed
follow_path Aborting ×2
follow_path Aborting ×2  →  Running backup  →  completed
follow_path Aborting ×4  →  Running backup  →  completed
bt_navigator: Goal failed
```

**Run B (01:31:03 → 01:31:10) — the planner.** No movement at all:

```
compute_path_to_pose Aborting ×6
behavior_server: Running backup → backup FAILED
bt_navigator: Goal failed
```

A had a path and could not follow it. B could not compute one. Run B was also
in MANUAL for part of it, which is now guarded — so B needs re-running before
being taken at face value.

### The hypothesis worth testing first

Nav2 here has **rotate-to-heading and spin recovery disabled**, on the recorded
grounds that this rover cannot turn in place (`./rover nav`, TODO 14). But
teleop's `left`/`right` **do** pivot it — `MOVES` counter-rotates the wheels,
and the comment there calls it "pivot in place (both wheels counter-rotate)".

If it can pivot, that config is a stale assumption, and it explains run A
exactly: the standoff goal needs a heading nav2 has been forbidden to turn to,
so `follow_path` aborts and `backup` runs instead, forever, until the BT gives
up. **Check whether the rover can still pivot before changing anything** — the
teleop comment is old too, and TODO 14 was presumably written for a reason.

For run B, suspect the goal pose itself: `_STANDOFF_M` is 0.45 m, and the map
is ~74% unknown. A goal inside the bucket's inflation radius, or in unknown
space, is unplannable and would abort exactly like that.

### Confounder now removed

Both runs happened with **LangGraph Studio running beside the brain**, two
graphs publishing `/cmd_vel` (see the Studio commit). Whether that contributed
is untested. Studio is stopped; the next run is the first clean measurement.

### Next

1. Re-run with Studio down and teleop in AUTO — clean baseline.
2. `./rover logs nav` for which of the two failures it is.
3. Test a hand pivot via teleop, then revisit the rotate-to-heading config.
4. If it is the planner, print the goal pose and check it against the costmap.

---

## ✅ 35. `./rover view` reported "nobody is logged in" at a laptop that was logged in — FIXED, verified 2026-09-10

After a power cycle, every Jetson layer came up green and `./rover view` ended
the bring-up with:

```
  found at 192.168.1.17
  config synced from the repo
  ✗ nobody is logged in at the laptop desktop.
    RViz cannot render into a login screen. Log in there, then re-run.
```

Someone **was** logged in, at the laptop, at its own screen. Re-running changed
nothing, and no amount of logging in would have — the advice the launcher gives
in this case cannot fix this case.

### The cause: Xwayland running is not what "logged in" means

The check read one thing:

```bash
pgrep -a Xwayland   # empty -> "nobody is logged in", exit 1
```

`loginctl` on the same laptop, at the same moment, disagreed:

```
SESSION  UID USER    SEAT  TTY  STATE  IDLE
     14 1000 rakhi24 -     -    active no
      2 1000 rakhi24 seat0 tty2 active no     <- Type=wayland, Active=yes
```

GNOME Shell was running (PID 2837). **GNOME starts Xwayland on demand.** Log in,
open nothing that speaks X11, and there is no Xwayland process — not because
there is no session, but because nothing has asked for one yet. The launcher was
testing for an X11 *client having already run*, and calling its absence a login
screen.

The evidence that a session was waiting was sitting in the runtime dir, written
at login:

```
/run/user/1000/.mutter-Xwaylandauth.QY3XV3     (11:11, cookie only, no process)
```

That cookie is exactly what mutter prepares **in advance** for the on-demand
path. Launching `~/rover_live.sh` with `DISPLAY=:0` and that `XAUTHORITY` brought
RViz straight up and spawned Xwayland as a side effect:

```
4626 /usr/bin/Xwayland :0 -rootless -noreset -accessx -core \
     -auth /run/user/1000/.mutter-Xwaylandauth.QY3XV3 ...
```

### Why the original check was written that way

It is not arbitrary, and the comment above it explains itself: on Wayland
`loginctl show-session -p Display` is **empty**, so the obvious source for the
display number is gone, and Xwayland's own cmdline carries both the display
number and the auth cookie. That reasoning is sound — the mistake was using a
*convenient source of two values* as the *test for a precondition*. When
Xwayland runs, it is still the best source. It was never evidence of a login.

### The fix

`rover`'s `view` now keeps Xwayland as the preferred source and falls back when
it is absent, instead of concluding failure:

1. `pgrep -a Xwayland` — if running, parse display and auth from it, as before.
2. Otherwise ask the two questions that actually matter: does `loginctl` show a
   session with `Active=yes` and `Type` of `wayland` or `x11`, and is there a
   `.mutter-Xwaylandauth.*` cookie in the runtime dir? If yes, use `:0` and that
   cookie, and say plainly that Xwayland is not started yet.
3. Only with no active graphical session does it print "nobody is logged in".

An `x11` session leaves no mutter cookie, so `XAUTHORITY` falls back to
`$HOME/.Xauthority` — an empty `XAUTHORITY` is worse than an absent one.

### Verified

Tested from the real failing state, not a simulation: `rviz2` and `Xwayland`
were both killed on the laptop, returning it to the exact condition that
produced the false negative (`rviz2: 0  Xwayland: 0`). Then:

```
  Xwayland not started yet — GNOME starts it on demand; starting rviz2
  is what spawns it. Someone IS logged in; loginctl says so.
  display :0
  started
  subscriptions (0 means it is connected to nothing):
      /rover/model                               1
      /fusion/path                               1
      /nvblox_node/static_occupancy_grid         1
```

Running **and** receiving. The primary path was re-checked with Xwayland alive
and still reports `display :0` / `already running`.

### The shape of this one is worth keeping

This is the third entry in this file where a **check** was wrong rather than the
thing it checked (see §30, where the cuVSLAM honesty gate passed a dead
tracker). Both failed the same way: a proxy signal was treated as the fact
itself, and the proxy was chosen because it was easy to read. §30 froze; this
one was never true in the first place. When a gate accuses hardware that looks
fine, suspect the gate.

---

## 🟠 36. Timed turns are computed against a yaw rate the rover never reaches (2026-09-10)

Ask for 90°, 180° or 360° and the rover turns a small fraction of it. Reported
from normal use: "when I asked to rotate 90 or 180 or 360 it looks like it is
rotating only near to 90 degrees."

`langrobo_core/tools/movement.py` drives every turn open-loop:

```python
duration = radians(angle) / _STEADY_STATE_ANGULAR_VEL
```

Two constants exist, correctly separated:

| constant | meaning | value |
|---|---|---|
| `_ANGULAR_VEL_RS` | yaw rate **commanded** in the Twist | 5.0 rad/s |
| `_STEADY_STATE_ANGULAR_VEL` | yaw rate **achieved** | **defaults to `_ANGULAR_VEL_RS`** |

The second defaulting to the first is the bug. Its own comment says "MEASURE IT
and set `LANGROBO_STEADY_ANGULAR_VEL`" — and it never was. There was no
`~/.langrobo/brain.env` on the Pi 5 at all, so nothing overrode it.

Commanded 5.0 rad/s is 286 °/s. Achieved is ~69 °/s. So every turn duration is
**~4.2× too short**:

| asked | duration computed | actual |
|---|---|---|
| 90° | 0.314 s | ~21° |
| 180° | 0.628 s | ~43° |
| 360° | 1.257 s | **~86°** |

That last row is the reported symptom, to within a few degrees.

### The consequence nobody had noticed

`approach_described_object`'s search calls `_duration("L", 90.0)` per step and
runs five steps, which its docstring calls "a full circle plus one recheck".
At the real rate each step turns ~21°, so the sweep covers **~105°, not 450°**.

**Anything behind the robot has never been searched**, and the failure message —
"I turned a full circle and looked carefully, but I couldn't spot the ... " — is
false. It turned about a quarter circle.

### The value, and why it is provisional

Set `LANGROBO_STEADY_ANGULAR_VEL=1.20` (69 °/s) in `~/.langrobo/brain.env`
2026-09-10. Two independent measurements agree:

- §13 (2026-08-21): body turned 65–75 °/s across four 360° pivots.
- `logs/measure_yaw_rate.py` (2026-09-10): one clean 3 s plateau at
  L −0.30 / R +0.32 m/s = **1.21 rad/s**, wheel telemetry continuous throughout.

**Provisional because of §33.** Most runs that day were unusable — the wheel link
drops out under driving load and the rover stutters ~1 s on, ~1 s off, so the
measured average is dominated by dropout rather than by the drivetrain. The
value above comes from the single window where the link held. **Re-measure after
§33 is fixed**, and on the surface actually driven — §13 warns scrub is
surface-dependent.

`LANGROBO_TURN_STARTUP_S` was deliberately left at 0.0. A 1.5–2 s "startup lag"
was visible, but it was §33's dropout, not mechanical ramp, and baking a link
fault into a calibration constant is exactly how it would get hidden.

### VERIFIED end to end 2026-09-10 — it works

Confirmed live through the real brain: a `String` published to
`/voice/user_input`, with `/cmd_vel` and `/gyro/base` recorded. The length of
the `/cmd_vel` window is the tell — `movement.py` holds the twist for
`radians(angle)/STEADY`, so it reveals the angle the *model* asked for,
independently of what was said to it.

| asked | model asked | twist held | achieved | was, before the fix |
|---|---|---|---|---|
| 360° | ~355° | 5.16 s | **357.0°** | ~86° |
| 180° | ~182° | 2.65 s | **202.0°** | ~43° |
| 90° | ~91° | 1.32 s | **114.3°** | ~21° |
| 90° (repeat) | ~88° | 1.27 s | **83.6°** | ~21° |

The 4.2× systematic error is gone. `/wheel_state` was continuous for every one
of these windows (max gap 0.11–0.16 s), so none of it is §33 dropout.

**The user's report of "still only ~90°" was against the un-fixed brain.** The
process that served it started before `brain.env` was written; the journal shows
zero user turns between the 13:20 restart and the report.

### And it still looked broken — because Studio never got the file

Reported again after the verification above, from a LangGraph Studio thread:
`R:180`, `R:360`, `R:90` all "looks like half rotations". Every number in that
trace is consistent with the **un-fixed** constant.

**`brain.env` reaches `langrobo-brain.service` and nothing else.** systemd loads
it via `EnvironmentFile=`. Studio is started by `scripts/start_studio.sh`, which
sourced ROS and nothing else — so it ran **the same graph** with
`_STEADY_STATE_ANGULAR_VEL` back at the commanded 5.0. Confirmed by reading
`/proc/<studio pid>/environ`: no `LANGROBO_*` at all.

So on the same rover, minutes apart, the same request measured 357° through
`agent_node` and about 86° through Studio. Two processes, one file, and only one
of them was ever given it.

Fixed 2026-09-10 in `start_studio.sh` (Pi 5 repo `1dbac6e`, copy synced to
`phase4/pi5/`): source `~/.langrobo/brain.env` with `set -a`, print
`loaded calibration from ...` on startup, and **warn loudly when the file is
absent** — "no calibration" must never again look identical to "calibrated".
Verified: the restarted Studio process has `LANGROBO_STEADY_ANGULAR_VEL=1.20`.

**Anything else ever added to `brain.env` inherits this trap.** Two processes run
this graph and they are configured by different mechanisms.

### A detour worth recording: it is not left/right asymmetry

The first suspicion was direction, since every command in the reported trace was
`R:` and every verification run had been `L:`. §13 does document the two sides
scrubbing differently (LEFT 1.60×, RIGHT 1.29× on a 360). Measured head to head,
same duration, alternating:

| dir | driven | coast | total | wheels L/R | wheel msgs | max gap |
|---|---|---|---|---|---|---|
| L | 114.3° | 0.0° | 114.3° | −0.198/+0.197 | 21 | **0.95 s** |
| R | 186.7° | 5.4° | 192.1° | +0.304/−0.311 | 56 | 0.17 s |
| L | 167.2° | 4.4° | 171.6° | −0.263/+0.274 | 55 | 0.12 s |
| R | 182.0° | 5.1° | 187.2° | +0.289/−0.296 | 59 | 0.13 s |

Both directions land near 180°. **The one outlier is a §33 dropout, and its own
row says so** — 21 wheel messages instead of ~55, and a 0.95 s gap. Without that
column it would have read as a clean 40% left/right asymmetry and sent the whole
investigation the wrong way. That column exists because §33 says to check it.

### What is left is variance, not a systematic error

Two numbers behind the totals above, from splitting the gyro integral at the
moment the twist stops:

- **driven rate 60–66 °/s** while commanded, consistently. `STEADY=1.20`
  implies 68.8 °/s, so the constant is ~5–10% optimistic. Close enough that
  changing it is not clearly an improvement.
- **coast after the stop varies wildly: 7°, 24°, 36°.** `_drive_for_duration`
  does publish a zero Twist, so this is mechanical spin-down, and it is the
  dominant error term on short turns. Two identical "turn left 90" requests
  produced 114.3° and 83.6°.

So repeatability is roughly **±20–25%**, and a single multiplicative constant
cannot fix it — the coast is an additive offset, so the correction it needs is
different for 90° than for 360°.

**`_TURN_STARTUP_DELAY` is the right-shaped knob and it is NOT safe to use
negatively.** `_duration()` would go negative for small angles, and `move_robot`
sends any step with `dur <= 0` down the `else` branch, which publishes a moving
twist that **nothing ever stops** — the wheels then run until the 500 ms
firmware watchdog catches them. `_parse_step`'s docstring already records that
exact bug from a previous encounter. Using it would need a
`max(0.05, ...)` floor in `_duration()` first.

**If precision matters more than this, the answer is closed-loop.** Open-loop
timing on a skid-steer whose scrub varies with surface, battery and heat cannot
do much better than ±20%. Turning until the *measured* yaw reaches the target
would remove the calibration constant entirely. `/wheel_state` integrated
through `WHEEL_BASE_ROT_M` gives a Pi 5-local yaw source that does not depend on
the Jetson being up — with a timeout fallback to the current open-loop path,
since §33 can stop that telemetry mid-turn.

## 🟠 37. Rotation-induced x,y drift — reported informally, not yet verified against a controlled test (2026-09-10)

Reported from manual pushing: coordinates track fine forward/back, but "when it
taking left and right or rotations it values slightly shifting wrongly." Asked
whether more sensors (LiDAR, or 2 spare IR rangefinders on hand) are needed to
fix it.

### The report itself is not yet a reproducible bug

Numbers quoted from a live `./rover compare` read — not a graded `--spin`,
`--return` or `--expect` run, and the exact physical motion (how far pushed,
how many degrees turned, whether it was actually returned to the start mark)
was never confirmed:

    wheel   (-4.4, -1.6, 2°)
    FUSED   ( 2,   22,  -2°)
    cuvslam (19,   18,  -9°)
    gyro    (4°)

With no known push distance or rotation angle, none of these is yet an
"error" — they can only be compared to each other, not to truth. And this
exact test class **already has a documented PASS baseline** that contradicts
the "wheels look correct" read: README.md's Phase 1 gate table records, under
a controlled test, heading 360° spin error of **3.68°** (gate ≤10°) and drift
of **2.5 cm hand-pushed** / **4.9 cm driven hard through 12 teleports** (gate
≤10 cm) — all from the FUSED output, not wheels. So either something has
regressed since that baseline, or — far more likely, since no controlled
maneuver was confirmed — the quoted numbers are not from a comparable test.

### Why "wheels look correct" is backwards

This is a skid-steer rover (ARCHITECTURE.md): turning is inherently a
controlled skid, and wheel encoders are the sensor MOST likely to silently
**under-read** a rotation (slip against the floor), not the one most likely
to be obviously noisy. That is exactly why `fusion.py` deliberately does not
trust wheels for heading — FUSED heading comes from the gyro, FUSED distance
from cuVSLAM (`compare.py`: "distance from cuVSLAM, heading from the gyro").
Small wheel numbers are not evidence of accuracy; they can just as easily
mean the wheels quietly missed the turn.

### What is already in place to fight this

- `WHEEL_BASE_ROT_M = 0.5216` — an empirically measured "effective" rotational
  wheelbase, wider than the real 0.34 m track, specifically compensating for
  skid-steer scrub inflating the apparent turn radius read from encoder ticks.
- Continuous gyro bias recalibration, and cuVSLAM sanity-checked against
  landmark count and implausible-jump rejection.
- §36 above already measured skid-steer turn repeatability directly: driven
  rate steady to ~5-10%, but **coast-after-stop varies wildly (7°, 24°, 36°
  between identical commands)** — the physical randomness this item is
  chasing is already measured and documented as inherent to the drivetrain,
  not a missing calibration constant.

### What to test, in order, before touching hardware

1. `./rover compare --expect 2.00` — push exactly 2.00 m straight, Ctrl-C.
   Confirms straight-line scale is still near the −2.3% baseline.
2. `./rover compare --spin 360` — mark a heading, rotate exactly one full
   turn, Ctrl-C. Compare FUSED heading error against the 3.68° baseline
   (gate ≤10°).
3. `./rover compare --return` — mark start position + heading, drive/turn
   freely, return to the exact mark, Ctrl-C. Compare FUSED drift against the
   2.5 cm / 4.9 cm baseline (gate ≤10 cm).
4. Only if 1-3 actually FAIL against these numbers: re-measure `cam_x`/
   `cam_y` (`vo_node.py`) with a tape measure. The code's own comment already
   flags `cam_x` as "unverified on your rig" and consequential for exactly a
   spin test — a camera mounted forward of the true pivot swings on a lever
   arm during rotation, producing a fake translation that looks identical to
   this symptom.

### Sensors asked about

- **The 2 spare IR rangefinders: not useful here.** Short-range proximity
  sensors don't help egomotion or localisation. Their real use is the
  separate, already-documented gap in README.md: "nothing below 10 cm, above
  24 cm, outside 87°, and nothing downward at all — no drop-off detection."
  Use them there, not on this problem.
- **LiDAR: a real option, but last resort.** 2D frame-to-frame scan-matching
  (laser odometry, not persistent-map SLAM — consistent with the "no
  persistent map, fresh start every time" decision) would give a
  rotation-slip-immune correction layered on the existing VIO+gyro+encoder
  fusion. Only worth the integration cost if steps 1-4 above show a genuine,
  reproducible failure against the documented baseline — the current sensor
  suite (D555 stereo + IMU + encoders) already passed this exact test class
  once, at 2.5-4.9 cm and 3.68°.

### Status

Deferred by the user 2026-09-10 ("will see it later... minor issue"). Nobody
has re-run the controlled test above on the current build to confirm whether
this is a real regression or just an unverified live reading. Do that first,
before any hardware discussion.

## 🟡 38. Studio's CORS fix lives in site-packages, so pip will silently undo it (2026-09-11)

The hosted Studio UI renders **blank** against a server whose `/ok` is healthy,
because the browser's private-network preflight is rejected:

```
OPTIONS /assistants/search  ->  400 Disallowed CORS private-network
```

starlette >= 1.0 added `allow_private_network` (default `False`) and rejects
that preflight; langgraph-api 0.10.0 builds its `CORSMiddleware` without ever
passing it, even though it computes `ALLOW_PRIVATE_NETWORK` correctly and its
own `PrivateNetworkMiddleware` adds the response header — too late, the 400 has
already been issued. A version bump on either side caused this; it worked before.

Patched on the Pi 5 by hand:

```
~/.local/lib/python3.12/site-packages/langgraph_api/server.py
    allow_private_network=config.ALLOW_PRIVATE_NETWORK,   # added
    (original kept as server.py.orig)
```

**Why this is a TODO and not done:** that file belongs to pip, not to either
repo, so it is in no commit and no backup. Any `pip install -U langgraph*`, a
rebuilt venv or a fresh SD card brings the blank page back, and the symptom
points nowhere near the cause. The durable fix is upgrading langgraph to a
release that passes this itself — deferred because the same upgrade moves the
graph runtime `agent_node` depends on, which is not a thing to do casually on a
working brain. Diagnostic curl is in [OPERATIONS.md](OPERATIONS.md) §7.

## 🟡 39. The robot reads VLM pixel coordinates out loud (2026-09-11, unreproduced)

Reported by the user: asked to go somewhere, the speaker said something like
`x:477, y:647`. Typed turns in Studio did not do it.

That format is the VLM grounding prompt's, in `langrobo_core/tools/approach.py`:

```
{"found": true, "x": N, "y": N}    normalized 0-1000, (0,0) top-left
```

which matches the reported numbers. The mechanism is available — there is no
filter between a reply and TTS (`langrobo_core/tools/__init__.py`: "each
agent's reply text IS the speech"), so any reply containing that JSON is spoken
verbatim.

**Not yet reproduced.** Every `→ TTS:` line in the brain's journal for that day
was clean, so it is unclear whether it came from `agent_node` or from a
Studio-driven turn — Studio's replies reach the same speaker through
`/studio_bridge` but are logged in a different process, which would explain the
absence. Do not add a coordinate-stripping filter before catching one real
instance: the useful fix depends on which path emitted it, and a regex over
spoken text would hide the bug rather than fix it.

```bash
journalctl -u langrobo-brain --since today | grep "→ TTS:"
```

---

## 🟠 40. Nav2 refuses gaps the rover fits through, and cannot turn its way out (2026-09-11)

Reported by the user: a goal is accepted, the rover drives, then **stops dead at
a narrow place it can physically pass**; and facing a wall with the goal behind
it, it never tries to rotate out — it needs to "proactively try to fit into that
path by rotating safely".

This is the explanation for **§34**, which recorded the same two failures — run A
`follow_path` aborting into repeated backups, run B `compute_path_to_pose`
aborting six times with no movement at all — and could not separate them. They
are not one bug. They are three, and they compound.

### 1. The planner forbids a band of gaps the rover fits through

`footprint_padding` was 0.07. Padding expands the footprint used to compute the
**inscribed radius**, and that radius is a hard gate in two places at once:
`InflationLayer` writes cost 253 to every cell within it, NavFn treats 253 as
solid, and RPP's collision sweep uses the same padded polygon.

The inscribed radius here is set by the **rear overhang** (0.170 m), not the
half-width (0.190 m) — the rear edge is the nearest one to centre. So:

| padding | inscribed | narrowest gap the planner accepts | real margin |
|---|---|---|---|
| 0.07 (was) | 0.24 m | **0.48 m** | 5 cm/side |
| 0.03 (now) | 0.20 m | **0.40 m** | 1 cm/side |
| 0.02 | 0.19 m | 0.38 m | 0 cm/side |
| 0.00 | 0.17 m | 0.34 m | **−2 cm/side** |

The rover is **0.38 m** across the tyres. At 0.07 the refused band was gaps of
roughly 0.38–0.53 m — passable, rejected — which is most interior doorways once
nvblox's 0.05 m voxels thicken each jamb. That is the user's symptom exactly.

Read the last row before dropping padding further: at zero the planner would
route the rover through a 0.35 m gap it cannot enter, because the gate is the
rear overhang and the body is wider. **0.02 is the floor**, and it is not a
safety margin — it is the correction that makes the inscribed radius equal the
true half-width. Set to **0.03**: 1 cm/side of planner margin, doorways allowed.

The old comment in `nav2.yaml` ended *"if it starts refusing doorways, this is
why"*. It did. The note was right and nobody was watching for it.

### 2. Cost regulation was inverting the wrong curve

`InflationLayer` decays cost at `cost_scaling_factor: 2.0`. RPP inverts that
curve to recover "how far am I from the nearest obstacle" — using **its own
copy** of the constant, `FollowPath.inflation_cost_scaling_factor`, which was
never set and defaulted to **3.0**:

```cpp
// regulation_functions.hpp, costConstraint()
min_distance_to_obstacle = inscribed + log(253 / pose_cost) / f;
if (min_distance_to_obstacle < cost_scaling_dist)
    vel *= cost_scaling_gain * min_distance_to_obstacle / cost_scaling_dist;
```

Two different constants for one curve: RPP read every cost as **a third closer**
to the obstacle than it is. Worse, `cost_scaling_dist` defaulted to 0.6 while
`min_distance_to_obstacle` can never fall below the inscribed radius — so the
rover was *always* inside the scaling window and permanently capped at
0.24/0.60 = 40% of `desired_linear_vel`, about **0.07 m/s**, any time a wall was
in the costmap. That is the crawl half of the report. Now 2.0 and 0.40.

### 3. There was no recovery that turns — only one that reverses

`behavior_plugins` was `[backup, wait]` and the tree's round-robin was
clear → BackUp 0.50 → Wait → BackUp 0.30. Nothing rotated.

**This rover is blind behind it.** One forward camera, 87° wide, nothing below
10 cm, nothing rearward. BackUp was the least safe recovery available and it was
the only one. Facing a wall with the goal behind, the sequence could only
reverse blindly and give up — never turn to face the goal.

It was removed for a reason that expired on **2026-08-22**, when §14 was fixed
and verified at 65 °/s. The controller settings were relaxed that day
(`use_rotate_to_heading: true`); the recovery settings were not. Three weeks of
the robot being unable to get itself unstuck came from a stale premise nobody
re-read.

### The trap that kept the wrong diagnosis alive

Restoring Spin by adding it to `behavior_plugins` alone would have **reproduced
the original bug**. Spin reads `max_rotational_vel` / `min_rotational_vel` /
`rotational_acc_lim` unnamespaced from the server, and ours held the stock-ish
`0.6 / 0.2 / 1.0` — **entirely inside** the 0.8 rad/s scrub breakaway measured in
§14. Spin would have commanded 0.2–0.6 rad/s, the wheels would have sat still,
the behaviour would have timed out, and the log would have said "this rover
cannot pivot" for the second time about a rover that can.

Set to `1.5 / 1.0 / 10.0`: the cap matches the velocity smoother so nothing
downstream clips it, the floor stays above breakaway so the last few degrees of
a spin still move, and the acceleration limit clears breakaway on the first
100 ms tick — the same fix, for the same reason, as the controller's
`max_angular_accel: 10.0`.

### Changed

| file | change |
|---|---|
| `phase3/config/nav2.yaml` | `footprint_padding` 0.07 → **0.03**, both costmaps |
| | `progress_checker` → `PoseProgressChecker`, `required_movement_angle: 0.5` — turning now counts as progress |
| | `FollowPath.inflation_cost_scaling_factor: 2.0`, `cost_scaling_dist: 0.40` |
| | `FollowPath.max_allowed_time_to_collision_up_to_carrot` 1.5 → **1.0** |
| | `behavior_plugins: [spin, backup, wait]`, rotational limits 1.5/1.0/10.0, `simulate_ahead_time: 1.0` |
| `phase3/bt/navigate_to_pose.xml` | renamed from `navigate_no_spin.xml`; round-robin now clear → **Spin +90°** → BackUp 0.30 → **Spin −90°** → Wait |
| `phase3/bt/navigate_through_poses.xml` | renamed from `navigate_through_no_spin.xml`, same round-robin |
| `phase3/launch/nav2.launch.py` | `behavior_server` `cmd_vel` → `cmd_vel_nav`, so recoveries go through the velocity smoother instead of stepping the PID |
| `knowledge/07-planners-and-controllers.md` | the "no Spin or BackUp" section was stale *and* wrong — the tree always had BackUp |

### Not yet verified on the floor — this is why it is 🟠

Nothing here has been driven. Required, in order:

1. **Tape-measure the actual footprint width.** Every number above rests on
   0.38 m across the tyres, and `nav2.yaml` has flagged the tyre half-width as
   the one assumption in that polygon since 2026-08-23. Measure it before
   trusting a 1 cm/side margin.
2. **Confirm Spin actually PIVOTS — log x,y, not just yaw.** This is the test
   most likely to fail, and the reason is a hole in the evidence this whole
   entry leans on. `nav2.yaml`'s wz sweep (2026-08-23) read yaw rate off
   `/odom`, and **yaw rate cannot tell a pivot from an arc** — a curving turn
   gives the same number. Meanwhile `phase1/teleop/teleop_web.py` measured the
   day before that wz 2.0 is only ~47% duty and "cannot break four tyres loose
   sideways", and §14 put a real pivot at ~93% duty, i.e. wz ≈ 4.7. Spin is
   capped at 1.5 here, which is ~25% duty per wheel.

   If that reading is right, nav2 has never pivoted — it has curved — and a
   Spin recovery that translates while it turns is **worse than no Spin** in
   exactly the narrow gaps this entry is trying to get through. It would also
   be a clean candidate cause for §37 and §36.

   Command a pure rotation through nav2 and log `/odom` x **and** y **and**
   yaw. Yaw alone proves nothing. If it curves, the fix is more angular
   authority (`velocity_smoother` `max_velocity[2]` and Spin's
   `max_rotational_vel`, currently both 1.5) — but raise it deliberately, since
   `nav2.yaml` records that commanded 2.0 made RPP overshoot badly, and
   OPERATIONS.md §2 warns a fast pivot disturbs cuVSLAM. **Until this runs,
   treat the Spin recovery as unproven.** Cross-referenced in the Pi 5 repo's
   `INTEGRATION_GAPS.md` §3, which flagged the 1.5 cap before this change did.
3. **The doorway test.** A gap measured at 0.42–0.45 m, goal on the far side.
   Before this change it was refused; it should now plan and pass.
4. **The wall test the user described.** Nose to a wall, goal 1 m behind.
   Expect: no path → clear → Spin +90° → replan → drive. Watch that it turns
   rather than reverses.
5. **Watch for grazing.** 4 cm/side of planner margin was given up. If it
   touches anything, raise `footprint_padding` to 0.05 (0.44 m gaps) rather
   than reverting to 0.07.

Related: **§34** (the observations this explains), **§14** (the pivot fix whose
consequences were only half applied), **§13/§22** (why commanded ≠ actual wz),
**§37** (rotation-induced x,y drift — more spinning will exercise it).
