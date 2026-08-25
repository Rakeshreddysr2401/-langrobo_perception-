# READINESS — what stands between this and an autonomous vehicle

Measured 2026-08-26 on branch `rover-v1.0.5`. Every value below was read off the
running stack, not remembered; the last section says how to reproduce each one.

`TODO.md` holds the problems, one per entry, in the order they were found. This
holds the **cross-cutting** view: subsystem by subsystem, what is measured
against what is missing. The two disagree about nothing; this one is for
deciding what to do next.

---

## The verdict

| | |
|---|---|
| **Stack today** | All six gates green. Two frame bugs fixed 2026-08-26 (§22, §23); 0 tracker resets since. |
| **What actually blocks autonomy** | Not the software. Three hardware-shaped gaps make a human mandatory. |
| **Still unmeasured** | Three gates — drift (§2), 360° spin (§5), estimate grading (§15) — have never produced a valid score. |

The uncomfortable summary: the estimator is in better shape than the vehicle.
Nothing in the list of real blockers is fixed by better estimation.

---

## Readiness by subsystem

Severity follows `TODO.md`: 🔴 blocks a gate · 🟠 real, worked around ·
🟡 unverified · ⚪ accepted.

| subsystem | state | the one-line problem |
|---|---|---|
| D555 camera | 🟠 | Streams clean. Drops out in three ways, each needing a human. |
| cuVSLAM | 🔴 | Diverges; rotation is where its translation goes wrong. |
| Encoders / ESP32 | 🟡 | Calibrated and steady at 20 Hz. No power telemetry at all. |
| Fusion | 🟠 | Working well. CPU-bound, and publishes no uncertainty. |
| Loop closure | 🟡 | Now usable after §23. Nothing renders the corrected frame. |
| nvblox | 🟡 | Maps well. Loop closure never corrects what it already wrote. |
| nav2 | 🟠 | Drives goals reliably. One session only — no persistent map. |

---

## D555 camera

The only exteroceptive sensor on the vehicle.

| | |
|---|---|
| IR stereo | 896 × 504 × 30 |
| intrinsics / baseline | `fx 448.3` / 9.49 cm |
| emitter / laser power | **OFF** / 0.0 — both load-bearing |
| colour · motion · sync | off · on · off |
| transport | PoE DDS, 192.168.11.55 — **not USB** |
| mount x / y / z | 0.170 / 0.00 / 0.163 m — measured |
| mount yaw | 2.06° — measured |

**🔴 It needs hands (§7).** Three distinct failure modes, all requiring a
physical PoE reseat; four failures in one day on record. It **pings while its
DDS server is dead**, so ping proves nothing, and no software restart recovers
it. This is the single largest blocker on the vehicle.

**🟡 Mount pitch never measured (§6).** x, z and yaw are all measured. Nose
up/down is not, and it moves the ground plane in mapping.

**🔴 Blind below 10 cm, above 24 cm, outside 87°, and downward.** There is no
drop-off detection of any kind. This is why a human watches every run.

**🟡 Emitter off weakens depth on untextured surfaces** — the same condition
that starves cuVSLAM of landmarks.

---

## cuVSLAM

Visual odometry, and the only source of translation *direction*.

| | |
|---|---|
| rate | 27–30 Hz |
| landmarks, healthy | 100–200 |
| SLAM mode | on |
| `planar_constraints` | True — but see below |
| distance bias (§11) | −2.2 % |
| heading, forward (§4) | −1.74° |
| heading, reverse (§4) | **+7.68°** |
| tracker resets since §22 | 0 |

**🔴 Still diverges, root cause unknown (§21).** `planar_constraints` is a
`SlamConfig` field and the divergence is in the **odometry** pose;
`OdometryConfig` has no planar option at all. The one setting whose name
promises to prevent vertical drift was never in that code path.

**🔴 Rotation is where translation goes wrong.** A powered in-place pivot
produced **743 cm** of phantom translation while the fused pose held position to
**1.4 cm**. Consistent with §4, and a candidate explanation for the 291 rejected
corrections in §23. That run lost battery partway — **indicative, not graded.**
It needs a clean repeat.

**🟠 Materially worse in reverse (§4).** Worked around by taking heading from
the gyro. Design consequence: prefer turn-then-drive over reversing.

**🟡 Failure tracks texture, not speed (§3).** Divergences on 2026-08-26
occurred at 2, 4, 12, 13 and 14 landmarks. The old 25 cm/s limit was disproven —
84–89 cm/s ran clean with good texture.

---

## Encoders and ESP32

The only measurement that does not care about light.

| | |
|---|---|
| `/wheel_state` | 20.0 Hz |
| metres per count | `π · 0.085 / 1560` |
| track, physical | 0.34 m |
| track, effective | **0.5216 m** — scrub, not a tape reading |
| `enc_scale`, live | 0.85 – 0.92 |
| pivot breakaway | **0.80 rad/s** commanded |
| firmware loop | 727 Hz |
| free heap | 175 KB |

**🔴 No voltage or current telemetry.** `/rover_diag` is a `Vector3` carrying
loop rate, free heap and agent state — nothing about power. On 2026-08-26 the
rover decayed from 0.77 rad/s of rotation to **no motion at all** while every
topic stayed green and the ESP32 reported `agentState = 2 (CONNECTED)`. The
failure was completely invisible to software. Two spare floats in a message that
already exists is the cheapest high-value fix on this page.

**🟠 Effective track width is floor-specific.** 0.5216 m against a 0.34 m tape
reading is scrub, and scrub changes with surface, tyre wear and load. What is
calibrated is one floor.

**🟡 The 0.80 rad/s deadband is a silent stall.** Commanded rotation below it
scrubs the tyres and does not turn the chassis. Nothing currently notices.

---

## Fusion

Encoder magnitude · cuVSLAM direction · gyro heading. See `phase1/nodes/fusion.py`.

| | |
|---|---|
| `/odom` | 20.0 Hz |
| gyro | 200 Hz |
| `JUMP_M` | 0.15 m |
| `MAX_PUSH_MS` | 1.0 m/s |
| `VO_MAX_Z` | 0.30 m |
| `LOW_LANDMARKS` | 30 |
| `YAW_TRUST_VO` | 2.0 e−5 |
| endpoint error, fused (§2) | 12.2 cm |

**🟠 CPU-bound.** `fusion_node` is the heaviest process in the container at
**47.8 %** — single-threaded Python servicing 200 Hz gyro, 27 Hz VO, 20 Hz
wheels and three timers. `wheels_alive` was observed flapping under load while
`/wheel_state` itself held 19.99 Hz with **zero** gaps over the 0.5 s threshold
and a 62 µs clock offset. The link was fine; the node was saturated.

**🟠 Publishes no uncertainty.** `Odometry.pose.covariance` is deliberately
zeroed rather than published in the wrong frame — defensible, but it means nav2
cannot reason about pose confidence and nothing downstream can behave
differently when the estimate is weak.

**🔴 Yaw has no absolute reference.** Gyro bias walks; cuVSLAM heading is the
only anchor, and it is the input least trustworthy during rotation. Nothing
bounds heading error over a long run.

**🟡 12.2 cm against a 10 cm bar (§2)** — and that bar was *chosen* (2 voxels at
5 cm), not measured.

---

## Loop closure and frames

The REP-105 split: `odom` continuous, `map` corrected. See
`vo_node.py::_publish_correction`.

| | |
|---|---|
| closures per session | 15 – 27 |
| `map → odom` | published |
| correction, typical | 1.5 – 6.9 cm |
| rejected, before §23 | **291** (against 15 accepted) |
| rejected, after §23 | 0 |
| `MAX_CORRECTION_M` | 5.0 m, **planar** |

**🟠 Nothing renders the corrected frame.** `/fusion/path` is stamped
`frame_id: odom` and the RViz Fixed Frame is `odom`, so closure corrections are
invisible on screen *by construction*. A `map`-framed twin of the path is the
missing piece — and it is the reason the operator's "it draws a line beside
itself" was so hard to attribute.

**🔴 No relocalization against a saved map.** Nothing survives a power cycle;
every session starts a new origin. This is Phase 2c, and it is what "knows where
it is" actually requires.

**🟠 A pose reset discards the map (§16)**, because the map was built in the old
frame.

---

## nvblox

Depth plus the **fused** pose → TSDF, mesh, and a 2D ESDF slice.

| | |
|---|---|
| voxel size | 0.05 m |
| ESDF mode / rate | `2d` · 5 Hz |
| slice band | 0.10 – 0.24 m |
| slice height | 0.16 m |
| integration distance | 3.0 m |
| grid published | 4.9 Hz |

**🔴 The map is not deformable.** nvblox integrates in `odom`, so a loop closure
corrects the *frame* but never the geometry already written. Drift accumulated
during a lap stays baked into the mesh.

**🟠 Pose damage is permanent.** The map on 2026-08-26 measured 5.4 m² of "wall"
against a room perimeter that should be under 1 m², median clearance 0.36 m
against a 0.35 m want, and only 14 % of cells beyond 1.0 m (want ≫ 15 %) — with
obstacle blobs scattered through open floor. Smeared by the §22 origin resets.
It had to be **wiped, not repaired**.

**🟡 The band is the blind spot.** 3.0 m integration caps the planning horizon;
0.10–0.24 m cannot see overhangs, or anything lower than 10 cm.

**🟡 Dynamic obstacles untested** — published on a separate layer, never
exercised.

---

## nav2

Plans and drives on the nvblox slice. See `phase3/config/nav2.yaml`.

| | |
|---|---|
| controller | RPP · 10 Hz |
| `desired_linear_vel` | 0.18 m/s |
| lookahead | 0.3 – 0.9 m |
| goal tolerance xy / yaw | 0.10 m / 0.25 rad |
| footprint | 35 × 38 cm |
| padding · inflation | 0.07 · 0.45 m |
| `max_angular_accel` | 10.0 rad/s² |
| recoveries | `backup`, `wait` |

**🔴 Navigates within one session only.** The global frame is `odom`, not `map`.
There is no way to send the rover somewhere it knew about yesterday — which is
most of what "autonomous vehicle" implies.

**🟠 Recovery is thin by necessity.** Spin is deliberately absent (§14) because
the chassis cannot reliably pivot in place; nav2 would command a rotation, sit
still and time out. Correct — but it leaves only BackUp and Wait between a stuck
rover and a failed goal.

**🟡 The smoother's angular cap and its own comment disagree.** `max_velocity`
theta is `1.5` commanded — about 0.41 rad/s actual by the 2026-08-26 sweep —
while the comment beside it reasons about 2.5. Worth reconciling.

**🟡 No mission layer above single goals.** No sequencing, no retry policy, no
"patrol these points".

---

## What actually blocks unattended operation

Not ranked by difficulty. These are the things that make a human mandatory.

1. **The camera needs hands.** §7. Until a switchable PoE port or a watchdog
   power-cycle exists, unattended running is not possible.
2. **Power is invisible.** The rover ran itself flat with every gate green.
   Nothing can refuse a mission on low battery, return to charge, or explain a
   stall.
3. **Nothing looks down.** No drop-off detection. A stair edge, a threshold, a
   balcony — no sensor would see it. A sensing gap, not a tuning one.
4. **No memory between sessions.** Every power cycle starts a fresh origin and
   an empty map. Without relocalization there is no persistent "the kitchen",
   only coordinates that expire with the process.

---

## Sequence

Ordered by what unblocks the most downstream work. Steps 1–3 are measurement;
nothing after them is trustworthy without them.

| # | do | why |
|---|---|---|
| 1 | Re-run the three gates on a charged battery | §2, §5, §15 have never produced a valid number, and every tuning decision depends on them. `logs/yaw_crosscheck.py` now automates the spin. |
| 2 | Add voltage and current to `/rover_diag` | Two more floats in a `Vector3` that already exists. Turns a silent failure into a readable state. |
| 3 | Quantify cuVSLAM's rotational translation error | The 743 cm figure is indicative. If it holds, rotation needs an explicit mitigation beyond the gyro heading override. |
| 4 | Publish `/fusion/path` in the `map` frame | Makes loop closure visible and drift a number you can watch rather than infer. |
| 5 | Get `fusion_node` off the CPU ceiling | A multi-threaded executor, or decimating the 200 Hz gyro, removes a class of load-dependent flapping. |
| 6 | Make the camera recoverable without hands | The gate between "supervised demo" and "autonomous vehicle". |
| 7 | Then relocalization — Phase 2c | Save a map, localize into it on boot, move nav2's global frame to `map`. |

**One caution on ordering.** Relocalization is tempting to pull forward because
it sounds like the headline feature. Building it on a drifting estimate bakes
that drift into the saved map permanently — the same way the 2026-08-26 map had
to be wiped rather than repaired. It is last for a reason.

---

## Where the numbers came from

```bash
./rover status                      # every layer's rate, and the cuvslam honesty line

docker exec rover bash -lc 'unset ROS_DISCOVERY_SERVER; export ROS_DOMAIN_ID=0; \
  source /opt/ros/jazzy/setup.bash; ros2 topic echo /vo/status --once'
#   loop_closures, tracker_resets, correction_rejected, correction_flattened

docker exec rover bash -lc '... ros2 topic echo /fusion/status --once'
#   vo_z, vo_implausible, jumps, dr_metres, enc_scale, landmarks

docker exec rover bash -lc '... ros2 topic echo /rover_diag --once'
#   x = loop() Hz, y = free heap KB, z = agent state (2 = CONNECTED)

tegrastats                          # GPU %, thermals, RAM
docker exec rover ps -eo pid,pcpu,comm --sort=-pcpu   # per-node CPU
```

Map quality, and the pivot and yaw measurements:

```bash
docker exec -it rover bash -lc 'source /opt/ros/jazzy/setup.bash; \
  export ROS_DOMAIN_ID=0; python3 -u /logs/SCRIPT.py'
#   map_stats.py       free / occupied / unknown, in m2
#   esdf.py            median clearance, wall:floor ratio -- is it planable?
#   map_view.py --once the map as ASCII, rover marked
#   wzsweep.py         the commanded wz that actually rotates this chassis
#   yaw_crosscheck.py  360 deg spin, all four yaw sources, drift during pivot
```

Config values are read from `phase3/config/nav2.yaml`,
`phase2/launch/nvblox.launch.py`, `phase1/nodes/fusion.py`,
`phase1/nodes/vo_node.py` and the `camera)` block of `./rover`.
