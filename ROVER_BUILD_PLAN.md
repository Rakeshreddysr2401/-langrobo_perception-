# Rover build plan — geometry, turning, power and weight

**Why this exists.** The software now plans around the rover's turning fault
(`./rover drive`, see [LOCALIZATION.md](LOCALIZATION.md)), and that gets a
+90° in place to within ~2 cm. The next step toward *precise and fast* is
the hardware and the description of it. Four things are open:

1. **The geometry is guessed.** Wheel, camera and LiDAR positions came from a
   description, and they live in four files that disagree.
2. **The rover cannot turn about its centre.** The motors work near stall on
   every pivot.
3. **Nothing measures the battery.** Pivot torque falls as the pack drains.
4. **Nobody has weighed it.** Weight on each wheel decides which side slips.

This plan turns each into measurements, a design change or a test, in an order
where each step feeds the next. Written 2026-09-24 from the owner's request and
the measurements in LOCALIZATION.md and TODO.md §43.

---

## 0. What we know, what we assume

| | value | source |
|---|---|---|
| motors | Rhino GB37 12 V, 200 RPM no-load, 1:30, **stall 3.5 kg·cm**, rated 0.3 A | `phase1/firmware/HARDWARE.md` §3 |
| wheels | 85 mm OD, tape-checked | firmware `WHEEL_DIAMETER_M` |
| drive | skid-steer, 4 driven wheels, 2 motors in parallel per BTS7960, one driver per side | HARDWARE.md §2, §4 |
| control | per-side PI on encoder speed, 50 Hz, saturates to full duty when a side lags | firmware `pidStep` |
| motor power | 3S pack → both BTS7960s; ESP32 on a separate 65 W USB bank | owner; HARDWARE.md §1 |
| track, physical | **0.34 m** wheel centre to wheel centre — the firmware's `WHEEL_BASE_M` | TODO.md §13, HARDWARE.md |
| track, effective for turns | **0.5216 m** `WHEEL_BASE_ROT_M` — measured over four 360° spins; the wheels over-read turns by ~1.53× | TODO.md §13 |
| track, RViz model | 0.385 m — from the owner's description of the tyres, **disagrees with 0.34** | `rover_marker.py` |
| body / envelope | 36 × 28 cm body, 46 × 42 cm with tyres — **described, not measured** | nav2.yaml |
| camera | x 0.170, z 0.163 m | ARCHITECTURE.md §3 |
| LiDAR | x 0.135, z 0.21 m **(described)**; yaw +88.60° **(measured)** | `./rover`, lidar/README.md |
| mass, CG, corner loads | **unknown** | — |
| battery voltage under load | **unknown**: no telemetry | — |

**Two of the three tracks are consistent; the third is not.** A skid-steer's
*effective* turning width is wider than its physical track because the tyres
slip sideways in a turn: 0.5216 m effective against 0.34 m physical is exactly
that, and it is measured. The RViz model's 0.385 m came from a description of
the tyres ("7 cm past each side of a 28 cm body"), so it and the footprint
derived from it are the likely error. §1 settles it with a tape.

TODO §13 also measured the **left side scrubbing more on turns**: its front and
rear encoders disagree by 1.60× in a 360°, against 1.29× on the right,
reproducible to half a percent. That was recorded on 2026-08-21, before any of
the turn work, and it points at the same asymmetry.

### What the turn measurements say (TODO 43, LOCALIZATION.md)

| mode | slide per 90° "in place" | pivot point | consistent? |
|---|---|---|---|
| pure wz 1.5 rad/s | 28–33 cm | left tyres, sometimes the centre | no |
| pure wz 5.0 rad/s (the brain's turns) | 31–68 cm, one stall at 67° | anywhere from the right tyres out to 66 cm | **no** |
| **held-left** 1.5 rad/s | ~21 cm per 45°, ~36–53 cm per 90° | left tyres, **spread 1.5 cm** | **yes** |

Heading is good in every mode (≤ 1.6°). The problem is **where** the rover
turns, not how far.

### Why it slides: the physics, with the one assumption stated

To pivot about its centre, a four-wheel skid-steer must drag every tyre
sideways around a circle of radius r = ½√(L² + W²). Per wheel, the push
needed is about

    F ≈ μ·N·√(1 + (L/W)²)

and a tyre cannot push harder than μ·N before it slips. For any L/W, that
factor is > 1, so **a pivot always needs the wheels to slip**, and the motor
must hold near μ·N·r_wheel of torque while doing it.

With the assumption **m ≈ 4 kg, μ ≈ 0.7 (rubber on glazed tile)**:
N ≈ 9.8 N per wheel, so torque ≈ 0.7 × 9.8 × 0.0425 = 0.29 N·m = **3.0 kg·cm**.
That is **85% of the motor's 3.5 kg·cm stall**. A DC motor's speed falls
linearly with load, which leaves ~15% of free speed (≈0.13 m/s at the wheel).
The rover measured ~11%. The model and the measurement agree, which is the
point: **this is a torque problem, and it is predictable.**

Two consequences drive the rest of this plan:

- **Battery voltage matters a lot for turns.** Stall torque scales with
  voltage. At 11.1 V (3S nominal) it is ~3.2 kg·cm and the margin over 3.0
  shrinks from 15% to ~7%. Near 10.5 V it is gone and **the rover can barely
  pivot at all**. Straight driving needs ~17% duty and will not notice.
- **Whichever side has more weight on it stalls first**, because N is bigger
  there. That is the leading suspect for "the left side does not move". §5
  measures it.

---

## 1. Measure — the tape and the scales (you, ~30 min)

Everything after this uses these numbers. Measure from **base_link = the
point on the floor midway between the four wheel contact patches**, x forward,
y left, z up. Take each measurement twice.

| # | what | how | value |
|---|---|---|---|
| 1 | wheel contact, left front (x, y) | centre of the tyre's floor patch | |
| 2 | wheel contact, left rear | | |
| 3 | wheel contact, right front | | |
| 4 | wheel contact, right rear | | |
| 5 | **track** W (L↔R contact centres) | from 1–4 | |
| 6 | **wheelbase** L (front↔rear axle centres) | from 1–4 | |
| 7 | tyre width, wheel OD (loaded) | caliper; OD while on the floor | |
| 8 | body extents: front, rear, left, right | to the outermost point, tyres included | |
| 9 | **camera**: lens centre x, y, z; which way it looks (pitch) | tape to the left IR lens | |
| 10 | **LiDAR**: rotor centre x, y, z | centre of the spinning window | |
| 11 | mass, total | bathroom scale: you with it, minus you | |
| 12 | **corner loads** LF, LR, RF, RR | one scale under each wheel in turn, the other three on blocks of the same height | |
| 13 | battery: chemistry, capacity (mAh), C rating, connector | label | |
| 14 | what powers the Jetson, Pi 5 and PoE injector | follow the cables | |

Rows 12 and 14 are new to this project and cost nothing.

---

## 2. CAD — one model of the real rover

**Goal:** a model accurate to ~2 mm where it matters (wheel contacts, sensor
mounts) and ~1 cm elsewhere. It produces three things: meshes for RViz, a
collision shape for nav2, and mass properties (CG, inertia) from component
weights.

**Tool:** **Onshape** (free, in the browser, and `onshape-to-robot` exports a
URDF straight from an assembly) or FreeCAD. Pick one and keep it; the
geometry must have one home.

**Model, in this order:**
1. Chassis plate(s), with the four motor mounts at the §1 positions.
2. Wheels as cylinders, and motors as boxes with their measured weights.
3. The D555 and the C1 at their mounts. These two positions matter most.
4. Battery, Jetson, Pi 5, ESP32, drivers, power bank, each as a box with its
   real mass. This is what makes the CG come out right.
5. A simplified collision body: the footprint as a box, plus anything that
   sticks out.

**Check it against the scales.** The CAD's total mass and CG must match §1
rows 11–12 to within ~5%. If they do not, a component's weight or position is
wrong.

**Also use it to try layouts** before moving anything: where the battery would
have to go to even out the corner loads (§5).

---

## 3. URDF — one source of geometry for everything

Today the same numbers are typed into four places:

| where | what it holds |
|---|---|
| `./rover` | `LIDAR_X/Z/YAW`, and the camera static TF |
| `phase2/nodes/rover_marker.py` | the RViz model's dimensions |
| `phase3/config/nav2.yaml` | the footprint |
| firmware `WHEEL_BASE_M` | the physical track, used for its wheel mixing |
| Jetson `WHEEL_BASE_ROT_M` | the effective turning width |

**Target:** a xacro in `description/` that reads one params file.

```
description/
  rover.urdf.xacro        links + joints
  params.yaml             every measured number, with its date and how it was measured
  meshes/                 from the CAD
```

**The link tree:**

```
base_link (floor, midway between the wheels)
 ├─ chassis_link          visual/collision mesh, inertial from CAD
 ├─ wheel_lf/lr/rf/rr     continuous joints; spin in RViz from /wheel_ticks
 ├─ camera0_link          measured; + the IR optical frame cuVSLAM uses
 │   └─ camera_imu        the D555's gyro (gyro_node re-frames into base_link)
 └─ laser                 measured x,y,z; yaw from ./rover lidar --calibrate
```

**What switches over to it:**

- `robot_state_publisher` publishes the fixed transforms. The two
  `static_transform_publisher` calls in `./rover` go away. They were the cause
  of the lost-LiDAR-frame bug, so this also removes a whole class of fault.
- RViz shows the URDF model with real meshes; `rover_marker.py` retires.
- A small script derives nav2's `footprint` from the collision body, so it
  cannot drift from the model.
- **Keep the physical and effective tracks as two separate, labelled numbers**
  in `params.yaml`: the URDF's wheel joints use the physical track (0.34 m,
  which the firmware's mixing also uses), and `WHEEL_BASE_ROT_M` (0.5216 m) is
  a calibration result that belongs next to it. Label both, so nobody "fixes"
  one to match the other.
- `vo_node`'s camera-to-body offset must be read from TF, not a constant
  (`cam_x`/`cam_y`, flagged unverified in its own comments).

**Validate the URDF, in order:** TF matches §1 to ±2 mm; `./rover lidar
--calibrate` still reads ~+88.6°; `./rover pivot` gives the same pivot points;
`./rover drive` results no worse.

---

## 4. Power — know the voltage, and plan for it

### 4.1 Measure it: the one piece of hardware to buy first

**An INA226 (or INA219) power-monitor board** in series with the motor pack,
on I²C. It gives voltage, current and power. ~₹200–400.

**Why not the ESP32's ADC:** every ADC1 pin (32–39) carries an encoder, and ADC2
cannot be read while WiFi is on. The BTS7960's `R_IS/L_IS` current-sense
outputs are analog too, so they have the same problem.

**Where it connects:** I²C on two **free** pins, **GPIO16 / GPIO17**. `Wire` can
use any pins. Also free: 4, 14, and the boot-strapping pins 2, 12 and 15, which
are best left alone (12 sets the flash voltage at boot; 2 and 15 affect
download mode and boot logging).

**Getting it to the Jetson.** The firmware is at its micro-ROS entity cap
(see the note by `INIT_OR_FAIL`), so there is no room for a new topic. Use
`/rover_diag` instead: keep `x` = loop Hz, put **battery volts** in `y`
(replacing free heap, which has served its purpose), and **pack current (A)**
in `z`. Move the agent state into the existing serial heartbeat.

### 4.2 Characterise the sag

With telemetry in, run the same tests at three charge levels: **full
(~12.5 V), mid (~11.4 V), low (~10.8 V)**.

| at each level | command | record |
|---|---|---|
| straight speed hold | `./rover wheels --nudge` and a 1 m drive | commanded vs reached |
| pivot capability | `PIVOT_ANCHOR=hold ./rover pivot 45 -45` | slide, pivot, time per turn, **pack current** |
| stall current | a pivot held 3 s | peak A |

This gives the table the software needs: **below what voltage do precise turns
stop being precise?**

### 4.3 Use it

- `./rover drive` and the brain **refuse precise moves below that voltage**,
  and say why, instead of producing a bad turn.
- A low-battery warning on the brain (Telegram / voice).
- Log voltage alongside every `pivot` / `drive` result, so a bad run can be
  told apart from a flat pack.

### 4.4 Check the supply layout

Motors, Jetson, Pi 5, ESP32 and the D555's PoE should each be on a **known,
separate** supply, or be regulated off the pack with enough headroom.
§1 row 14 answers what they are on now. **A motor stall must never brown out
the ESP32 or the Jetson.** The current split (motors on the pack, ESP32 on the
USB bank) already gives that for the ESP32. Keep it.

---

## 5. Weight distribution

**Measure** (§1 row 12): the four corner loads, and from them the CG.

    CG_x = L·(front − rear) / (2·total)
    CG_y = W·(left − right) / (2·total)

**What we expect to learn:** if the left corners carry more weight, the left
tyres grip harder, stall first, and the rover pivots about them. That matches
the 1.5 rad/s behaviour and the left side's larger scrub in TODO §13. The 5.0 rad/s behaviour (pivoting about the *right*
side at first, then anywhere) says it is not only weight; motor or driver
differences are the other candidate, and the lift test separates them.

**Targets:**
- **Left/right within 5%.** Symmetric grip is what makes a pivot land at the
  centre.
- **Front/rear within 10%.** A pivot centre follows the CG.
- **CG as low as practical.** The LiDAR and camera are on top at the front.
  Heavy things (the battery) go low and central.

**How:** move the battery first. It is the heaviest single item, so a few cm
of it balances more than anything else. Use the CAD (§2) to find the spot,
then re-weigh.

---

## 6. Turning — the options, ranked

### 6.1 Software (in place now, or small)

| | state |
|---|---|
| held-left turns (predictable pivot) | ✅ `./rover drive` default |
| plan around the slide (rotate/straight/rotate/straight, re-plan on the LiDAR pose) | ✅ `./rover drive` |
| **the brain's turns** — timed, 5.0 rad/s, measured unpredictable (31–68 cm, one stall) | ⬜ move them to closed-loop turns on the gyro, in held-left mode, or call `./rover drive` |
| balanced pivot: cap the stronger side to the weaker side's measured speed | ⬜ experiment. Centred but slow; only possible if the weak side can move at all |
| voltage-gated precision (§4.3) | ⬜ needs the INA226 |

### 6.2 Mechanical: fixes for the cause

From the physics in §0 (push needed ≈ μ·N·√(1 + (L/W)²), torque ≈ μ·N·r_wheel):

| change | effect | cost |
|---|---|---|
| **lower-RPM GB37 (e.g. 100 RPM)**, same mount | ~2× torque. A pivot at ~40% of stall instead of 85%. Top speed ~0.44 m/s, still double today's 0.22 m/s teleop/brain cap | 4 motors; recalibrate `ENCODER_CPR` / `MAX_WHEEL_VEL` |
| **balance the corner loads** (§5) | both sides stall together, so the pivot centres | free |
| **reduce side grip on one axle**: harder tyres or tape on the rear pair | less sideways drag, so μ·N falls where it matters | cheap, reversible, try first |
| **omni wheels on one axle** | that axle stops dragging sideways; the rover pivots cleanly about the other axle | medium; changes the odometry model |
| shorter wheelbase / wider track | lowers √(1 + (L/W)²) | a chassis change |
| keep the pack near full | torque ∝ V (§4) | behaviour |

**Recommended order:** balance weight → try low-grip rear tyres → measure
again → if pivots still run near stall, 100 RPM motors.

### 6.3 Acceptance, for "turns precisely"

- A +90° in place, **as commanded** (no planner): centre moves **< 3 cm**,
  heading error **< 0.5°**, pivot-point spread **< 1 cm** over 8 turns, both
  directions.
- The same with the pack at mid charge.

---

## 7. Speed and accuracy — targets, and how each is tested

| target | test |
|---|---|
| straight: distance error **< 1%** at 0.20 m/s | `./rover compare --expect 2.00`, a tape |
| turn in place: see §6.3 | `./rover pivot` |
| **go and return to a mark: < 2 cm, < 1°** — the owner's goal | `./rover drive --mark A`, a pattern with turns, `--to A`, a tape from the X |
| heading truth: odometry, slam and the walls agree within 1° | the tile test (square to a grout line) |
| **faster**: the top speed at which the go-and-return still passes | repeat the go-and-return at 0.1 / 0.2 / 0.3 / 0.4 m/s |

Teleop and the brain cap speed near 0.22 m/s today. That cap is not a measured
tracking limit: TODO §3 found cuVSLAM's teleports follow floor *texture*, not
speed, and withdrew the old 25 cm/s limit. So the safe top speed is still
unknown. The last row finds it by measurement, with the LiDAR correction in
the loop, rather than by raising the cap and hoping.

---

## 8. The order of work

| step | who | needs |
|---|---|---|
| 1. §1 measurements + corner weights | owner | tape, a scale, blocks |
| 2. finish the live tests (tile, go-and-return, lift) | both | the X on the floor |
| 3. `description/params.yaml` from §1, then the URDF | Claude | step 1 |
| 4. switch the TFs, RViz, footprint to the URDF; re-validate | Claude | step 3 |
| 5. INA226 on GPIO16/17, `/rover_diag` y/z = V/A | owner wires, Claude firmware | the board |
| 6. voltage sweep (§4.2) | both | step 5 |
| 7. weight balance (§5), re-measure the turns | owner, CAD if built | step 1 |
| 8. CAD model | owner (Claude can draft from §1) | step 1 |
| 9. mechanical turn fixes (§6.2) in the recommended order | owner | steps 6–7 |
| 10. the brain's turns onto the fixed primitive | Claude, both repos | steps 6–9 |

Steps 1–4 need nothing bought. Step 5 needs one ~₹300 board. Only step 9 may
need new motors or wheels, and only if the cheap fixes are not enough.

**To buy:** INA226 module · (maybe) a small digital scale per corner, or borrow
one and use blocks · (only if §6.2 says so) 4× GB37 100 RPM, or a pair of omni
wheels.
