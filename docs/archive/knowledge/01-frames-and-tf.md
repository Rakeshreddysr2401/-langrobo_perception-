# Frames and TF

**Used by:** Phases 1, 2 and 3 — nearly everything.

> Nearly every confusing thing a robot does is a frame problem. If you only
> properly learn one topic here, make it this one.

---

## The problem TF solves

The camera says *"there is a wall 2 metres in front of me."* The planner needs
*"there is a wall at x=3.1, y=0.4 in the room."* Those are the same fact in two
different coordinate systems, and something has to convert between them.

TF is that something. It is a **live, timestamped tree of coordinate frames**,
where each edge says "to get from this frame to that one, translate by X and
rotate by Y". Any node can ask "where is this point, expressed in that frame?"
and TF walks the tree for you.

**Timestamped matters.** The rover moves, so `base_link → map` was different half
a second ago. TF stores a short history and interpolates, which is why a stale
transform is an error rather than just an old number.

---

## This robot's tree

```
map ──────────> odom ──────────> base_link ──────────> camera0_link
      ▲                 ▲                     ▲
   cuVSLAM            EKF              static, MEASURED
 (jumps when      (smooth, but         x 0.10, z 0.163
  it recognises    drifts slowly)
  a place)
```

| Frame | Means | Who publishes it |
|---|---|---|
| `map` | the world, drift-corrected | cuVSLAM |
| `odom` | dead-reckoned world | (the parent end of the EKF's transform) |
| `base_link` | the rover itself | EKF |
| `camera0_link` | the camera | `static_transform_publisher` |

**Rule: exactly one publisher per edge.** Two nodes publishing the same
transform fight, and the answer flickers between them. This is why task 03
restarts cuVSLAM with `publish_odom_tf:=false` — up to that point cuVSLAM owned
`odom → base_link`, and from then on the EKF does.

---

## Why there are two world frames

This is the part that confuses everyone, and it is worth getting properly.

**`odom` is smooth but wrong.** It is built by adding up motion. It never jumps,
but small errors accumulate, so after ten minutes it may be half a metre off.

**`map` is right but jumpy.** When cuVSLAM recognises a place it has seen before,
it corrects the accumulated drift — and the robot's position *teleports* to the
corrected answer.

Neither property is optional:

- A **controller** cannot cope with the robot teleporting mid-manoeuvre. It needs
  smooth. → uses `odom`.
- A **goal** must not drift away over ten minutes. It needs accurate. → uses `map`.

So ROS keeps both, and stores the difference in the `map → odom` transform. When
a loop closure happens, **`map → odom` is what jumps** — `odom → base_link` stays
smooth, and everything reading `odom` is undisturbed.

### Where this shows up in our config

- `phase2/launch/nvblox.launch.py`: `global_frame: "odom"`
- `phase3/config/nav2.yaml`: both costmaps use `global_frame: odom`, but
  `bt_navigator` uses `global_frame: map`

So goals arrive in `map` and get transformed through cuVSLAM's `map → odom`,
while the costmaps live in the smooth frame and never teleport under the
controller. That is deliberate — and the cost is that **the map is built in the
drifting frame**, so it is never corrected by loop closure and does not line up
across sessions. That trade-off is task 05's subject.

---

## `base_link`'s origin is on the floor

Not at the axle, not at the centre of mass — **on the ground**.

Everything follows from that:

- open floor must deproject to **z ≈ 0.000**
- the camera TF is `z = 0.163` because the camera is 16.3 cm above the ground
- `phase2/launch/nvblox.launch.py`'s obstacle band, `0.10 → 0.24`, is **height above the
  floor**

### The 3.7 cm that broke the robot

The camera TF was `z = 0.200`, guessed. `tools/floor_probe.py` measured open
floor landing at **+0.037 m** instead of 0.000 — the TF overstated the camera
height by 3.7 cm, so the whole world was placed 3.7 cm too high.

Consequences, in order:

1. floor deprojected *above* zero, into the obstacle band
2. nvblox mapped the floor as an obstacle
3. the costmap grew a wide inflation plateau with **no lethal cells anywhere**
4. MPPI crawled at 0.082 m/s against a `vx_max` of 0.30
5. the collision monitor's SlowZone cut that to 0.033 m/s
6. the rover covered **0.9 cm in 30 seconds** and nav2 aborted

One wrong number in a static transform, six layers away from the symptom.
**Re-run `./rover compare` (floor height check — see TODO §6, never measured) after any camera remount.**

---

## Reading TF when it goes wrong

```bash
ros2 run tf2_tools view_frames        # writes a PDF of the whole tree
ros2 run tf2_ros tf2_echo map base_link
```

| Symptom | Meaning |
|---|---|
| "Could not find a connection between X and Y" | the tree is **broken** — some publisher is dead |
| transform is present but old | a publisher is alive but starving. Check rates. |
| pose flickers between two answers | **two publishers on one edge** |
| everything is offset by a constant | a static transform is wrong — measure it |

`./rover status` shows the age of `map → odom` and `odom → base_link` for
exactly this reason. A transform that exists but is 4 seconds old is worse than
one that is missing, because consumers will happily use it.

---

## The mental model to keep

> TF does not know where anything *is*. It only knows how frames relate. If the
> relationship you gave it is wrong, everything downstream is confidently wrong,
> and nothing raises an error.

---

**See also:** [`02-visual-odometry.md`](02-visual-odometry.md) (who publishes
`map → odom`), [`03-sensor-fusion-ekf.md`](03-sensor-fusion-ekf.md) (who
publishes `odom → base_link`).
