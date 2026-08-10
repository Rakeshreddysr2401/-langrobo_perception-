# 01 — cuVSLAM: where am I?

**Needs:** issue 00 passed (RViz is live on the laptop).
**Motors move:** no. You push the rover by hand.
**Time:** ~30 minutes.

---

## What this is (read first)

The rover has no GPS. Indoors it wouldn't work anyway. So how does it know where
it is?

**It watches the world slide past its cameras.**

### Visual odometry, in one picture

The D555 has two infrared cameras a fixed distance apart. cuVSLAM picks out
sharp features — corners of furniture, edges of a doorframe, texture on a rug —
and tracks where each one lands in the next frame.

```
   frame 1                    frame 2
   ┌──────────────┐           ┌──────────────┐
   │   .    *     │           │  .    *      │
   │      ▲       │           │     ▲        │
   │   corner     │           │  moved LEFT  │
   └──────────────┘           └──────────────┘

   The corner slid left in the image
        ⇒ the camera moved RIGHT
```

Two cameras instead of one matters: because the spacing between them is known,
the shift of a feature between *left* and *right* gives its **distance**. That's
what makes the answer come out in real metres instead of "some amount".

Do that 20 times a second and you have a position. That is **visual odometry**.

### The SLAM part

cuVSLAM also remembers places. When it recognises somewhere it has been before,
it can cancel out the error that accumulated in between — a **loop closure**.

When that happens it does *not* teleport `base_link`. It adjusts the
`map → odom` correction instead. That is why, in RViz, the `map` frame
occasionally jumps while `odom` stays perfectly smooth.

### Two settings that are wrong-by-default, and why

Both of these were found the hard way, and both are already fixed in
`run_stack.sh`. Understanding them will save you later.

**1. IR emitter must be OFF.**

The D555 can project a dot pattern to help depth sensing on blank surfaces. It
sounds like it would help. It is catastrophic for cuVSLAM:

> The dots are painted onto **the world**, not attached to the camera. So as the
> camera moves, the dots stay put on the wall. cuVSLAM tracks them as if they
> were real features, concludes almost nothing moved, and **under-reports
> distance by about 4×**.
>
> Measured: a tape-measured **100 cm** push read as **26 cm** with the emitter on.
> The same push read **97 cm** with it off. Nothing logged an error either time.

**2. Colour stream must not be synced with IR.**

With `enable_sync:=true`, the colour stream's auto-exposure starves the IR
streams, and the map comes up blank. Fixed with `enable_sync:=false`.

Both of these are the same lesson, and it is the lesson of this whole repo:
**this system fails silently.** It keeps producing confident, wrong numbers. That
is why every issue ends in a measured gate.

---

## Do this

### 1. Make sure the stack is up

```bash
cd ~/langrobo_perception/orin-nav-stack
./run_stack.sh status
```

You need `cuVSLAM odom` at 10 Hz or better. If the stack isn't running,
`./run_stack.sh up` first.

### 2. Look at what cuVSLAM is publishing

```bash
docker exec -it orin_nav bash -lc '
  source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0
  ros2 topic echo /odom --once'
```

Find `pose.pose.position` — that is where cuVSLAM thinks it is, in metres, from
wherever it started. It should be near zero if you haven't moved.

### 3. Ask the robot whether it trusts itself

```bash
docker exec -it orin_nav bash -lc '
  source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0
  ros2 topic echo /odom/health --once'
```

`odom_health.py` watches for the specific ways this system has lied before:

| Flag | Meaning |
|---|---|
| `VO_STALLED` | cuVSLAM stopped updating — it has frozen |
| `CAMERA_STARVED` | IR frame rate collapsed, cuVSLAM is about to freeze |
| `VO_UNDER_REPORTING` | Moving, but odometry barely changes — the emitter bug's signature |
| `VO_DRIFT_STATIONARY` | Position changing while the rover is still |
| `VO_SCALE_OFF` | Distance doesn't match the wheels |

You want `trust: true`.

---

## See it in RViz

On the laptop, in the RViz you opened in issue 00:

1. **`Fixed Frame` → `odom`.** Everything is now drawn relative to where the
   robot booted.
2. Make sure **TF** is enabled. You should see the frame chain.
3. Turn on the **`/odom` trail** if it isn't already showing.

Now **push the rover slowly by hand** — walk it a few metres around the room,
including a turn.

> ⚠️ **Push slowly, under ~0.5 m/s.** Fast motion blows up cuVSLAM's feature
> tracking. Turns especially: make them gentle and wide, not sharp pivots.

Watch for:

- **`base_link` follows you.** The robot is tracking its own motion.
- **The trail is a clean line, not a scribble.** A scribble means it's losing
  tracking — usually too fast, or a blank featureless wall.
- **Point the camera at a bare white wall and push.** Watch the trail degrade.
  This is worth doing deliberately: it shows you cuVSLAM's real failure mode, and
  it is why the gyro and wheel encoders exist (issue 03).

---

## GATE — all four

```
  [ ] cuVSLAM odom rate >= 10 Hz in ./run_stack.sh status
  [ ] /odom/health reports trust: true
  [ ] TF map->odom age under ~0.5 s
  [ ] Pushed several metres by hand, /odom trail is a clean line, not a scribble
```

**Not yet checked: whether the distance is *correct*.** A clean trail can still be
wrong by 4×, exactly as the emitter bug was. Proving the number is issue 02.

---

## If it fails

| Symptom | Cause | Fix |
|---|---|---|
| cuVSLAM rate at 0 | Camera not streaming | `./run_stack.sh logs cuvslam` and `logs realsense`. "No RealSense devices were found" ⇒ physical PoE power-cycle. |
| Rate collapses to ~1 Hz and stays | Orin overloaded → camera starved | Check `orin load1` in `status`. Above ~8 starves the camera. **cuVSLAM does not recover on its own** — restart the stack once load is down. |
| Trail is a scribble | Pushing too fast, or no visual features | Push slower. Point it at something with texture. |
| Trail barely moves while you walk metres | The emitter-scale bug pattern | Confirm the emitter is OFF. `/odom/health` should be flagging `VO_UNDER_REPORTING`. |
| `map → odom` age growing | cuVSLAM has frozen | `logs cuvslam`, then restart. |

---

## Commit

```bash
git add learn/PROGRESS.md
git commit -m "issue 01: cuVSLAM verified — 20.8 Hz, trust true, clean trail over ~4 m hand push"
```

**Next:** [`02-odometry.md`](02-odometry.md) — is that position actually correct?
