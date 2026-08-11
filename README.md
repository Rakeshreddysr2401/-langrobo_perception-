# rover

**Click a goal in RViz, and the rover drives there, on a map it built itself.**

That is the whole objective. Everything here serves it.

---

## Start here

| Read | For |
|---|---|
| **[PRD.md](PRD.md)** | what we are building, what we are deliberately not, and the rules |
| **[FACTS.md](FACTS.md)** | everything we know **because we measured it**. The most valuable file here. |
| **[ARCHITECTURE.md](ARCHITECTURE.md)** | every block, its inputs and outputs, and the frame tree |
| **[TODO.md](TODO.md)** | every known bug and open question, nothing hidden in a comment |
| **[tasks/](tasks/)** | the work, one layer at a time |

---

## The idea: layers you can prove

The old stack started camera + TF + cuVSLAM + nvblox all at once. That is why it
was so hard to debug — when something broke, any of four layers could have done
it, and on 2026-08-11 it ran for an hour with **two nvblox nodes competing**
without anyone noticing.

Here each layer starts **only what it adds**, and refuses to start if the layer
below it is unhealthy.

```bash
./rover.sh l1      # camera alone
./rover.sh l2      # + cuVSLAM          (needs l1 healthy)
./rover.sh l3      # + gyro + EKF       (needs l2 healthy)
./rover.sh l4      # + nvblox mapping   (needs l3 healthy)
./rover.sh l5      # + nav2             (needs l4 healthy)  MOVES THE ROBOT
```

Support commands:

```bash
./rover.sh status          # health of every layer — measures RATES, not existence
./rover.sh measure 2.00    # tape-measure the pose: push it 2.00 m by hand
./rover.sh floor           # where does the robot think the floor is? want ~0.000
./rover.sh view start      # RViz on the laptop
./rover.sh logs cuvslam    # tail any component
./rover.sh stop            # e-stop
./rover.sh down            # remove the container
```

## The task loop

One task per session. Never two.

```
read the task  ->  run it  ->  watch it in RViz  ->  CHECK THE GATE  ->  commit
```

The gate is the point. "It looked fine" is not a gate — record the **numbers**.

---

## Six rules

1. **Never advance past a gate you have not verified.**
2. **One layer at a time.** Two changes at once teach you nothing.
3. **The machine beats the doc.** If they disagree, fix the doc in the same commit.
4. **Measure rates, not existence.** Every failure here was a rate collapsing.
5. **Check pose trust before believing anything spatial.** A frozen cuVSLAM still reports `slam_pose_ok: true`.
6. **Nothing enters `src/` until you have run it, tested it and can explain it.**

---

## Three things that will bite you

**Never subscribe to a raw camera topic to "just look at it."** Those topics are
lazily published; a new subscriber makes the driver restart streams over the
camera's fragile DDS control channel. It has taken the camera offline twice, and
only a physical PoE power-cycle recovers it.

**Minimise restarts.** nvblox is the only depth subscriber, so restarting it
cycles the camera's stream too. Six restarts in an hour took the camera from
26 Hz to offline.

**The camera answers ping while completely dead.** Use `./rover.sh status`.

---

## Status

Nothing here has been run yet. The code was promoted from a working stack and
the wiring was corrected, but **this repo's layers have never been executed** —
see `TODO.md §7`. Start at [`tasks/01-camera.md`](tasks/01-camera.md).

The old stack at `../langrobo_perception/` is the **parts bin**: read it, never
edit it. It still runs, so it is your fallback.
