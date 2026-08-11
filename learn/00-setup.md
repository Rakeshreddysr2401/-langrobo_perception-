# 00 — Setup: can I see anything at all?

**Needs:** nothing. This is the start.
**Motors move:** no. Nothing in this issue can make the rover move.
**Time:** ~20 minutes the first time.

---

## What this is (read first)

Before learning anything about how the robot thinks, you need a window into it.
That window is **RViz**, running on your laptop, showing what the Jetson believes.

This is its own issue — not part of issue 01 — because getting RViz on screen is
a *networking and display* problem, and understanding visual odometry is a
*robotics* problem. Mixing them means that when nothing shows up you won't know
which of the two you're debugging. That has already cost a full session here.

### The two traps that cause "RViz shows nothing"

**Trap 1 — the laptop moved.** It used to be `192.168.1.12`. DHCP later gave
`.12` to the **ESP32**. If you ssh to `.12` you are talking to a
microcontroller, which refuses the connection and looks exactly like a laptop
that's switched off. **The laptop is `192.168.1.10`.**

**Trap 2 — nobody is logged in.** If the laptop is sitting at the login screen,
RViz launched over ssh starts, renders into nothing, and reports no error. It
looks broken. It isn't — there is simply no desktop to draw on. **You must be
physically logged in at the laptop**, not just able to ssh to it.

**Trap 3 — Wayland hides the display (found 2026-08-11, now fixed).** The laptop
runs a Wayland desktop, and on Wayland `loginctl` reports an **empty** display.
`run_stack.sh view start` used to guess `:0` and pass no X authorization cookie,
so rviz2 died in under a second with:

```
Authorization required, but no authorization protocol specified
qt.qpa.xcb: could not connect to display :0
```

…written to `/tmp/rviz.log` **on the laptop**, where nobody thinks to look. From
the Jetson it just said "rviz2 did not stay up".

The fix reads both the display number and the cookie path from the **Xwayland
process itself** (`pgrep -a Xwayland` → `-auth /run/user/1000/.mutter-Xwaylandauth.XXXXXX`),
which is authoritative for both. `view` now also prints the tail of the laptop's
log when the launch fails, instead of telling you to go and read it.

This is the third variation on the same theme: **RViz fails silently and the
evidence lands on the other machine.**

### Why the robot is a container

The whole stack (cuVSLAM, nvblox, nav2, YOLO, all the nodes and configs) lives in
one Docker container built from `orin-nav-stack/`. The repo is mounted
**read-only** over the baked copy, which means:

> **Edit a config or a node on the Jetson → restart → it takes effect. No rebuild.**

That is the loop you will use for all nine issues.

---

## Do this

### 1. On the Jetson — bring the perception stack up

```bash
cd ~/langrobo_perception/orin-nav-stack
./run_stack.sh up
```

This starts: the D555 camera, the base TF tree, cuVSLAM (full SLAM), and nvblox.
It does **not** start nav2 and does **not** touch the motors.

Give it about 30 seconds. If it aborts complaining the camera never streamed, see
*If it fails* below.

### 2. On the Jetson — read the health

```bash
./run_stack.sh status
```

This is your single source of truth. It measures **rates**, not "is something
publishing" — because every failure this robot has had was a rate collapsing to
1 Hz, not a topic disappearing.

You want something close to this:

```
  ok camera IR left       26.0 Hz   >=15 Hz
  ok camera depth         23.5 Hz   >=10 Hz
  ok cuVSLAM odom         20.8 Hz   >=10 Hz
  ok nvblox slice          9.2 Hz    >=5 Hz
  ok EKF odom             20.0 Hz   >=10 Hz
  ok IMU (gyro)           74.2 Hz   >=50 Hz
  !! ESP32 encoders       10.0 Hz   >=15 Hz     ← known, that's issue 03
  ok TF map->odom        0.06s old
  ok TF odom->base_link  0.06s old
  ok pose trust      trust=True  status=OK
  ok orin load1      2.5
```

Two lines are expected to complain right now, and neither blocks you:

- **ESP32 encoders 10.0 Hz** — real bug, it's what issue 03 is for.
- **collision src: not started** — that's a nav2 thing, it arrives in issue 07.

### 3. Physically log in at the laptop

Walk over to it. Log in properly to the desktop. Not ssh — the actual screen.

### 4. From the Jetson — check the laptop is ready

```bash
./run_stack.sh view
```

This pings the laptop, proves ssh works, and — importantly — checks that a real
desktop session exists rather than just `gdm`. It tells you plainly which of the
two traps you're in.

### 5. On the laptop — start RViz

```bash
bash ~/rover_view.sh
```

That script is already set up: it clears any stale discovery settings, sets
`ROS_DOMAIN_ID=0`, and opens `~/laptop_view.rviz`.

(You can also run `./run_stack.sh view start` from the Jetson to launch it
remotely, but running it on the laptop is easier to debug the first time.)

---

## See it in RViz

You should get a window with the TF tree, a camera image, and an empty grid.

**Set `Fixed Frame` to `odom`** in the top-left panel. Then look at the TF
display and find these four frames:

```
  map ──► odom ──► base_link ──► camera0_link
```

Now **push the rover gently by hand, about half a metre.**

> ✔ **Hand-pushing is safe.** The firmware's PID returns zero whenever the target
> speed is under 0.01, and at zero duty both motor driver pins are pulled low —
> that is *coast*, not brake. The wheels free-wheel. The motors will not fight
> you. (Re-check this if the firmware ever changes: a PID that actively *held*
> zero would resist you.)

`base_link` should slide across the grid as you push. That is the robot watching
itself move.

---

## GATE — all four must be true

```
  [ ] ./run_stack.sh status shows camera >= 15 Hz and cuVSLAM >= 10 Hz
  [ ] ./run_stack.sh view reports "ok desktop session"
  [ ] RViz on the laptop shows the camera image and a live TF tree
  [ ] You can say out loud what map, odom, and base_link each mean
```

That last one is not filler. If `map` vs `odom` isn't clear yet, re-read
[`ARCHITECTURE.md` §2](ARCHITECTURE.md) — nearly every confusing thing this robot
does is a frame problem, and issues 01–08 all assume you have this.

---

## If it fails

| Symptom | Cause | Fix |
|---|---|---|
| `up` aborts, log says "No RealSense devices were found" | The D555's on-camera DDS server is dead | **Physically power-cycle it** — unplug the PoE cable ~5 s, replug. No software restart recovers this. It still answers ping while dead, so ping is not a health check. |
| `view` says "does not respond to ping" | Laptop off/asleep, or you're on `.12` | It's `192.168.1.10`. Check with `getent hosts rover-esp32.local` that you're not looking at the ESP32. |
| `view` says "NOBODY IS LOGGED IN" | Trap 2 | Physically log in at the laptop. |
| `view start` says "rviz2 did not stay up" | Trap 3, or a genuine RViz error | `view` now prints the tail of the laptop's `/tmp/rviz.log` for you. If it mentions "Authorization required" or "could not connect to display", the Xwayland cookie lookup failed — check `pgrep -a Xwayland` on the laptop. |
| RViz opens but everything is empty | `Fixed Frame` is set to a frame that doesn't exist yet | Set it to `odom`. |
| RViz opens, no topics at all | Domain mismatch | Use `bash ~/rover_view.sh` — it sets `ROS_DOMAIN_ID=0` and clears stale discovery vars. |
| `status` shows camera at ~1 Hz | Orin is overloaded | Check `orin load1`. Above ~8 the camera starves and **cuVSLAM freezes silently and never recovers**. Stop something and restart the stack. |

---

## Commit

Record what you actually saw in `PROGRESS.md`, then:

```bash
git add learn/PROGRESS.md
git commit -m "issue 00: setup verified — RViz live on laptop, camera 26 Hz, cuVSLAM 21 Hz"
```

**Next:** [`01-cuvslam.md`](01-cuvslam.md) — where am I?
