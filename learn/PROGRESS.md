# PROGRESS — where you are right now

Update this file as the last step of every issue, in the same commit.

**Currently on: issue 00 — setup.**

| # | Issue | Status | Date passed | Notes |
|---|---|---|---|---|
| 00 | setup — can I see anything? | 🟡 in progress | | 3 of 4 gate items done; blocked on the D555 needing a power-cycle |
| 01 | cuVSLAM — where am I? | ⬜ not started | | |
| 02 | odometry — is my position correct? | ⬜ not started | | |
| 03 | imu — survive the camera blinking | ⬜ not started | | known issue: encoders at 10 Hz, want 20 |
| 04 | nvblox — what does the world look like? | ⬜ not started | | |
| 05 | map on the fly — build and keep it | ⬜ not started | | |
| 06 | obstacle detection — what's in my way? | ⬜ not started | | carries the `esdf_slice_min_height` 0.12 → ~0.06 change |
| 07 | nav2 — drive to a goal by itself | ⬜ not started | | |
| 08 | path planning — why *that* path? | ⬜ not started | | |

Status key: ⬜ not started · 🟡 in progress · ✅ passed gate · ❌ gate failed

---

## Gate results

Record the actual numbers here when a gate passes. Numbers, not "worked" — the
next issue may need them, and six weeks from now "worked" tells you nothing.

<!-- Example of what a good entry looks like:

### Issue 01 — cuVSLAM  ✅ 2026-08-11
    cuVSLAM odom rate : 20.8 Hz   (gate: >= 10)
    pose trust        : True
    TF map->odom age  : 0.06 s
    RViz: TF tree visible, base_link followed the rover when pushed.
    Note: had to restart the camera once — realsense log said
    "No RealSense devices were found", needed a physical PoE power-cycle.
-->

---

## PICK UP HERE — next session

The stack is **healthy and running as of 2026-08-11**, with RViz live on the
laptop. Verified this session:

```
  camera IR 26.5 Hz · depth 20.2 Hz · cuVSLAM 23.0 Hz · nvblox 9.5 Hz
  EKF 20.0 Hz · IMU 74.5 Hz · trust=True · load1 4.0
  /odometry/filtered z = 0.0   ← exactly, this is the smearing fix
  at the laptop: map 9.5 Hz, odom 20 Hz, /rover/model subscribed
```

The **remaining issue 00 gate work is yours to do by eye**, not something a
command can prove:

1. Look at the laptop screen. Confirm you can see the **rover body** (grey box,
   green forward arrow, blue camera, translucent FOV wedge) and the **TF tree**.
2. **Push the rover by hand around the room.** Watch two things:
   - does `base_link` slide across the grid, leaving the map behind it *intact*?
   - do walls come out as **lines**, not the thick smeared blobs you saw
     yesterday? That is the whole point of pinning z.
3. Say out loud what `map`, `odom` and `base_link` each mean.

Bring-up order from cold, every time:

```bash
cd ~/langrobo_perception/orin-nav-stack
./run_stack.sh up              # camera + TF + cuVSLAM + nvblox
./run_stack.sh fuse            # ALWAYS before mapping — pins odom z to 0
./run_stack.sh status          # want: camera >=15, cuVSLAM >=10, trust=True
./run_stack.sh view start      # RViz on the laptop (must be logged in at it)
```

**Do not run `remap` and then drive** — it drops fusion and z drifts. It now
warns you, but the habit is: `remap` ➜ `fuse` ➜ drive.

Ignore the ESP32 encoder complaint for now. Nothing in issue 00 uses the wheels,
and it is issue 03's job — but read the blocker note below, because the number
changed and it now says something different from what it said yesterday.

---

## Blockers and open questions

Things you hit that you could not resolve. Keep them here rather than in your
head, so a later issue can pick them up.

| Found | Issue | What | Status |
|---|---|---|---|
| 2026-08-11 | 00 | **Subscribing to a raw D555 topic can kill the camera.** Happened twice in one day: toggling the RViz Image display, and an ad-hoc script subscribing to `depth/image_rect_raw` + `color/image_raw`. Those topics are lazily published, so a new subscriber triggers stream start/stop over the D555's DDS control channel, which times out (`"id":"hwm"`) and the device goes **offline**. Only a physical PoE power-cycle recovers it. Use `./run_stack.sh vision` to look at imagery. | **known, avoid** |
| 2026-08-11 | 04 | **The occupancy map is a solid black wedge, not a room.** Measured from a stationary rover: occupied 9.3 % vs free 5.6 % — *more wall than floor*, which is backwards. The occupied region averages ~1 m thick where a wall should be one 5 cm cell. Cause not yet established; the slice-band test that would settle it was cut short when the camera went offline. This is what issue 04/06 is for. | **open — blocks "map looks like a house"** |
| 2026-08-11 | 00 | ~~D555 stopped streaming~~ — **resolved 2026-08-11 by a plain `up` after a full power cycle.** No cable-pull was needed. Worth knowing: a dead D555 DDS server can survive a container restart but not a host reboot. | resolved |
| 2026-08-11 | 03 | ESP32 `/wheel_state` dropped from 10.0 Hz to **exactly 1.00 Hz** (jitter ±0.05 s) across the reboot. This is *not* half-rate and *not* idle: the firmware publishes unconditionally every 50 ms once `AGENT_CONNECTED` (`rover_firmware_v2.ino:391`), with no motion gating. The only 1000 ms timer in the file is the `WAITING_AGENT` ping (`:374`), so the ESP32 is very likely **not in the connected loop**. Check the micro-ROS agent on the Pi 5 first — it has no systemd unit either. | open |
| 2026-08-11 | 00 | `view` aborted with "does not respond to ping" while ssh worked fine. Laptop wifi power-saves: RTT swings 21 → 128 ms and the first ICMP packet is dropped. `ping -c1` now `-c3`. | fixed |
| 2026-08-11 | 00 | `view start` failed again on Wayland — but a *different* variant: Xwayland had not started yet (GNOME starts it on demand), so the probe silently guessed `:0` + `~/.Xauthority`. It now refuses to guess and says so. | fixed |
| 2026-08-10 | 06 | `esdf_slice_min_height` still at the 0.12 workaround value; the 4 cm camera-height error it was dodging is now fixed, so it can come down. | open |
| 2026-08-10 | later | Pi 5 wheel-odom relay has no systemd unit — it dies on every reboot and must be restarted by hand. | open |
| 2026-08-10 | — | Camera sees forward only (~87°). No pan/tilt head, so the map has no sides or back. Hardware gap, not a bug. | accepted |
