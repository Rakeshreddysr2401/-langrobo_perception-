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

The D555 stopped streaming at 01:21 on 2026-08-11 and everything downstream is
waiting on it. Do this first, in order:

```bash
cd ~/langrobo_perception/orin-nav-stack
./run_stack.sh status          # expect camera IR / depth / cuVSLAM at 0.0 Hz
./run_stack.sh cam             # cheap software relaunch — try this first
./run_stack.sh status          # if camera is back, carry on below
```

If it is still 0 Hz: **physically power-cycle the camera** — unplug the PoE
cable for ~5 s and replug. No software restart recovers a dead D555 DDS server,
and it keeps answering ping the whole time it is dead.

Once the camera is streaming:

```bash
./run_stack.sh fuse            # ALWAYS before mapping — pins odom z to 0
./run_stack.sh status          # want: camera >=15, cuVSLAM >=10, trust=True
./run_stack.sh view start      # RViz on the laptop (must be logged in at it)
```

Then re-drive the room by hand and see whether the map comes out clean now that
z is pinned. That is the remaining issue 00 gate work.

**Do not run `remap` and then drive** — it drops fusion and z drifts. It now
warns you, but the habit is: `remap` ➜ `fuse` ➜ drive.

---

## Blockers and open questions

Things you hit that you could not resolve. Keep them here rather than in your
head, so a later issue can pick them up.

| Found | Issue | What | Status |
|---|---|---|---|
| 2026-08-11 | 00 | **D555 stopped streaming** (IR/depth 0 Hz, `map->odom` MISSING). Log shows "callback took too long!" then silence. Needs `cam` relaunch or a physical PoE power-cycle. | **blocking** |
| 2026-08-10 | 03 | ESP32 `/wheel_state` publishes at 10.0 Hz; firmware asks for 20 Hz (`rover_firmware_v2.ino:391`). Cause not yet found. | open |
| 2026-08-10 | 06 | `esdf_slice_min_height` still at the 0.12 workaround value; the 4 cm camera-height error it was dodging is now fixed, so it can come down. | open |
| 2026-08-10 | later | Pi 5 wheel-odom relay has no systemd unit — it dies on every reboot and must be restarted by hand. | open |
| 2026-08-10 | — | Camera sees forward only (~87°). No pan/tilt head, so the map has no sides or back. Hardware gap, not a bug. | accepted |
