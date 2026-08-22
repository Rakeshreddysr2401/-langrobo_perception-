# Teleop — hold-to-move web control

**This runs on the Pi 5, not the Jetson.** It is copied here so the tuning is
version-controlled; the live copy is `~/langrobo_teleop/` on `192.168.1.16`,
started by `langrobo-teleop.service`.

Open **`http://192.168.1.16:8091`** on a phone on the same WiFi.

**It defaults to AUTO and the buttons do nothing until you flip it to MANUAL.**
That is nav2 or the brain owning `/cmd_vel`, not a fault. In MANUAL it publishes
at 10 Hz whether or not a button is held, so `/cmd_vel` traffic alone proves
nothing — only a non-zero velocity does.

---

## The units mismatch that cost a mapping session

`rover_firmware_v2.ino` reads `wz` as **radians per second** and computes each
wheel as `vx ± wz × WHEEL_BASE_M/2`, with `WHEEL_BASE_M = 0.34`.

This app was written for an **earlier firmware** (`rover_sim
contract_bridge.py`) where `wz` was a **PWM fraction**. Its original comment
claimed `WZ = 2.0` gave "~100% PWM per wheel". Against this firmware it gives
**48%**:

| WZ | wheel target | duty | |
|---|---|---|---|
| 2.0 | 0.34 m/s | 48% | not enough to scrub four tyres sideways |
| 4.0 | 0.68 m/s | 87% | |
| **5.0** | **0.85 m/s** | **100%** | current setting |

Full authority is `2 × 0.86 / 0.34 = 5.06 rad/s`, both wheels at maximum in
opposite directions.

### How it presented

Not as "turning is weak". As **the rover driving backward or forward when told
to turn** — the wheels could not break loose, so the small residual differential
pushed it along a slight curve instead. Measured 2026-08-22, `logs/turn_diag.py`:
counter-rotating in **0%** of samples on the floor, and **100%** of samples
lifted on blocks at the same command.

It cost a whole mapping session before anyone looked: 10.7 m of "room loop"
driven inside a **1.8 m box**, total rotation **−11°**. The map was never wrong;
the rover simply never went anywhere.

---

## Speeds

| button | vx | wz | note |
|---|---|---|---|
| forward / back | ±0.20 m/s | 0 | proven: 0.197 measured against 0.200 commanded |
| left / right | 0 | ±5.0 | pivot, ~290 °/s. **Tap, do not hold** |
| slight left/right | 0.08 | ±4.25 | inner wheel reverses, net forward creep |

Release is a **coast**, not a brake — the wheels decay to zero over ~0.5 s.
Motion is gated on a `/cmd_vel` newer than 500 ms, so a lost connection stops
the rover.

---

## Restarting it without a password

The service runs as `rakhi24` with `Restart=always`, so killing the process is
enough — systemd respawns it in ~3 s with whatever the file now says:

```bash
ssh 192.168.1.16 'pkill -f teleop_web.py'
```

`sudo systemctl restart langrobo-teleop` does the same thing and needs a
password.

---

## Never run the keepalive at the same time

`logs/keepalive.py` publishes zero `/cmd_vel` continuously. It would interleave
with real commands and make the rover stutter. It exists only as a fallback if a
future firmware flash regresses the executor fix.
