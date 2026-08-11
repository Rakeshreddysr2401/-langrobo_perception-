# 03 — IMU + encoders: surviving the moments cuVSLAM can't see

> **STUB.** Written in full when you get here. The gate below is deliberately
> unwritten — it depends on measurements from issue 02 and on what the ESP32 is
> actually doing once we look. A made-up number in a gate is worse than no
> number.

**Needs:** issue 02 passed (odometry metrically verified).
**Motors move:** no — hand push, plus a pivot on the spot.

---

## Goal

Fuse the gyro and the wheel encoders into the pose, so it stays honest through
the moments cuVSLAM goes blind — fast pivots, blank walls, sudden light changes.

## What you will learn

Why three sensors are better than one, and specifically what each contributes:

| Sensor | Gives | Fails when |
|---|---|---|
| cuVSLAM | absolute x, y, yaw | fast pivots, blank walls |
| Gyro | yaw *rate*, ~74 Hz | drifts if integrated alone |
| Encoders | forward speed | wheels slip |

Also: why the EKF deliberately **ignores the accelerometer** — a MEMS accel fused
naively diverges by hundreds of metres, because gravity and bias swamp the real
signal. A slow ground robot gains nothing from it and inherits all its drift.

## Known state going in

- **Gyro is already fused and working** — 74.2 Hz, `imu_to_base` rotates it into
  `base_link`, EKF fuses yaw-rate only. This half is done.
- **Encoders are the open problem, and the number is not stable.** Measured
  **10.0 Hz** on 2026-08-10, then **exactly 1.00 Hz** (jitter ±0.05 s) on
  2026-08-11 after a full power cycle. Same firmware, same everything else.
  > **Read the firmware before believing any theory about this.** Once the
  > agent is `AGENT_CONNECTED`, `rover_firmware_v2.ino:391` publishes
  > **unconditionally every 50 ms** — there is no motion gating and no
  > slow/idle path. So a low rate can never be explained by "the rover was
  > parked". And the *only* 1000 ms timer in the whole file is the
  > `WAITING_AGENT` ping at `:374`. A rock-steady 1.00 Hz therefore points at
  > the ESP32 **not being in the connected loop at all**, which is a different
  > bug from the 10 Hz half-rate.
  >
  > Check the **micro-ROS agent on the Pi 5 first** — like the relay below, it
  > has no systemd unit, so a reboot leaves the ESP32 with nothing to connect
  > to. Do that before considering a reflash.
- The Pi 5 relay (`ESP32 /wheel_state → /wheel_odom`) has **no systemd unit**, so
  it dies on every reboot and must be restarted by hand.
- `config/ekf.yaml` already has `odom1: /wheel_odom` wired.

## Likely work

1. First establish **which** failure you have, using the rate: ~1 Hz = not
   connected (check the Pi 5 micro-ROS agent is even running); ~10 Hz = connected
   but half-rate.
2. For the half-rate case, suspects are the `rmw_uros_ping_agent(300, 1)` call
   blocking the loop, the `spin_some` budget, or WiFi/agent backpressure.
3. Give the Pi 5 relay **and the micro-ROS agent** systemd units so they survive
   reboots. Half of this issue's mystery is things that silently don't restart.
3. Validate `/odometry/filtered` on a straight push **and** on a pivot in place —
   the pivot is the interesting one, because that's where cuVSLAM struggles and
   the gyro should carry it.

## Gate

*To be written when we get here, from real measurements.* It will assert
something like: `/wheel_state` at its true rate, `/odometry/filtered` tracking
through a deliberate cuVSLAM dropout, and `/odom/health` not reporting
`VO_SCALE_OFF` during a slow straight drive.
