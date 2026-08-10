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
- **Encoders are the open problem.** `/wheel_state` publishes at exactly
  **10.0 Hz**. The firmware asks for 50 ms → 20 Hz at
  `firmware/rover_firmware_v2.ino:391` and delivers half that. Cause unknown.
  > Older notes say "1 Hz, still on old firmware". **That is out of date** — a
  > reflash has happened. The remaining half-rate is a genuine bug worth finding.
- The Pi 5 relay (`ESP32 /wheel_state → /wheel_odom`) has **no systemd unit**, so
  it dies on every reboot and must be restarted by hand.
- `config/ekf.yaml` already has `odom1: /wheel_odom` wired.

## Likely work

1. Find why the ESP32 telemetry lands at half rate — suspects are the
   `rmw_uros_ping_agent(300, 1)` call blocking the loop, `spin_some` budget, or
   WiFi/agent backpressure.
2. Give the Pi 5 relay a systemd unit so it survives reboots.
3. Validate `/odometry/filtered` on a straight push **and** on a pivot in place —
   the pivot is the interesting one, because that's where cuVSLAM struggles and
   the gyro should carry it.

## Gate

*To be written when we get here, from real measurements.* It will assert
something like: `/wheel_state` at its true rate, `/odometry/filtered` tracking
through a deliberate cuVSLAM dropout, and `/odom/health` not reporting
`VO_SCALE_OFF` during a slow straight drive.
