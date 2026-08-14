# Flashing the ESP32

## Why this needs doing

`/wheel_state` arrives at a metronomic **1.000 Hz** (min 0.989, max 1.012,
σ = 3.4 ms over 55 samples). That regularity rules out packet loss — it is a
timer.

The board is healthy in every other respect:

- `langrobo-microros.service` on the Pi 5 exists, is enabled, and starts at boot
- the ESP32 holds **one stable session** (`_CREATED_BY_BARE_DDS_APP_`), so it is
  `AGENT_CONNECTED`, not reconnecting in a loop
- it answers at `192.168.1.3` (it moved off `.12` — DHCP)

But `rover_firmware_v2.ino` publishes `/wheel_state` **only** in the
`AGENT_CONNECTED` branch, at `EXECUTE_EVERY_N_MS(50, …)` = 20 Hz. No state in it
publishes at 1 Hz, and the macro's `static` is correctly scoped per expansion.

**So the flashed binary is not built from this source.** Reflashing is the fix.

## What it unblocks

Phase 1's drift gate. On a 2 m out-and-back the fused pose lands 12.2 cm from
the start against a 10 cm bar, and the residual is **distance, not heading**: the
return leg registered 186.5 cm against the outbound 198.0, so reversing
under-reads by ~6%. cuVSLAM is currently the only translation source, so nothing
contradicts it. Encoders measure distance without caring about visual texture or
direction of travel — they are exactly the missing cross-check.

---

## One-time setup in Arduino IDE

**1. ESP32 board support.** File → Preferences → *Additional Board Manager URLs*:

```
https://espressif.github.io/arduino-esp32/package_esp32_index.json
```

Then Tools → Board → Boards Manager → install **esp32 by Espressif**, version
**3.x**. The firmware uses the 3.x PWM API (`ledcAttach` per pin), so a 2.x core
will not compile.

**2. micro_ros_arduino.** Not in Library Manager — install from ZIP. Download:

```
https://github.com/micro-ROS/micro_ros_arduino/archive/refs/heads/jazzy.zip
```

Then Sketch → Include Library → **Add .ZIP Library** → pick that file. The branch
must be **jazzy**, matching the Pi 5's ROS 2 distro.

**3. ESP32Encoder.** Library Manager → search `ESP32Encoder` → install the one by
**madhephaestus**.

---

## Every time you flash

**1. Folder name.** Arduino requires the sketch folder to match the file, so put
`rover_firmware_v2.ino` inside a folder called `rover_firmware_v2/`.

**2. Set the WiFi password.** Line 66:

```c
const char* WIFI_PASS  = "YOUR_WIFI_PASSWORD";   // <-- set locally
```

Put the real password in **locally only**. Do not commit it back.

Everything else is already correct and should not be touched:

| setting | value | why |
|---|---|---|
| `AGENT_IP` | `192.168.1.16` | the Pi 5's wlan0, where the agent runs |
| `AGENT_PORT` | `8888` | matches `run_microros.sh` |
| `WHEEL_DIAMETER_M` | `0.085f` | confirmed with a tape 2026-08-15 |
| `WHEEL_BASE_M` | `0.34f` | confirmed with a tape 2026-08-15 |
| `ENCODER_CPR` | `1560.0f` | 13 PPR × 4 (quad) × 30 gear |

**3. Board settings.** Tools →

- Board: **ESP32 Dev Module**
- Upload Speed: 921600 (drop to 115200 if it fails)
- Port: whatever appears when you plug the board in

**4. Upload.**

---

## Verifying, in order

**1. Serial monitor, 115200 baud.** This is the most useful check and it needs
nothing else running. `DEBUG_SERIAL` is on, so the control task prints at ~2 Hz:

```
[READY] waiting for micro-ROS agent
[uROS] CONNECTED
IN vx=0.00 wz=0.00 | OUT velL=0.00 velR=0.00 | odom(x=0.00 y=0.00 th=0.00) dt=0.020
```

- `[uROS] CONNECTED` means it reached the agent.
- `dt=0.020` confirms the control loop really is running at 50 Hz.
- **Spin a wheel by hand** and the matching `velL` or `velR` must move. That
  proves the encoders, independently of ROS. If one side stays at 0.00, that
  wheel's encoder wiring is the problem, not the software.

**2. The rate, from the Jetson:**

```bash
./rover wheels
```

Wants **≥ 15 Hz**. It should read about 20.

**3. In the instrument:**

```bash
./rover compare --plain
```

The `wheels` row should come alive. Push the rover by hand and its `x` should
climb alongside `cuvslam`'s.

---

## Safety

Hand-pushing stays safe after flashing. `pidStep()` returns 0 whenever the target
velocity is under 0.01, and `driveSide()` with zero duty pulls both BTS7960 pins
low — that is **coast, not brake**. The wheels free-wheel and the PID will not
fight you. Motion is also gated on `controlEnabled` *and* a `/cmd_vel` newer than
500 ms, so an idle rover cannot creep.
