# Flashing the ESP32

## Serial transport (2026-10-07)

The ESP32 now reaches the Pi 5 over its **USB cable**, not WiFi. The switch is
`#define UROS_SERIAL 1` at the top of the sketch:

- **1 (default)** -- micro-ROS on `Serial` at 115200; the Pi 5 agent opens
  `/dev/ttyUSB0` (`~/ros2_ws/scripts/run_microros.sh`). WiFi and OTA are not
  compiled in, and **every debug print is compiled out**: the cable carries
  micro-ROS frames only, so the serial-monitor check below no longer applies
  (verify with `./rover wheels` instead). No WiFi password needed.
- **0** -- the old WiFi UDP link + OTA + debug prints, as described below. Run
  the Pi 5 agent with `run_microros.sh udp` to match.

**Flash from the Pi 5 itself** (no laptop, no IDE; set up 2026-10-07 with
arduino-cli at `~/.local/bin`, esp32 core 3.3.12 trimmed to the classic ESP32,
ESP32Encoder 5.0.0, micro_ros_arduino jazzy in `~/Arduino/libraries`):

```bash
sudo systemctl stop langrobo-microros          # the agent holds the port
mkdir -p /tmp/rover_firmware_v2 && scp rakhi-jetson.local:rover/phase1/firmware/rover_firmware_v2.ino /tmp/rover_firmware_v2/
arduino-cli compile --fqbn esp32:esp32:esp32 /tmp/rover_firmware_v2
arduino-cli upload  --fqbn esp32:esp32:esp32:UploadSpeed=921600 -p /dev/ttyUSB0 /tmp/rover_firmware_v2
sudo systemctl start langrobo-microros
```

The agent resets the ESP32 every time it starts (`run_microros.sh`): over serial
the board does not recover on its own from an agent that went away mid-session.
Benign at boot: eight `gpio_pullup_en ... input-only pad has no internal PU`
lines from ESP32Encoder on pins 34-39 -- the pull-ups are off by design.

## Why this needs doing

**One line changed on 2026-08-15: `/wheel_state` is now a RELIABLE publisher.**
That is the whole fix for the 1 Hz problem, and it needs a flash to take effect.

`/wheel_state` was arriving at a metronomic **1.000 Hz** (±2.9 ms measured at the
agent) while `loop()` was publishing at 20 Hz. The code was never wrong — a
best-effort message in micro-ROS is written into an output stream buffer and only
reaches the wire when the XRCE session next runs, so 19 of every 20 messages sat
in that buffer. Reliable streams are flushed inside `rmw_publish`, so they leave
at once.

Everything else about the board was already proven healthy:

- it consumed **19.71 of 20** `/cmd_vel` per second with zero lag — `loop()` is fast
- both encoders verified by hand, correct channels, correct sign
- one stable micro-ROS session at `192.168.1.3`, no reconnect loop
- ping 2.4–10 ms at −36 dBm; the WiFi is not the problem

Full diagnosis, including everything that was ruled out, is in `TODO.md` §1.

**If this flash does not raise the rate**, the remaining possibility is that the
board emits 20 packets/s and the agent discards them. Settle it with:

```bash
ssh -t 192.168.1.16 "sudo timeout 10 tcpdump -i any -nn 'src 192.168.1.3 and udp' -w /dev/null"
```

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

Then Sketch → Include Library → **Add .ZIP Library** → pick that file. Use the
**jazzy** branch (what the firmware is built and tested with). It does not have to match the
Pi 5's distro: the ESP32 speaks XRCE to the standalone agent, which bundles its own Fast DDS
(3.6.2 = Lyrical's); verified 2026-10-25 with the Pi 5 on Lyrical, /wheel_state 20 Hz.

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
