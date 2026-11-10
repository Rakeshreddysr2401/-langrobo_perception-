# main.py -- KINO's front sonar board (MicroPython 1.24 on its own ESP32), runs at power-up.
#
# Two HC-SR04-type sensors at the front corners (HARDWARE.md section 2b; run at 3.3 V):
#   S1  TRIG GPIO5,  ECHO GPIO19        S2  TRIG GPIO18, ECHO GPIO21
# Pings S1, then S2 -- never both at once, so one never hears the other's echo --
# ~15 Hz each, and prints one line per reading on USB serial (115200):
#
#     S1 1234      millimetres to the nearest echo
#     S2 -1        no echo within TIMEOUT_US (nothing within ~4 m, or it soaked up the ping)
#
# The Pi 5's sonar node reads these lines and publishes /sonar/left, /sonar/right
# (sensor_msgs/Range). Which of S1/S2 is LEFT is the node's job (params.yaml
# sonar.left_is_s1), so a swapped mount is one setting, not a re-flash.
# Updated over the same USB link: Ctrl-C stops it, the file is replaced, reset.
from machine import Pin, time_pulse_us
import time

VERSION = 'kino-sonar 1'
TRIG = (Pin(5, Pin.OUT, value=0), Pin(18, Pin.OUT, value=0))
ECHO = (Pin(19, Pin.IN), Pin(21, Pin.IN))
TIMEOUT_US = 25000          # ~4.3 m there and back at 343 m/s
PERIOD_MS = 33              # one ping every 33 ms, alternating: ~15 Hz per sensor
SOUND_MM_PER_US = 0.343


def ping(i):
    TRIG[i].value(0)
    time.sleep_us(3)
    TRIG[i].value(1)
    time.sleep_us(10)
    TRIG[i].value(0)
    t = time_pulse_us(ECHO[i], 1, TIMEOUT_US)
    return -1 if t < 0 else int(t * SOUND_MM_PER_US / 2)


print('#', VERSION, 'S1 TRIG5/ECHO19 S2 TRIG18/ECHO21 mm')
i = 0
while True:
    t0 = time.ticks_ms()
    print('S%d %d' % (i + 1, ping(i)))
    i ^= 1
    left = PERIOD_MS - time.ticks_diff(time.ticks_ms(), t0)
    if left > 0:
        time.sleep_ms(left)
