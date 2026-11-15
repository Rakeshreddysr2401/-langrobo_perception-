#!/usr/bin/env python3
"""Do the Pi 5's clock and this Jetson's agree?   python3 clock_check.py [seconds]

The Pi 5 stamps /sonar/left with ITS clock; nav2's collision monitor places each
reading in time with OURS (transform_tolerance 0.2 s). Each machine sets its
clock from the internet by itself, and this Jetson has no clock battery: it
boots in 1970 until it reaches a time server. With the clocks apart every sonar
reading is dropped as too old / from the future, silently -- every rate gate
stays green.

Measured on the data itself, no ssh: the smallest (arrival time - stamp) over
the window is the clock difference plus the best WiFi delay (a few ms).

Prints one gate line; exit 0 = within LIMIT_S, 1 = apart, 2 = no sonar message.
"""
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

LIMIT_S = 0.10      # half of the collision monitor's transform_tolerance
TOPIC = '/sonar/left'


def verdict(diffs):
    """(exit code, text) from the (arrival - stamp) samples, in seconds."""
    if not diffs:
        return 2, 'no message on %s (sonar node down?) -- not checked' % TOPIC
    d = min(diffs, key=abs)
    if abs(d) <= LIMIT_S:
        return 0, '%+.0f ms   (want within %.0f)   OK' % (d * 1e3, LIMIT_S * 1e3)
    who = 'behind' if d > 0 else 'ahead of'
    return 1, ('%+.1f s   FAIL -- the Pi 5 clock is %s this one: nav2 drops the sonars. '
               'On both: timedatectl (is "System clock synchronized: yes"?)' % (d, who))


def main():
    window = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    rclpy.init()
    node = rclpy.create_node('clock_check')
    diffs = []

    def on_range(m):
        diffs.append(time.time() - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))

    node.create_subscription(Range, TOPIC, on_range, qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < window + 2.0 and not (diffs and time.time() - t0 > window):
        rclpy.spin_once(node, timeout_sec=0.2)
    code, text = verdict(diffs)
    print('      %-26s %s' % ('Pi 5 clock vs Jetson', text))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
