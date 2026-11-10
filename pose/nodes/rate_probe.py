#!/usr/bin/env python3
"""rate_probe.py -- how fast a topic publishes, without decoding it.

    python3 rate_probe.py TOPIC [SECONDS]     prints the rate in Hz (0 = nothing)

Replaces `ros2 topic hz` in the ./rover gates. That tool deserializes every
message in Python, and on the loaded Jetson it falls behind on big ones:
measured 2026-09-26, it read raw depth (~800 KB frames) at 11.5 Hz and
nvblox's back-projected cloud at 2.3 Hz -- a "LOW" in ./rover status -- while
a subscriber that did not decode counted 28.4 and 10.5 Hz. This one takes the
messages raw (bytes only), and BEST_EFFORT / VOLATILE, which matches any
publisher's QoS (reliable or not, latched or not).

The rate is from the stamps of arrival: (n - 1) / (last - first), counted for
SECONDS after the first message, so discovery time does not dilute it.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

DISCOVER_S = 5.0


def main():
    topic = sys.argv[1]
    window = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    rclpy.init()
    n = Node('rate_probe')
    end = time.time() + DISCOVER_S
    mtype = None
    while mtype is None and time.time() < end:
        for name, types in n.get_topic_names_and_types():
            if name == topic and types:
                mtype = types[0]
        if mtype is None:
            rclpy.spin_once(n, timeout_sec=0.1)
    if mtype is None:
        print(0)
        return
    arrivals = []
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    n.create_subscription(get_message(mtype), topic, lambda _: arrivals.append(time.monotonic()),
                          qos, raw=True)
    end = time.time() + DISCOVER_S + window
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
        if arrivals and arrivals[-1] - arrivals[0] >= window:
            break
    if len(arrivals) < 2 or arrivals[-1] <= arrivals[0]:
        print(0)
    else:
        print(round((len(arrivals) - 1) / (arrivals[-1] - arrivals[0]), 2))
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
