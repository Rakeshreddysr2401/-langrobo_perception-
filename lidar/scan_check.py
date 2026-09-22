#!/usr/bin/env python3
"""scan_check.py — one /scan message, read out in the ROBOT's four directions.

`ros2 topic hz` proves the driver is alive; it cannot say whether the scan is
pointing the right way. This prints the range straight ahead, left, behind and
right of base_link, so the box test (`./rover lidar`) has a number to look at.

The four values are taken in the `laser` frame with LIDAR_YAW already applied
by the static TF, so once the yaw is right, "front" here IS the nose.

It also asserts that `base_link -> laser` exists at all. A scan nothing can
transform is worse than no scan: the rate gate stays green and the failure is
invisible everywhere except RViz, which simply draws nothing.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener

NAMES = ("front (+x)", "left  (+y)", "back  (-x)", "right (-y)")


def main():
    rclpy.init()
    n = Node("scan_check")
    got = []
    n.create_subscription(LaserScan, "/scan", lambda m: got.append(m), qos_profile_sensor_data)
    buf = Buffer(); TransformListener(buf, n)
    deadline = n.get_clock().now().nanoseconds + int(8e9)
    while not got and n.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(n, timeout_sec=0.5)
    if not got:
        print("      no /scan message in 8 s")
        return 1
    m = got[0]

    # The rate does NOT prove the frame is usable. /scan is published by the
    # driver whether or not base_link -> laser exists, and when it does not,
    # RViz draws nothing and slam_toolbox cannot run -- with every rate green.
    # That is exactly what ./rover pose's kill_match used to cause. Check it.
    # Give the listener a few seconds: a static TF is latched, but a listener
    # that has only just subscribed has not necessarily received it yet.
    tf_deadline = n.get_clock().now().nanoseconds + int(4e9)
    tf_ok = False
    while n.get_clock().now().nanoseconds < tf_deadline:
        if buf.can_transform("base_link", m.header.frame_id, rclpy.time.Time()):
            tf_ok = True
            break
        rclpy.spin_once(n, timeout_sec=0.2)
    if not tf_ok:
        print(f"      ! /scan is alive but base_link -> {m.header.frame_id} does NOT exist.")
        print("        Nothing can use this scan: RViz draws no ring and slam_toolbox")
        print("        cannot run. Re-run ./rover lidar.")

    valid = [r for r in m.ranges if math.isfinite(r) and r > 0]
    print(f"      {len(m.ranges)} beams, {math.degrees(m.angle_increment):.2f} deg apart, "
          f"{100 * len(valid) / len(m.ranges):.0f}% valid, "
          f"nearest {min(valid):.2f} m, farthest {max(valid):.2f} m")
    for name, deg in zip(NAMES, (0, 90, 180, -90)):
        i = int(round((math.radians(deg) - m.angle_min) / m.angle_increment)) % len(m.ranges)
        # median of the 7 beams around the direction, so one dropout cannot fool us
        win = sorted(m.ranges[(i + d) % len(m.ranges)] for d in range(-3, 4)
                     if math.isfinite(m.ranges[(i + d) % len(m.ranges)]) and m.ranges[(i + d) % len(m.ranges)] > 0)
        v = f"{win[len(win) // 2]:.2f} m" if win else "  --  "
        print(f"      {name}: {v}")
    n.destroy_node()
    rclpy.shutdown()
    return 0 if tf_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
