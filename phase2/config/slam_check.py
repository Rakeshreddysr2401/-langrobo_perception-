#!/usr/bin/env python3
"""slam_check.py — is the lidar correction alive, and how big is it right now?

Prints the pose from odometry alone (odom -> base_link, fusion_node) next to
the lidar-corrected pose (map -> base_link, slam_toolbox), and the difference.
Both start coincident; the difference is exactly what dead reckoning got wrong
since bring-up. If it is 0.00 forever, the correction is not running.
"""
import math
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    n = Node("slam_check")
    buf = Buffer(); TransformListener(buf, n)
    deadline = n.get_clock().now().nanoseconds + int(8e9)
    while n.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(n, timeout_sec=0.2)
        if buf.can_transform("map", "base_link", rclpy.time.Time()) and \
           buf.can_transform("odom", "base_link", rclpy.time.Time()):
            break
    else:
        print("      no map -> base_link transform after 8 s (slam_toolbox not publishing, or no /scan)")
        return 1
    m = buf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
    o = buf.lookup_transform("odom", "base_link", rclpy.time.Time()).transform
    c = buf.lookup_transform("map", "odom", rclpy.time.Time()).transform
    print(f"      {'':14s}{'x':>8s}{'y':>8s}{'yaw':>9s}")
    print(f"      {'odom alone':14s}{o.translation.x:8.3f}{o.translation.y:8.3f}{math.degrees(yaw_of(o.rotation)):8.1f}°")
    print(f"      {'lidar-corrected':14s}{m.translation.x:8.3f}{m.translation.y:8.3f}{math.degrees(yaw_of(m.rotation)):8.1f}°")
    print(f"      {'correction':14s}{c.translation.x:8.3f}{c.translation.y:8.3f}{math.degrees(yaw_of(c.rotation)):8.1f}°   (map -> odom)")
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
