#!/usr/bin/env python3
"""yaw_find.py — measure LIDAR_YAW from a target held in front of the nose.

The C1's case does not mark which way its zero beam points, and `./rover lidar`'s
box test only answers it to the nearest quadrant ("front is the one that drops").
A quadrant is not good enough: a scan 10 deg off the body still fights odometry
on every move, and slam_toolbox will happily push that error into map -> odom.

So measure it. Put a flat target (a box, a book) about 50 cm DIRECTLY in front
of the nose, nothing else that close, and run this. It finds the nearest
sustained cluster across several scans, reports its bearing in the RAW laser
frame, and prints the LIDAR_YAW that puts it at 0 deg -- straight ahead.

    LIDAR_YAW = -bearing      (a target physically at 0 deg in base_link
                               appears at bearing + LIDAR_YAW, so we cancel it)

Run it with the CURRENT yaw still applied or not -- it reads the raw scan and
is unaffected either way. Repeat with the target to the LEFT to confirm: the
answer must not move.
"""
import math
import statistics
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

NEAR, FAR = 0.20, 1.50      # where the target is allowed to be
N_SCANS = 10                # averaged, so one dropout cannot move the answer


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def bearing_of_nearest(m):
    """Bearing (rad, laser frame) of the nearest cluster inside [NEAR, FAR]."""
    cand = [(r, m.angle_min + i * m.angle_increment)
            for i, r in enumerate(m.ranges)
            if math.isfinite(r) and NEAR <= r <= FAR]
    if not cand:
        return None, None
    rmin = min(r for r, _ in cand)
    # everything within 8 cm of the closest return is the same object
    face = [(r, a) for r, a in cand if r <= rmin + 0.08]
    # average as unit vectors, so a cluster straddling +/-pi does not average to 0
    x = sum(math.cos(a) for _, a in face)
    y = sum(math.sin(a) for _, a in face)
    return math.atan2(y, x), rmin


def main():
    rclpy.init()
    n = Node("yaw_find")
    got = []
    n.create_subscription(LaserScan, "/scan", got.append, qos_profile_sensor_data)
    deadline = n.get_clock().now().nanoseconds + int(15e9)
    while len(got) < N_SCANS and n.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(n, timeout_sec=0.5)
    if len(got) < 3:
        print(f"      only {len(got)} scans in 15 s — is ./rover lidar up?")
        return 1

    reads = [bearing_of_nearest(m) for m in got]
    reads = [(b, r) for b, r in reads if b is not None]
    if not reads:
        print(f"      nothing between {NEAR:.2f} and {FAR:.2f} m — is the target actually there?")
        return 1

    bx = sum(math.cos(b) for b, _ in reads)
    by = sum(math.sin(b) for b, _ in reads)
    bearing = math.atan2(by, bx)
    spread = max(abs(math.degrees(wrap(b - bearing))) for b, _ in reads)
    dist = statistics.median(r for _, r in reads)

    print(f"      {len(reads)} scans, target at {dist:.2f} m, spread {spread:.1f} deg")
    print(f"      bearing in the raw laser frame : {math.degrees(bearing):+7.1f} deg")
    print(f"      => LIDAR_YAW                   : {-bearing:+7.3f} rad "
          f"({math.degrees(-bearing):+.1f} deg)")
    if spread > 5:
        print(f"      ! spread {spread:.1f} deg is high — something else is inside "
              f"{FAR:.2f} m, or the target moved. Clear the area and re-run.")
    else:
        print(f"      set it:  LIDAR_YAW={-bearing:.3f} ./rover lidar")
        print(f"      then make it permanent in ./rover (LIDAR_YAW default).")

    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
