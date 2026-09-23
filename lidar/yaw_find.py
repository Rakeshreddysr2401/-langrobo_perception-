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
from tf2_ros import Buffer, TransformListener

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


def watch(n, buf):
    """Live readout: hold something in front of the nose and read the answer.

    The one-shot below needs the target placed and left alone. This instead
    prints, once a second, where the nearest object is in the RAW scan and
    where that currently RENDERS on the rover -- so you can hold a box at the
    nose, watch 'renders at' settle near 0 deg, and know the yaw is right
    without describing anything to anyone. When it does not settle near 0, the
    LIDAR_YAW column is the number that would make it.
    """
    # Wait for the transform BEFORE the first row. A TransformListener that has
    # only just subscribed has an empty buffer, so the first sample would print
    # with yaw 0 -- an un-rotated reading in a column whose whole job is to show
    # the rotation, i.e. exactly the wrong number in exactly the wrong place.
    for _ in range(40):
        if buf.can_transform("base_link", "laser", rclpy.time.Time()):
            break
        rclpy.spin_once(n, timeout_sec=0.25)
    else:
        print("      ! base_link -> laser never arrived — is ./rover lidar up?")
    print("      hold a box ~50 cm off the NOSE. Ctrl-C to stop.")
    print(f"      {'nearest':>9s}  {'raw bearing':>12s}  {'renders at':>11s}   {'LIDAR_YAW would be':>19s}")
    while rclpy.ok():
        got = []
        sub = n.create_subscription(LaserScan, "/scan", got.append, qos_profile_sensor_data)
        deadline = n.get_clock().now().nanoseconds + int(2e9)
        while not got and n.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(n, timeout_sec=0.2)
        n.destroy_subscription(sub)
        if not got:
            print("      (no scan)")
            continue
        m = got[-1]
        b, r = bearing_of_nearest(m)
        if b is None:
            print(f"      nothing between {NEAR:.2f} and {FAR:.2f} m")
            continue
        # Where that beam actually lands on the rover, through the live TF.
        # If the transform is not there, SAY SO -- falling back to yaw 0 would
        # print a plausible number that silently means "unrotated", in the one
        # column that exists to show the rotation.
        if not buf.can_transform("base_link", m.header.frame_id, rclpy.time.Time()):
            print(f"      {r:7.2f} m  {math.degrees(b):+11.1f}   "
                  f"no base_link -> {m.header.frame_id} yet")
            rclpy.spin_once(n, timeout_sec=0.6)
            continue
        q = buf.lookup_transform("base_link", m.header.frame_id,
                                 rclpy.time.Time()).transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        shown = wrap(b + yaw)
        where = {0: "FRONT", 1: "LEFT", 2: "BACK", 3: "RIGHT"}[
            int(round(math.degrees(shown) / 90.0)) % 4]
        print(f"      {r:7.2f} m  {math.degrees(b):+11.1f}  {math.degrees(shown):+8.1f} "
              f"{where:<6s} {-b:+8.3f} rad ({math.degrees(-b):+.1f})")
        rclpy.spin_once(n, timeout_sec=0.6)


def main():
    rclpy.init()
    n = Node("yaw_find")
    buf = Buffer()
    TransformListener(buf, n)
    if "--watch" in sys.argv:
        try:
            watch(n, buf)
        except KeyboardInterrupt:
            pass
        return 0

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
        print(f"      then make it permanent: lidar.yaw in description/params.yaml")

    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
