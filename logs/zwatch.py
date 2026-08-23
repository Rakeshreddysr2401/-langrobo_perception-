"""Watch cuVSLAM's z, and whether it drifts or jumps.

Gradual drift points at geometry -- baseline, extrinsics, rectification.
A step change points at a discrete event -- a frame gap, a timestamp jump, a
tracking loss. They need completely different fixes, and TODO 21 has never
distinguished them.
"""
import math, time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
n = rclpy.create_node("zwatch")
rows = []
def cb(m):
    p = m.pose.pose.position
    t = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
    rows.append((t, p.x, p.y, p.z))
n.create_subscription(Odometry, "/vo/odom", cb, qos_profile_sensor_data)

DUR = 90.0
print("  watching /vo/odom for %ds — leave the rover STILL" % DUR)
print()
print("      t      x        y        z         dz/s     gap")
print("   " + "-"*56)
t0 = time.time(); last = 0.0; prev = None; worst_gap = 0.0
while rclpy.ok() and time.time()-t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.1)
    if not rows or time.time()-last < 5.0:
        continue
    last = time.time()
    t, x, y, z = rows[-1]
    dz = 0.0
    if prev:
        dt = t - prev[0]
        dz = (z - prev[3])/dt if dt > 0 else 0.0
    # biggest inter-frame gap since the last print
    gaps = [rows[i][0]-rows[i-1][0] for i in range(max(1,len(rows)-200), len(rows))]
    gap = max(gaps) if gaps else 0.0
    worst_gap = max(worst_gap, gap)
    print("   %5.0fs  %+7.3f %+7.3f %+8.3f   %+7.4f  %5.0f ms"
          % (time.time()-t0, x, y, z, dz, gap*1000))
    prev = (t, x, y, z)

print()
if len(rows) > 2:
    zs = [r[3] for r in rows]
    steps = [abs(zs[i]-zs[i-1]) for i in range(1, len(zs))]
    big = [(i, s) for i, s in enumerate(steps) if s > 0.05]
    print("  %d samples over %.0f s" % (len(rows), rows[-1][0]-rows[0][0]))
    print("  z went %+.3f -> %+.3f  (net %+.3f m)" % (zs[0], zs[-1], zs[-1]-zs[0]))
    print("  largest single-sample z step: %.4f m" % max(steps))
    print("  steps over 5 cm: %d" % len(big))
    print("  worst frame gap: %.0f ms" % (worst_gap*1000))
    print()
    if max(steps) > 0.05:
        print("  -> JUMPS. Something discrete is doing this, not accumulating error.")
    else:
        print("  -> DRIFT. No single step over 5 cm; the error accumulates smoothly.")
rclpy.shutdown()
