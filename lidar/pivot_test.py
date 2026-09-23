#!/usr/bin/env python3
"""pivot_test.py — how much does a turn actually shift x,y? Ask the walls.

THE PROBLEM
    Turning is reported to shift the rover's x,y (TODO 37). It has never been
    measured, because every instrument that could measure it is the instrument
    under suspicion: odometry grading its own pivot will always say the pivot
    was clean. The gyro measures rotation and cannot see translation at all.

THE INSTRUMENT
    The room. Now that LIDAR_YAW is measured, a scan can be put into base_link,
    and two scans either side of a turn can be registered against each other.
    That registration gives the rigid motion the ROOM says happened:

        p_base(0) ~ R(dtheta) p_base(1) + d

    dtheta is the true rotation and d is the true body-frame displacement. For
    a pivot in place d should be zero; whatever it actually is, is the answer.

    Odometry's own dtheta and d are printed beside it. The difference between
    the two rotations is the heading scale error; the lidar's d is the drift.

    ICP is started FROM odometry's rotation, which is legitimate -- it is a
    refinement, and what gets reported is how far the refinement had to move.

    ./rover pivot            +90, -90, +180, -180
    ./rover pivot 90 -90     any list of degrees
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

import os
# 1.5, not 0.5: TODO 43 measured pivots reaching ~7% of ANY commanded rate --
# the firmware's PI saturates to full duty and the motors stall against scrub.
# A low command does not give a gentler pivot, only a slower one.
WZ = float(os.environ.get("PIVOT_WZ", "1.5"))
# Deadlines assume the rate the rover ACTUALLY reaches, not the commanded one.
# The first version budgeted |angle|/WZ and every turn "stalled" at 18 s while
# still turning. Abort instead on NO PROGRESS for STALL_S.
MIN_RATE = 0.03          # rad/s, the slowest turn still worth waiting for
STALL_S = 8.0
# The spin sweeps a circle of this radius: half the 46 x 42 footprint diagonal
# plus 5 cm. Anything the lidar sees inside it gets hit.
SWEEP_R = math.hypot(0.23, 0.21) + 0.05
RANGE_LO, RANGE_HI = 0.20, 6.0
SETTLE = 1.5


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def scan_in_base(scan, lx, lz_yaw):
    """Scan points expressed in base_link, using the calibrated mount."""
    r = np.asarray(scan.ranges, dtype=np.float64)
    a = scan.angle_min + np.arange(r.size) * scan.angle_increment
    ok = np.isfinite(r) & (r > RANGE_LO) & (r < RANGE_HI)
    q = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)
    c, s = math.cos(lz_yaw), math.sin(lz_yaw)
    R = np.array([[c, -s], [s, c]])
    return q @ R.T + np.array([lx, 0.0])


def icp(src, dst, theta0, iters=60):
    """Rigid 2D fit: dst ~ R(theta) src + t, started at theta0."""
    tree = cKDTree(dst)
    th, t = theta0, np.zeros(2)
    used = 0
    for _ in range(iters):
        c, s = math.cos(th), math.sin(th)
        R = np.array([[c, -s], [s, c]])
        cur = src @ R.T + t
        d, idx = tree.query(cur)
        keep = d < max(0.20, np.percentile(d, 70))
        used = int(keep.sum())
        if used < 40:
            return None
        P, Q = src[keep], dst[idx[keep]]
        pc, qc = P.mean(axis=0), Q.mean(axis=0)
        H = (P - pc).T @ (Q - qc)
        U, _, Vt = np.linalg.svd(H)
        D = np.diag([1.0, np.linalg.det(Vt.T @ U.T)])
        Rn = Vt.T @ D @ U.T
        th_n = math.atan2(Rn[1, 0], Rn[0, 0])
        t_n = qc - Rn @ pc
        if abs(wrap(th_n - th)) < 1e-6 and np.linalg.norm(t_n - t) < 1e-6:
            th, t = th_n, t_n
            break
        th, t = th_n, t_n
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s], [s, c]])
    d, _ = tree.query(src @ R.T + t)
    return th, t, float(np.median(d)), used


class Pivot(Node):
    def __init__(self):
        super().__init__("pivot_test")
        self.scan = None
        self.odom = None
        self.create_subscription(LaserScan, "/scan", self._s, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._o, 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.buf = Buffer()
        TransformListener(self.buf, self)

    def _s(self, m): self.scan = m
    def _o(self, m): self.odom = m

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def ready(self, sec=12.0):
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.scan is not None and self.odom is not None and \
               self.buf.can_transform("base_link", "laser", rclpy.time.Time()):
                return True
        return False

    def mount(self):
        q = self.buf.lookup_transform("base_link", "laser",
                                      rclpy.time.Time()).transform
        return q.translation.x, yaw_of(q.rotation)

    def pose(self):
        p = self.odom.pose.pose
        return np.array([p.position.x, p.position.y]), yaw_of(p.orientation)

    def stop(self):
        for _ in range(6):
            self.cmd.publish(Twist())
            self.spin(0.05)

    def turn(self, deg):
        """Closed loop on odom yaw. Returns None if it stalls."""
        target = math.radians(deg)
        _, y0 = self.pose()
        t = Twist()
        t.angular.z = math.copysign(WZ, target)
        deadline = time.time() + min(abs(target) / MIN_RATE + 10.0, 120.0)
        turned = 0.0
        prev = y0
        best, best_t = 0.0, time.time()
        while rclpy.ok():
            self.cmd.publish(t)
            self.spin(0.05)
            _, y = self.pose()
            turned += wrap(y - prev)
            prev = y
            if abs(turned) >= abs(target):
                break
            if abs(turned) > best + math.radians(1.0):
                best, best_t = abs(turned), time.time()
            if time.time() - best_t > STALL_S or time.time() > deadline:
                self.stop()
                print(f"        stalled at {math.degrees(turned):+.1f} of {deg:+.0f} deg "
                      f"(no progress for {STALL_S:.0f} s)")
                return None
        self.stop()
        self.spin(SETTLE)
        return turned


def main():
    rclpy.init()
    n = Pivot()
    try:
        if not n.ready():
            print("      need /scan, /odom and base_link -> laser")
            return 1
        lx, lyaw = n.mount()
        print(f"      mount: x {lx:.3f} m, yaw {math.degrees(lyaw):+.2f} deg")
        near = float(np.min(np.linalg.norm(scan_in_base(n.scan, lx, lyaw), axis=1)))
        print(f"      nearest thing to the centre: {near:.2f} m "
              f"(the spin sweeps {SWEEP_R:.2f} m)")
        if near < SWEEP_R:
            print("      ✗ something is inside the spin circle — not turning. Clear it.")
            return 1

        angles = [float(a) for a in sys.argv[1:]] or [90, -90, 180, -180]
        print(f"      commanding {WZ:.2f} rad/s. Turns: "
              f"{', '.join(f'{a:+.0f}' for a in angles)} deg")
        print()
        print(f"      {'turn':>6s} {'odom deg':>9s} {'lidar deg':>10s} {'err':>7s}   "
              f"{'SHIFT x,y (cm)':>16s} {'|shift|':>8s} {'resid':>7s}")

        rows = []
        for a in angles:
            n.spin(0.6)
            s0 = scan_in_base(n.scan, lx, lyaw)
            p0, y0 = n.pose()
            got = n.turn(a)
            if got is None:
                continue
            n.spin(0.4)
            s1 = scan_in_base(n.scan, lx, lyaw)
            p1, y1 = n.pose()

            d_odom = wrap(y1 - y0)
            r = icp(s1, s0, d_odom)
            if r is None:
                print(f"      {a:+6.0f}   ICP could not register the two scans")
                continue
            th, t, resid, used = r
            err = math.degrees(wrap(th - d_odom))
            shift = float(np.linalg.norm(t))
            print(f"      {a:+6.0f} {math.degrees(d_odom):+9.2f} {math.degrees(th):+10.2f} "
                  f"{err:+7.2f}   {t[0]*100:+7.1f},{t[1]*100:+7.1f} {shift*100:7.1f} "
                  f"{resid*100:6.1f}")
            rows.append((a, math.degrees(d_odom), math.degrees(th), err, shift, resid))

        if not rows:
            print("      no usable turns")
            return 1

        print()
        scale = [abs(r[2]) / abs(r[1]) for r in rows if abs(r[1]) > 1]
        print(f"      heading error : {min(r[3] for r in rows):+.2f} to "
              f"{max(r[3] for r in rows):+.2f} deg   "
              f"(odom/lidar scale {np.mean(scale):.4f})")
        print(f"      pivot shift   : {min(r[4] for r in rows)*100:.1f} to "
              f"{max(r[4] for r in rows)*100:.1f} cm   "
              f"(mean {np.mean([r[4] for r in rows])*100:.1f} cm)")
        print()
        print("      SHIFT is how far the rover's centre actually moved during a turn")
        print("      it believes was in place. That is TODO 37, with a number.")
        return 0
    finally:
        try:
            n.stop()
        except Exception:
            pass
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
