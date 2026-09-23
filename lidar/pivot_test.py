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
# The spin sweeps a circle of this radius: the furthest corner of the measured
# 36 x 38 envelope (nav2.yaml's footprint) plus 5 cm. Anything the lidar sees
# inside it gets hit.
CORNERS = np.array([[0.182, 0.19], [0.182, -0.19], [-0.178, -0.19], [-0.178, 0.19]])
SWEEP_R = float(np.max(np.linalg.norm(CORNERS, axis=1))) + 0.05

# LEFT-ANCHORED TURNS (PIVOT_ANCHOR=left). Commanded as a pure wz, this rover
# picks between two behaviours unpredictably: usually the left wheels stay
# planted and it rotates about the left tyres, sometimes the left side joins in
# and it rotates about the centre (2026-09-23: 7 of 8, then a -90 that pivoted
# near the centre and landed a planned turn 35 cm off). A planner cannot plan
# around a coin flip. The firmware mixes wL = vx - wz*B/2 and switches a side
# OFF (duty 0, braked) when its target is under 0.01 m/s, so commanding
# vx = wz*B/2 makes the left target exactly zero and removes the choice: the
# right side swings the rover about the planted left tyres every time. B is the
# FIRMWARE's WHEEL_BASE_M, not the physical track -- it has to cancel the
# firmware's own arithmetic.
FW_HALF_B = 0.17
ANCHOR = os.environ.get("PIVOT_ANCHOR", "none").strip().lower()


# "left" -- left side OFF -- was tried first and is WRONG: an unpowered left
# side does not stay planted, it free-rolls behind the right. Measured
# 2026-09-23: the rover swung about a point 75 cm to its left, ~60 cm of slide
# per 45 deg. Kept only so that result can be reproduced.
#
# "hold" is the fix: give the left side a small target, HOLD_V against the turn
# direction, so the firmware's PI actively holds it near zero. It cannot then
# free-roll forward (the "left" failure), and it cannot reverse at full speed
# either (the occasional centre-pivot that made pure wz a coin flip). It also
# stops the left motor sitting at full duty against a 0.255 m/s target it
# cannot reach, which is what a pure-wz turn asks of it for seconds at a time.
HOLD_V = 0.02


def turn_twist(wz):
    """Twist for a turn at wz, in the mode ANCHOR selects."""
    t = Twist()
    t.angular.z = wz
    if ANCHOR == "left":
        t.linear.x = wz * FW_HALF_B
    elif ANCHOR == "hold":
        # wL = vx - wz*B/2 = -sign(wz) * HOLD_V
        t.linear.x = wz * FW_HALF_B - math.copysign(HOLD_V, wz)
    return t

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
        t = turn_twist(math.copysign(WZ, target))
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
        # A held-left turn does not spin about the centre: it swings about a
        # point 28 cm to the left, and the far corner sweeps ~0.55 m from it.
        # The centre circle above misses the right-hand side of that sweep.
        if ANCHOR == "hold":
            piv = np.array([0.017, 0.277])
            need = float(np.max(np.linalg.norm(CORNERS - piv, axis=1))) + 0.05
            got = float(np.min(np.linalg.norm(scan_in_base(n.scan, lx, lyaw) - piv, axis=1)))
            print(f"      nearest thing to the pivot : {got:.2f} m (the swing sweeps {need:.2f} m)")
            if got < need:
                print("      ✗ something is inside the swing — not turning. Clear it.")
                return 1

        angles = [float(a) for a in sys.argv[1:]] or [90, -90, 180, -180]
        print(f"      turn mode: {dict(left='LEFT OFF (free-rolls; kept for the record)', hold='LEFT HELD near zero').get(ANCHOR, 'pure wz')}")
        print(f"      commanding {WZ:.2f} rad/s. Turns: "
              f"{', '.join(f'{a:+.0f}' for a in angles)} deg")
        print()
        # Two shifts, because they answer different questions. LIDAR is where
        # the centre REALLY went (the room says so). ODOM is where the pose
        # thinks it went. If they agree, the slide is real motion that the pose
        # tracks, and a goal-seeking drive simply corrects it. Only the GAP
        # between them is localization error -- that is the column that decides
        # whether "go back to 0,0,0" lands on 0,0,0.
        print(f"      {'turn':>6s} {'odom deg':>9s} {'lidar deg':>10s} {'err':>6s}  "
              f"{'LIDAR x,y cm':>14s} {'ODOM x,y cm':>14s} {'GAP cm':>7s} {'resid':>6s}")

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
            # odometry's own displacement, in the body frame the turn STARTED in
            # -- the same frame the ICP translation t is expressed in
            dw = p1 - p0
            c, sn = math.cos(-y0), math.sin(-y0)
            od = np.array([c * dw[0] - sn * dw[1], sn * dw[0] + c * dw[1]])
            gap = float(np.linalg.norm(t - od))
            print(f"      {a:+6.0f} {math.degrees(d_odom):+9.2f} {math.degrees(th):+10.2f} "
                  f"{err:+6.2f}  {t[0]*100:+6.1f},{t[1]*100:+6.1f} "
                  f"{od[0]*100:+6.1f},{od[1]*100:+6.1f} {gap*100:7.1f} {resid*100:6.1f}")
            # The point the rover ACTUALLY rotated about, in the start body
            # frame: a rotation th about P moves the centre by (I - R(th)) P,
            # so P = (I - R(th))^-1 t. Ill-conditioned for small turns (det =
            # 2 - 2cos th), so only reported from ~30 deg up.
            piv = None
            if abs(th) > math.radians(30):
                c, sn = math.cos(th), math.sin(th)
                A = np.array([[1 - c, sn], [-sn, 1 - c]])
                piv = np.linalg.solve(A, t)
            rows.append((a, math.degrees(d_odom), math.degrees(th), err, shift, resid, gap, piv))

        if not rows:
            print("      no usable turns")
            return 1

        print()
        scale = [abs(r[2]) / abs(r[1]) for r in rows if abs(r[1]) > 1]
        print(f"      heading error : {min(r[3] for r in rows):+.2f} to "
              f"{max(r[3] for r in rows):+.2f} deg   "
              f"(odom/lidar scale {np.mean(scale):.4f})")
        print(f"      real slide    : {min(r[4] for r in rows)*100:.1f} to "
              f"{max(r[4] for r in rows)*100:.1f} cm   "
              f"(mean {np.mean([r[4] for r in rows])*100:.1f} cm)   <- the rover moved")
        print(f"      pose error    : {min(r[6] for r in rows)*100:.1f} to "
              f"{max(r[6] for r in rows)*100:.1f} cm   "
              f"(mean {np.mean([r[6] for r in rows])*100:.1f} cm)   <- odom did not see it")
        piv = [r[7] for r in rows if r[7] is not None]
        if piv:
            P = np.mean(piv, axis=0)
            spread = max(float(np.linalg.norm(q - P)) for q in piv)
            print(f"      pivot point   : x {P[0]*100:+.1f} cm, y {P[1]*100:+.1f} cm from the centre "
                  f"(spread {spread*100:.1f} cm over {len(piv)} turns)")
            print(f"                      -> PIVOT_X={P[0]:.3f} PIVOT_Y={P[1]:.3f}")
        print()
        print("      A big slide with a small pose error is fine: the pose knows, and")
        print("      driving to a goal corrects it. A big POSE ERROR is what stops the")
        print("      rover landing back on 0,0,0.")
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
