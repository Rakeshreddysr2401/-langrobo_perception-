#!/usr/bin/env python3
"""pivot_goto.py — drive to an exact pose on a rover that cannot pivot about its centre.

THE PROBLEM IT WORKS AROUND (TODO 43)
    Commanded to turn in place, this rover does not. The left side cannot
    reverse against the scrub, so the right side swings the rover around the
    left tyres -- USUALLY. Sometimes the left side does get going and it
    pivots near the centre instead, and a planner cannot plan around a coin
    flip (a planned -90 landed 35 cm off that way).

    So turns are made deterministic first: the left side is HELD near zero
    (see ANCHOR), and every turn then rotates about one point,

        P = (+0.017, +0.277) m    from base_link (ahead, left)

    measured over four 45 deg turns, both directions, spread 1.5 cm, slides
    21.0-21.6 cm each. Nothing in (vx, wz) can make a side produce torque it
    does not have, so this does not try to fix the turn. It plans around it.

THE ONE IDENTITY EVERYTHING HERE RESTS ON
    Rotate th1 about P, drive straight d, rotate th2 about P. Starting at the
    origin facing +x, the centre ends at

        (I - R(th1 + th2)) P  +  d * (cos th1, sin th1)

    The intermediate terms cancel exactly: the slide depends only on the TOTAL
    rotation, never on how it is split. So to reach a target T with heading
    psi, the straight leg must be

        d * u(th1)  =  T - (I - R(psi)) P

    That fixes d and th1 (up to the sign of d, which picks forward or back),
    and th2 = psi - th1. Any pose is three primitives away, and a turn "in
    place" is just the target T = 0. For +90 deg: (with the pure-wz pivot) rotate
    +34, reverse 30 cm, rotate +56, and the centre lands where it started.

CLOSED LOOP, ON THE ROOM
    The plan uses the LiDAR-corrected pose (map -> base_link, slam_toolbox),
    and every iteration re-plans from where the rover ACTUALLY ended up. So a
    turn that does not follow the model -- the eighth turn above pivoted near
    the centre -- is caught and corrected on the next pass rather than trusted.
    Inside a segment it steers on /odom, which is smooth at 20 Hz; between
    segments it plans on the map pose, which is right.

SAFETY, in this order
    It refuses to start without a map frame, refuses a straight leg over
    MAX_LEG, checks the LiDAR for anything within 8 cm of the area the
    planned turn will actually sweep (about that direction's pivot) and in
    the straight corridor before each segment, aborts a segment that makes no
    progress, and stops the wheels on any exit. The LiDAR sees one plane at
    21 cm; the rover is blind below it. A human watches.

    pivot_goto.py X Y THETA_DEG          map frame
    pivot_goto.py X Y THETA_DEG --rel    relative to where it is now
    pivot_goto.py --mark NAME            remember the current pose
    pivot_goto.py --to NAME              go back to it
"""
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

# The same wall-registration the pivot test grades turns with. Used here as a
# second opinion on the END pose that does not go through slam_toolbox.
sys.path.insert(0, os.environ.get("LIDAR_DIR", "/opt/lidar"))
try:
    from pivot_test import icp as _icp
except ImportError:
    _icp = None

# The pivot for HELD-LEFT turns (the default mode, see ANCHOR below): four
# 45 deg turns on 2026-09-23 slid 21.0-21.6 cm each, pivot spread 1.5 cm. The
# pure-wz pivot was (0.04, 0.21) most of the time and near the centre the rest
# -- which is why pure wz is not the default.
P = np.array([float(os.environ.get("PIVOT_X", "0.017")),
              float(os.environ.get("PIVOT_Y", "0.277"))])
# ...but the pivot depends on the DIRECTION. Measured 2026-09-23, held-left:
#   left turns   +45 (3.1, 27.7)  +45 (3.1, 27.7)  +90 (4.7, 24.9)   cm
#   right turns  -45 (0.3, 27.1)  -45 (0.3, 27.3)  -90 (-0.7, 37.6)  cm
# Left turns sit on one point at both angles. Right turns sit farther out, and
# farther the bigger the turn -- one -90 sample, so this is the mean, not a
# fitted curve; the closed loop corrects what it misses. PIVOT_X/Y above
# overrides both, for re-measuring.
if "PIVOT_X" in os.environ or "PIVOT_Y" in os.environ:
    P_LEFT = P_RIGHT = P
else:
    P_LEFT = np.array([0.035, 0.268])
    P_RIGHT = np.array([0.000, 0.305])


def pivot_for(th):
    return P_LEFT if th >= 0 else P_RIGHT
MARKS = os.environ.get("PIVOT_MARKS", "/logs/drive_marks.json")

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
ANCHOR = os.environ.get("PIVOT_ANCHOR", "hold").strip().lower()


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


WZ = 1.5              # rad/s; TODO 43: a lower command is slower, not gentler
VX = 0.07             # m/s straight
VX_SLOW = 0.04        # last 3 cm
MAX_LEG = 1.5         # m; longer moves belong to nav2
POS_TOL = 0.025       # m
YAW_TOL = math.radians(1.5)
MAX_ITERS = 4
STALL_S = 8.0

# footprint 46 x 42, centred on base_link
CORNERS = np.array([[0.23, 0.21], [0.23, -0.21], [-0.23, -0.21], [-0.23, 0.21]])
SWEEP_MARGIN = 0.08   # m, clearance from the swept footprint during a turn


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def R(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s], [s, c]])


def plan(cur, tgt):
    """(th1, d, th2) taking the centre from pose cur to pose tgt, both (x, y, yaw).

    With ONE pivot P the slide depends only on the total rotation and the leg
    has a closed form (see the docstring). With a pivot per DIRECTION it does
    not: rotating th1 about P1, driving d, then th2 about P2 ends the centre at

        (I - R(th1)) P1  +  d u(th1)  +  R(th1) (I - R(th2)) P2

    so for each th1 the leg must be w(th1) = T - (I-R(th1))P1 - R(th1)(I-R(th2))P2,
    and a straight leg can only run along u(th1). Search th1 for the angles
    where w is parallel to u, take d = u . w, and keep the solution that turns
    least.
    """
    T = R(-cur[2]) @ (np.array(tgt[:2]) - np.array(cur[:2]))
    psi = wrap(tgt[2] - cur[2])

    def resid(th1):
        th2 = wrap(psi - th1)
        w = T - (np.eye(2) - R(th1)) @ pivot_for(th1) \
              - R(th1) @ (np.eye(2) - R(th2)) @ pivot_for(th2)
        u = np.array([math.cos(th1), math.sin(th1)])
        return u[0] * w[1] - u[1] * w[0], float(u @ w)

    grid = np.linspace(-math.pi, math.pi, 1441)
    vals = [resid(a)[0] for a in grid]
    best = None
    for i in range(len(grid) - 1):
        f0, f1 = vals[i], vals[i + 1]
        if f0 == 0.0 or f0 * f1 < 0.0:
            a, b = grid[i], grid[i + 1]
            for _ in range(50):                      # bisect to the root
                m = 0.5 * (a + b)
                if resid(a)[0] * resid(m)[0] <= 0.0:
                    b = m
                else:
                    a = m
            th1 = wrap(0.5 * (a + b))
            # the pivot switches at th = 0, so the residual JUMPS there and a
            # bisection happily "finds" a root at the jump. Keep real ones only.
            if abs(resid(th1)[0]) > 1e-6:
                continue
            d = resid(th1)[1]
            th2 = wrap(psi - th1)
            cost = abs(th1) + abs(th2) + 0.5 * abs(d)   # prefer less turning, then less driving
            if best is None or cost < best[0]:
                best = (cost, th1, d, th2)
    if best is None:
        return 0.0, 0.0, psi
    return best[1], best[2], best[3]


class Driver(Node):
    def __init__(self):
        super().__init__("pivot_goto")
        self.odom = None
        self.scan = None
        self.create_subscription(Odometry, "/odom", lambda m: setattr(self, "odom", m), 10)
        self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "scan", m),
                                 qos_profile_sensor_data)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.buf = Buffer()
        TransformListener(self.buf, self)

    # ── plumbing ────────────────────────────────────────────────────────────
    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def ready(self):
        for _ in range(80):
            self.spin(0.25)
            if self.odom is not None and self.scan is not None and \
               self.buf.can_transform("map", "base_link", rclpy.time.Time()) and \
               self.buf.can_transform("base_link", "laser", rclpy.time.Time()):
                return True
        return False

    def map_pose(self):
        t = self.buf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
        return (t.translation.x, t.translation.y, yaw_of(t.rotation))

    def odom_pose(self):
        p = self.odom.pose.pose
        return np.array([p.position.x, p.position.y]), yaw_of(p.orientation)

    def stop(self):
        for _ in range(8):
            self.cmd.publish(Twist())
            self.spin(0.03)

    def scan_base(self):
        """Current scan as Nx2 points in base_link."""
        m = self.scan
        t = self.buf.lookup_transform("base_link", m.header.frame_id, rclpy.time.Time()).transform
        r = np.asarray(m.ranges, dtype=np.float64)
        a = m.angle_min + np.arange(r.size) * m.angle_increment
        ok = np.isfinite(r) & (r > 0.15) & (r < 8.0)
        q = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)
        return q @ R(yaw_of(t.rotation)).T + np.array([t.translation.x, t.translation.y])

    # ── safety ──────────────────────────────────────────────────────────────
    def clear_to_rotate(self, th):
        """Is the area THIS turn sweeps free? Not a full circle -- the turn.

        The footprint outline is sampled every 2 cm and swung about this
        direction's pivot in 2 deg steps from 0 to th; every scan point must be
        SWEEP_MARGIN from every swept point. The first version demanded a
        clear 360 deg circle round the pivot, which refused a 38 deg turn in a
        room where that turn had 59 cm to spare. Exact for the model, plus the
        margin for the 1.5-7 cm the held-left pivot varies by (TODO 43).
        """
        pts = self.scan_base()
        piv = pivot_for(th)
        edge = []
        for a, b in zip(CORNERS, np.roll(CORNERS, -1, axis=0)):
            n = max(2, int(np.linalg.norm(b - a) / 0.02))
            edge += [a + (b - a) * k / n for k in range(n)]
        edge = np.array(edge) - piv
        steps = max(2, int(abs(math.degrees(th)) / 2.0) + 1)
        swept = np.vstack([edge @ R(a).T + piv for a in np.linspace(0.0, th, steps)])
        near = pts[np.linalg.norm(pts - piv, axis=1) < 1.5]
        if near.size == 0:
            return True
        dmin = float(np.min(np.linalg.norm(near[:, None, :] - swept[None, :, :], axis=2)))
        if dmin < SWEEP_MARGIN:
            print(f"      ✗ not clear to turn {math.degrees(th):+.0f} deg: something is "
                  f"{dmin*100:.0f} cm from where the rover will swing (need "
                  f"{SWEEP_MARGIN*100:.0f})")
            return False
        return True

    def clear_to_drive(self, d):
        pts = self.scan_base()
        lane = np.abs(pts[:, 1]) < 0.21 + 0.05
        if d > 0:
            ahead = pts[lane & (pts[:, 0] > 0.0), 0] - 0.23
        else:
            ahead = -pts[lane & (pts[:, 0] < 0.0), 0] - 0.23
        free = float(np.min(ahead)) if ahead.size else 9.0
        need = abs(d) + 0.08
        if free < need:
            print(f"      ✗ not clear to drive {'forward' if d > 0 else 'back'} {abs(d)*100:.0f} cm: "
                  f"{free*100:.0f} cm free in the lane (need {need*100:.0f})")
            return False
        return True

    # ── primitives ──────────────────────────────────────────────────────────
    def rotate(self, th):
        if abs(th) < math.radians(1.0):
            return True
        if not self.clear_to_rotate(th):
            return False
        _, prev = self.odom_pose()
        t = turn_twist(math.copysign(WZ, th))
        turned, best, best_t = 0.0, 0.0, time.time()
        while rclpy.ok():
            self.cmd.publish(t)
            self.spin(0.02)
            _, y = self.odom_pose()
            turned += wrap(y - prev)
            prev = y
            if abs(turned) >= abs(th):
                break
            if abs(turned) > best + math.radians(1.0):
                best, best_t = abs(turned), time.time()
            if time.time() - best_t > STALL_S:
                self.stop()
                print(f"      ✗ turn stalled at {math.degrees(turned):+.1f} of "
                      f"{math.degrees(th):+.1f} deg")
                return False
        self.stop()
        self.spin(0.8)
        return True

    def straight(self, d):
        if abs(d) < 0.015:
            return True
        if not self.clear_to_drive(d):
            return False
        p0, y0 = self.odom_pose()
        t = Twist()
        best, best_t = 0.0, time.time()
        while rclpy.ok():
            p, y = self.odom_pose()
            gone = float(np.linalg.norm(p - p0))
            left = abs(d) - gone
            if left <= 0.0:
                break
            t.linear.x = math.copysign(VX if left > 0.03 else VX_SLOW, d)
            # hold the heading the leg started on; small, or it becomes a pivot
            t.angular.z = max(-0.3, min(0.3, 1.5 * wrap(y0 - y)))
            self.cmd.publish(t)
            self.spin(0.02)
            if gone > best + 0.01:
                best, best_t = gone, time.time()
            if time.time() - best_t > STALL_S:
                self.stop()
                print(f"      ✗ straight leg stalled at {gone*100:.0f} of {abs(d)*100:.0f} cm")
                return False
        self.stop()
        self.spin(0.8)
        return True

    # ── the loop ────────────────────────────────────────────────────────────
    def go(self, tgt):
        print(f"      target (map): x {tgt[0]:+.3f}  y {tgt[1]:+.3f}  "
              f"yaw {math.degrees(tgt[2]):+.1f} deg     pivots L ({P_LEFT[0]*100:+.1f}, "
              f"{P_LEFT[1]*100:+.1f}) R ({P_RIGHT[0]*100:+.1f}, {P_RIGHT[1]*100:+.1f}) cm, "
              f"turn mode {ANCHOR}")
        cur = self.map_pose()
        start, s_start = cur, self.scan_base()
        print(f"      start  (map): x {cur[0]:+.3f}  y {cur[1]:+.3f}  "
              f"yaw {math.degrees(cur[2]):+.1f} deg")
        for it in range(1, MAX_ITERS + 1):
            e = math.hypot(tgt[0] - cur[0], tgt[1] - cur[1])
            ey = wrap(tgt[2] - cur[2])
            if e < POS_TOL and abs(ey) < YAW_TOL:
                break
            th1, d, th2 = plan(cur, tgt)
            if abs(d) > MAX_LEG:
                print(f"      ✗ straight leg {d:+.2f} m exceeds {MAX_LEG} m — use nav2 for long moves")
                return False
            # A correction pass that needs two big turns to fix a few cm would
            # cost more error than it removes (~7 cm per 90 deg, TODO 43).
            if it > 1 and e < 0.06 and abs(th1) + abs(th2) > math.radians(60) \
                    and abs(ey) < math.radians(5):
                print(f"      stopping at {e*100:.1f} cm: fixing it would take "
                      f"{math.degrees(abs(th1) + abs(th2)):.0f} deg of turning, which "
                      f"slides more than it corrects")
                break
            print(f"      pass {it}: rotate {math.degrees(th1):+6.1f} deg, "
                  f"{'forward' if d >= 0 else 'reverse'} {abs(d)*100:5.1f} cm, "
                  f"rotate {math.degrees(th2):+6.1f} deg")
            if not (self.rotate(th1) and self.straight(d) and self.rotate(th2)):
                return False
            self.spin(1.5)            # let slam_toolbox match the new view
            cur = self.map_pose()
            e = math.hypot(tgt[0] - cur[0], tgt[1] - cur[1])
            ey = wrap(tgt[2] - cur[2])
            print(f"              now x {cur[0]:+.3f}  y {cur[1]:+.3f}  "
                  f"yaw {math.degrees(cur[2]):+.1f}   -> off by {e*100:.1f} cm, "
                  f"{math.degrees(ey):+.1f} deg")
        e = math.hypot(tgt[0] - cur[0], tgt[1] - cur[1])
        ey = wrap(tgt[2] - cur[2])
        ok = e < POS_TOL and abs(ey) < YAW_TOL
        print()
        print(f"      {'✓' if ok else '~'} final: {e*100:.1f} cm and "
              f"{math.degrees(ey):+.1f} deg from the target, by the LiDAR-corrected pose")
        self.verify(start, s_start, tgt, cur)
        print("        Measure it with a tape — that is the only number here not from the rover.")
        return True

    def verify(self, start, s_start, tgt, end):
        """Register the end scan against the start scan: where the WALLS say it went."""
        if _icp is None:
            return
        s_end = self.scan_base()
        r = _icp(s_end, s_start, wrap(end[2] - start[2]))
        if r is None:
            print("        (walls: the start and end views do not overlap enough to check)")
            return
        th, t, resid, _ = r
        want = R(-start[2]) @ (np.array(tgt[:2]) - np.array(start[:2]))
        want_th = wrap(tgt[2] - start[2])
        e = float(np.linalg.norm(t - want))
        print(f"        walls say: moved ({t[0]*100:+.1f}, {t[1]*100:+.1f}) cm, turned "
              f"{math.degrees(th):+.1f} deg — asked ({want[0]*100:+.1f}, {want[1]*100:+.1f}) cm, "
              f"{math.degrees(want_th):+.1f} deg")
        print(f"        => {e*100:.1f} cm and {math.degrees(wrap(th - want_th)):+.1f} deg off "
              f"by direct scan registration (resid {resid*100:.1f} cm), no slam involved")


def load_marks():
    try:
        with open(MARKS) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return 1
    rclpy.init()
    n = Driver()
    try:
        if not n.ready():
            print("      need /odom, /scan, base_link -> laser and a MAP frame (./rover slam)")
            return 1
        if args[0] == "--mark":
            name = args[1] if len(args) > 1 else "mark"
            marks = load_marks()
            marks[name] = n.map_pose()
            with open(MARKS, "w") as f:
                json.dump(marks, f, indent=1)
            x, y, t = marks[name]
            print(f"      marked '{name}': x {x:+.3f}  y {y:+.3f}  yaw {math.degrees(t):+.1f} deg (map)")
            return 0
        if args[0] == "--to":
            name = args[1] if len(args) > 1 else "mark"
            marks = load_marks()
            if name not in marks:
                print(f"      no mark '{name}' — known: {', '.join(marks) or 'none'}")
                return 1
            tgt = tuple(marks[name])
        else:
            x, y, t = float(args[0]), float(args[1]), math.radians(float(args[2]) if len(args) > 2 and not args[2].startswith("--") else 0.0)
            if "--rel" in args:
                cx, cy, cyaw = n.map_pose()
                w = R(cyaw) @ np.array([x, y])
                tgt = (cx + w[0], cy + w[1], wrap(cyaw + t))
            else:
                tgt = (x, y, t)
        return 0 if n.go(tgt) else 1
    except KeyboardInterrupt:
        print("      interrupted")
        return 1
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
