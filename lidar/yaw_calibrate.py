#!/usr/bin/env python3
"""yaw_calibrate.py — measure LIDAR_YAW by driving, with no props and no eyes.

WHY THIS EXISTS
    LIDAR_YAW is the angle between the lidar's zero beam and the rover's nose.
    It is not marked on the C1's case. Reading it off a picture failed twice --
    a quarter-turn error looks the same from several descriptions and the sign
    inverts easily between eye and file -- so this measures it from the sensors
    instead.

THE IDEA
    Drive straight forward a short, odometry-measured distance. Every fixed
    point in the room then shifts by the SAME vector in the lidar's own frame,
    and the direction of that shift is the mount angle. Nothing has to be
    placed, described, or held.

THE MATH, once
    A world point p seen in the laser frame is  q = R(-yaw) (p_base - o).
    Drive forward by d with no rotation, so p_base(1) = p_base(0) - (d, 0):

        q1 - q0 = -R(-yaw) (d, 0)

    Let t be the translation that maps the SECOND scan back onto the first,
    i.e. t = q0 - q1 = R(-yaw) (dx, dy) for a body-frame move of (dx, dy). Its
    angle is therefore (alpha - yaw) where alpha = atan2(dy, dx), so

        LIDAR_YAW = alpha - atan2(t_y, t_x)

    Check it: driving forward is alpha = 0, and yaw 0 slides the room backwards
    along the lidar's own -x, so t points along +x and the answer is 0. yaw +90
    gives t along -y and the answer is +90.

    Taking alpha from odometry rather than assuming it is 0 is what lets the
    SAME formula read the return leg (alpha = pi). That matters: any curvature
    in the drive biases the forward and backward estimates in OPPOSITE
    directions, so averaging the two cancels most of it. A single forward run
    cannot do that, and the first one we took was curved by -1.26 deg.

WHAT IT DRIVES
    0.20 m at 0.07 m/s, closed loop on /odom, with a hard timeout, a straight-
    ness check, and a depth-camera clearance gate before it moves. The depth
    camera is used for the gate and the lidar is not, on purpose: which lidar
    beams point forward is the very thing being measured.

    ./rover lidar --calibrate
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
from sensor_msgs.msg import Image, LaserScan

DIST = 0.20          # m, how far to drive
SPEED = 0.07         # m/s
REPS = 3             # out-and-back pairs; each pair cancels curvature bias
MIN_CLEAR = 0.40     # m, depth-camera 1st percentile required before moving
MAX_YAW_DRIFT = 4.0  # deg, above this the drive was not straight enough
RANGE_LO, RANGE_HI = 0.20, 6.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def to_xy(scan):
    """LaserScan -> Nx2 points in the scan's OWN frame."""
    r = np.asarray(scan.ranges, dtype=np.float64)
    a = scan.angle_min + np.arange(r.size) * scan.angle_increment
    ok = np.isfinite(r) & (r > RANGE_LO) & (r < RANGE_HI)
    return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)


def icp_translation(src, dst, iters=40):
    """Translation t minimising |src + t - dst|, by nearest-neighbour ICP.

    Translation only: the drive is commanded straight and its rotation is
    checked against odometry first, so a rotation term here would only give
    the fit somewhere to hide a bad drive.
    """
    tree = cKDTree(dst)
    t = np.zeros(2)
    for _ in range(iters):
        d, idx = tree.query(src + t)
        keep = d < max(0.25, np.percentile(d, 80))   # drop what moved or vanished
        if keep.sum() < 30:
            return None, 0.0, 0
        step = (dst[idx[keep]] - (src[keep] + t)).mean(axis=0)
        t = t + step
        if np.linalg.norm(step) < 1e-5:
            break
    d, _ = tree.query(src + t)
    return t, float(np.median(d)), int(keep.sum())


class Calib(Node):
    def __init__(self):
        super().__init__("yaw_calibrate")
        self.scan = None
        self.odom = None
        self.depth = None
        self.create_subscription(LaserScan, "/scan", self._s, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._o, 10)
        # Held only until the clearance gate has its frame, then dropped. The
        # raw depth stream is ~900 kB a frame at 30 Hz, and ARCHITECTURE.md
        # warns that subscribing to raw camera topics can take the D555
        # offline. Holding it for a whole multi-minute calibration did exactly
        # that on 2026-09-22: the camera stalled 9.36 s, cuVSLAM lost tracking
        # and the gyro went with it (its source is the camera's own IMU), so
        # the pivot run that followed measured a frozen pose and read 0.000.
        self.depth_sub = self.create_subscription(
            Image, "/camera/camera0/depth/image_rect_raw",
            self._d, qos_profile_sensor_data)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)

    def _s(self, m): self.scan = m
    def _o(self, m): self.odom = m
    def _d(self, m): self.depth = m

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_all(self, sec=12.0):
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.scan is not None and self.odom is not None and self.depth is not None:
                return True
        return False

    def clearance(self):
        m = self.depth
        d = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width)
        d = d.astype(np.float32) / 1000.0
        d[d <= 0.05] = np.nan
        h, w = d.shape
        band = d[int(h * 0.35):int(h * 0.75), int(w * 0.30):int(w * 0.70)]
        v = band[np.isfinite(band)]
        return float(np.percentile(v, 1)) if v.size >= 200 else 0.0

    def pose(self):
        p = self.odom.pose.pose
        return np.array([p.position.x, p.position.y]), yaw_of(p.orientation)

    def stop(self):
        for _ in range(6):
            self.cmd.publish(Twist())
            self.spin(0.05)

    def drive(self, dist, speed):
        p0, y0 = self.pose()
        t = Twist()
        t.linear.x = speed
        # abs(): the return leg passes a NEGATIVE speed, and dist/speed would
        # then put the deadline in the past and time out before a wheel turned.
        deadline = time.time() + dist / abs(speed) * 3.0 + 4.0
        while rclpy.ok():
            self.cmd.publish(t)
            self.spin(0.1)
            p, _ = self.pose()
            gone = float(np.linalg.norm(p - p0))
            if gone >= dist:
                break
            if time.time() > deadline:
                self.stop()
                print(f"      timed out after {gone:.3f} m — the wheels may not be driving")
                return None
        self.stop()
        self.spin(1.5)                     # let it settle before the second scan
        p1, y1 = self.pose()
        return p0, y0, p1, y1


def leg(n, label, signed_speed):
    """One drive, and the yaw it implies. signed_speed < 0 reverses."""
    n.spin(0.6)
    s0 = to_xy(n.scan)
    got = n.drive(DIST, signed_speed)
    if got is None:
        return None
    p0, y0, p1, y1 = got
    n.spin(0.5)
    s1 = to_xy(n.scan)

    d_world = p1 - p0
    dyaw = math.degrees(wrap(y1 - y0))
    c, sn = math.cos(-y0), math.sin(-y0)
    dx = c * d_world[0] - sn * d_world[1]
    dy = sn * d_world[0] + c * d_world[1]
    dist = float(np.linalg.norm(d_world))
    alpha = math.atan2(dy, dx)

    t, resid, used = icp_translation(s1, s0)
    if t is None:
        print(f"      {label}: scans do not overlap enough to align")
        return None
    shift = float(np.linalg.norm(t))
    yaw = wrap(alpha - math.atan2(t[1], t[0]))
    agree = abs(shift - dist) / max(dist, 1e-6) * 100

    flag = ""
    if agree > 20:
        flag = "  ! lidar/odom distance disagree"
    elif abs(dyaw) > MAX_YAW_DRIFT:
        flag = "  ! curved"
    print(f"      {label:9s} odom {dist:.3f} m  lidar {shift:.3f} m  "
          f"turn {dyaw:+5.2f} deg  resid {resid*100:4.1f} cm  "
          f"-> {math.degrees(yaw):+7.2f} deg{flag}")
    if agree > 20 or abs(dyaw) > MAX_YAW_DRIFT:
        return None
    return yaw


def main():
    rclpy.init()
    n = Calib()
    try:
        if not n.wait_all():
            print("      need /scan, /odom and depth — is the whole stack up?")
            return 1

        clear = n.clearance()
        n.destroy_subscription(n.depth_sub)      # see the note where it is made
        n.depth = None
        print(f"      forward clearance (depth, 1st pct): {clear:.2f} m")
        if clear < MIN_CLEAR:
            print(f"      ✗ under {MIN_CLEAR:.2f} m — not driving. Clear the space in front.")
            return 1

        print(f"      {REPS} out-and-back pairs of {DIST:.2f} m at {SPEED:.2f} m/s.")
        print("      Each pair cancels drive curvature, which biases out and back")
        print("      in opposite directions.")
        print()
        est = []
        for i in range(REPS):
            f = leg(n, f"{i+1} fwd", SPEED)
            b = leg(n, f"{i+1} back", -SPEED)
            est += [e for e in (f, b) if e is not None]
        print()
        if len(est) < 2:
            print("      ✗ too few usable legs to average")
            return 1

        # circular mean: these are angles, and a naive mean is wrong near +/-pi
        cs = sum(math.cos(e) for e in est)
        sn = sum(math.sin(e) for e in est)
        yaw = math.atan2(sn, cs)
        spread = max(abs(math.degrees(wrap(e - yaw))) for e in est)
        print(f"      {len(est)} usable legs, spread {spread:.2f} deg")
        print()
        print(f"      LIDAR_YAW = {yaw:+.4f} rad   ({math.degrees(yaw):+.2f} deg)")
        print()

        near90 = abs(abs(math.degrees(yaw)) - 90.0)
        if spread > 3.0:
            print(f"      ! spread {spread:.1f} deg is wide — treat this as provisional.")
        elif near90 < spread:
            q = math.copysign(math.pi / 2, yaw)
            print(f"      Within the spread of a clean {math.degrees(q):+.0f} deg mount.")
            print(f"      Use the measured value anyway: {yaw:+.4f}")
        else:
            print(f"      {near90:.1f} deg off a clean quarter turn, and the spread is only")
            print(f"      {spread:.1f} deg — so the mount really is skewed. Use the measured value.")
        print(f"      set it:  LIDAR_YAW={yaw:.4f} in ./rover, then ./rover lidar")
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
