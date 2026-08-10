#!/usr/bin/env python3
"""
odom_ruler.py — a tape measure for the robot's own sense of distance.

WHY
    Phase 1 of ThingsTodo.md: before trusting ANY autonomous motion, prove the pose
    is metrically honest. Push the rover a tape-measured 1.00 m BY HAND (motors off)
    and see what each odometry source claims.

    Hand-pushing is safe with firmware v2: pidStep() returns 0 whenever the target
    velocity is <0.01, and driveSide() with zero duty pulls both BTS7960 PWM pins low,
    which is COAST, not brake. The wheels free-wheel and the PID does not fight you.

    This matters because cuVSLAM has been wrong by 4x here without saying so: with the
    IR emitter ON, a 100 cm push read as 26 cm on /odom. Emitter OFF it read 97 cm.
    Nothing logged an error either time. See memory: emitter-breaks-cuvslam-scale.

USAGE
    python3 odom_ruler.py                 # live readout, Ctrl-C for the summary
    python3 odom_ruler.py --expect 1.00   # also grades the result against a tape figure

WHAT THE TWO NUMBERS MEAN
    straight = distance from where you started (what a tape measures)
    path     = total distance travelled (longer if you wandered or reversed)
    A big path-vs-straight gap on a "straight" push means the pose is wandering.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# label -> topic. Each is an independent story about how far the robot moved;
# where they disagree is exactly where the trouble is.
SOURCES = [
    ("EKF  (fused)", "/odometry/filtered"),
    ("VO   (cuVSLAM)", "/odom"),
    ("wheel (encoders)", "/wheel_odom"),
]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Track:
    """Start pose, straight-line displacement and integrated path for one source."""

    def __init__(self):
        self.x0 = self.y0 = self.yaw0 = None
        self.x = self.y = self.yaw = 0.0
        self.path = 0.0
        self.n = 0
        self._px = self._py = None

    def update(self, x, y, yaw):
        self.n += 1
        if self.x0 is None:
            self.x0, self.y0, self.yaw0 = x, y, yaw
            self._px, self._py = x, y
        # ignore the teleport-sized jumps cuVSLAM produces when it loses tracking,
        # otherwise one explosion adds tens of metres to the path length
        d = math.hypot(x - self._px, y - self._py)
        if d < 0.5:
            self.path += d
        self._px, self._py = x, y
        self.x, self.y, self.yaw = x, y, yaw

    @property
    def straight(self):
        if self.x0 is None:
            return 0.0
        return math.hypot(self.x - self.x0, self.y - self.y0)

    @property
    def dyaw_deg(self):
        if self.x0 is None:
            return 0.0
        return math.degrees((self.yaw - self.yaw0 + math.pi) % (2 * math.pi) - math.pi)


class OdomRuler(Node):
    def __init__(self):
        super().__init__("odom_ruler")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.tracks = {}
        for label, topic in SOURCES:
            self.tracks[label] = Track()
            self.create_subscription(
                Odometry, topic, self._make_cb(label), qos)
        self.health = None
        self.create_subscription(String, "/odom/health", self._health_cb, qos)

    def _make_cb(self, label):
        def cb(msg):
            p = msg.pose.pose
            self.tracks[label].update(p.position.x, p.position.y,
                                      yaw_of(p.orientation))
        return cb

    def _health_cb(self, msg):
        self.health = msg.data

    def reset(self):
        for t in self.tracks.values():
            t.__init__()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--expect", type=float, default=None,
                    help="tape-measured distance in metres, to grade against")
    ap.add_argument("--settle", type=float, default=2.0,
                    help="seconds to watch before zeroing (default 2)")
    args = ap.parse_args()

    rclpy.init()
    node = OdomRuler()

    print("\n  odom_ruler — MOTORS OFF, push the rover by hand.\n")
    print(f"  settling {args.settle:.0f}s (hold still) ...")
    t_end = time.time() + args.settle
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    live = {l: t for l, t in node.tracks.items() if t.n > 0}
    if not live:
        print("\n  ✗ no odometry at all. Is the stack up? './run_stack.sh up' then 'fuse'\n")
        rclpy.shutdown()
        return 1
    for label, _ in SOURCES:
        if label not in live:
            print(f"  --  {label}: no data (not running)")

    node.reset()
    print("  ZEROED. Push now. Ctrl-C when you reach the mark.\n")

    try:
        last_print = 0.0
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if now - last_print < 0.2:
                continue
            last_print = now
            parts = []
            for label, _ in SOURCES:
                t = node.tracks[label]
                if t.n == 0:
                    continue
                parts.append(f"{label} {t.straight:6.3f} m")
            sys.stdout.write("\r  " + " | ".join(parts) + "   ")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass

    print("\n\n  ── result " + "─" * 52)
    ref = None
    for label, _ in SOURCES:
        t = node.tracks[label]
        if t.n == 0:
            continue
        line = (f"  {label:<18} straight {t.straight:6.3f} m   "
                f"path {t.path:6.3f} m   yaw {t.dyaw_deg:+6.1f}°")
        if args.expect:
            err = t.straight - args.expect
            pct = 100.0 * err / args.expect if args.expect else 0.0
            line += f"   err {err:+.3f} m ({pct:+.1f}%)"
        print(line)
        if label.startswith("VO"):
            ref = t

    if args.expect and ref is not None:
        pct = abs(ref.straight - args.expect) / args.expect * 100.0
        print()
        if pct <= 5.0:
            print(f"  ✓ PASS — cuVSLAM within {pct:.1f}% of the tape. Scale is honest.")
        elif pct <= 15.0:
            print(f"  ~ MARGINAL — cuVSLAM off by {pct:.1f}%. Re-run; if it repeats, "
                  f"suspect the CAL factor in drive_test.py.")
        else:
            print(f"  ✗ FAIL — cuVSLAM off by {pct:.1f}%. This is the emitter/scale class "
                  f"of bug.\n    Check the IR emitter is OFF and the floor has texture. "
                  f"DO NOT drive autonomously until this passes.")

    if node.health:
        print(f"\n  /odom/health: {node.health}")
    print()

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
