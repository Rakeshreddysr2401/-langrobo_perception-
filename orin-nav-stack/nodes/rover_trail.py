#!/usr/bin/env python3
"""
rover_trail.py — draw the line the rover has actually driven.

WHY
    "I want a line showing where it went" is not something RViz can do on its own.
    The laptop view had an Odometry display, which draws a CLOUD OF ARROWS, not a
    line — and it was pointed at /odom, the RAW cuVSLAM pose. When you run
    `./run_stack.sh fuse`, the EKF owns the pose and /odom is the un-fused signal
    with the drifting z that smeared the map (learn/PROGRESS.md, 2026-08-11). So
    the trail was being drawn from the one pose source we know not to trust.

    This node keeps a real nav_msgs/Path from the FUSED pose, which RViz draws as
    a continuous line.

FRAME
    The path is published in whatever frame the odometry is in — `odom` — because
    that is the frame the nvblox map lives in (config/nvblox.yaml: global_frame).
    Trail and map therefore always line up. If nvblox ever moves to `map`, this
    follows automatically; it never hardcodes a frame.

BANDWIDTH — the reason for every constant below
    This view is watched over wifi, and that channel is already near its limit: a
    raw camera image took the nvblox map at the laptop from 9.4 Hz to ZERO
    (learn/00-setup.md). A Path is ~56 bytes per pose, so a naive 5000-pose path
    at 10 Hz is ~2.8 MB/s and would do exactly the same damage.

    Three defences, in order of importance:
      1. Only publish when a point is actually ADDED. A parked rover sends nothing.
      2. Only add a point every MIN_STEP_M of travel, not every message.
      3. Cap the path at MAX_POINTS, dropping the oldest.
    Worst case is then MAX_POINTS * 56 B * MAX_HZ ~= 110 kB/s while driving, which
    sits well under the map's own ~290 kB/s.

    TRANSIENT_LOCAL means RViz gets the whole trail the moment it connects, even
    if the rover is standing still and publishing nothing.

USAGE
    python3 rover_trail.py
    Add a Path display on /rover/trail in RViz (Durability: Transient Local).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

SRC_TOPIC = "/odometry/filtered"   # FUSED pose. Never /odom — see docstring.
MIN_STEP_M = 0.05                  # add a point every 5 cm of travel
MAX_POINTS = 2000                  # 2000 * 5 cm = 100 m of trail
MAX_HZ = 2.0                       # ceiling on publish rate while driving


class Trail(Node):
    def __init__(self):
        super().__init__("rover_trail")

        # Best-effort IN (odometry is a stream; a dropped sample costs 5 cm of
        # resolution and nothing else), transient-local OUT (so a late RViz sees
        # the whole trail immediately).
        self.sub = self.create_subscription(
            Odometry, SRC_TOPIC, self.on_odom,
            QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE))
        self.pub = self.create_publisher(
            Path, "/rover/trail",
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # Clearing the trail is genuinely useful: after a `remap` the old line is
        # stale geometry drawn over a brand-new map, which looks like a bug.
        self.create_service(Empty, "/rover/trail/clear", self.on_clear)

        self.path = Path()
        self.last = None          # (x, y) of the last point we kept
        self.last_pub = 0.0
        self.warned = False
        self.get_logger().info(
            f"trail from {SRC_TOPIC}: point every {MIN_STEP_M} m, "
            f"max {MAX_POINTS} points, <= {MAX_HZ} Hz on /rover/trail")

    def on_clear(self, req, resp):
        n = len(self.path.poses)
        self.path.poses = []
        self.last = None
        self.publish(force=True)
        self.get_logger().info(f"trail cleared ({n} points dropped)")
        return resp

    def on_odom(self, msg):
        p = msg.pose.pose.position

        # The fused pose pins z to 0 (two_d_mode). If z wanders, we are running
        # visual-only and the map is being smeared — say so once, loudly, because
        # it is invisible otherwise and it is the exact bug from 2026-08-11.
        if not self.warned and abs(p.z) > 0.02:
            self.warned = True
            self.get_logger().warn(
                f"pose z = {p.z:.3f} m, expected 0.000 — is the EKF running? "
                "Visual-only odometry drifts in z and smears the nvblox map. "
                "Run ./run_stack.sh fuse")

        if self.last is not None:
            if math.hypot(p.x - self.last[0], p.y - self.last[1]) < MIN_STEP_M:
                return                       # not moved enough: publish nothing

        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.path.poses.append(ps)
        if len(self.path.poses) > MAX_POINTS:
            del self.path.poses[:len(self.path.poses) - MAX_POINTS]
        self.last = (p.x, p.y)

        # frame_id comes from the odometry itself, so trail and map cannot
        # disagree about which frame they are in.
        self.path.header.frame_id = msg.header.frame_id
        self.publish()

    def publish(self, force=False):
        now = self.get_clock().now().nanoseconds * 1e-9
        if not force and (now - self.last_pub) < (1.0 / MAX_HZ):
            return
        self.last_pub = now
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path)


def main():
    rclpy.init()
    n = Trail()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
