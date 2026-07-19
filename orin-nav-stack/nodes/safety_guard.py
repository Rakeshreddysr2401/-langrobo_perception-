#!/usr/bin/env python3
"""No-crash layer (phase-1 Step 6): the LAST gate before the wheels.

Chain:  nav2 MPPI -> /cmd_vel_nav -> cmd_vel_deadband -> /cmd_vel_shim
                                   -> safety_guard (THIS) -> /cmd_vel -> wheels

Two independent protections:

1. POSE-SANITY WATCHDOG — cuVSLAM can blow up under CPU load (2026-07-19:
   -34 m jump after a nav goal). A rover navigating on a hallucinated pose
   drives into walls, so on any of:
       * translation jump > `jump_max` m between consecutive /odom msgs
       * |z| > `z_max` m (floor rover must stay near z=0)
       * roll/pitch > `tilt_max` rad
   the guard TRIPS: cancels all nav2 goals, zeroes the wheels, and drops every
   incoming command until the pose has been continuously sane for
   `stable_sec` s (auto-clear, logged).

2. VIRTUAL BUMPER — nvblox's occupancy grid remembers walls the depth camera
   saw even once they're inside the D555's 0.4 m blind zone. Any forward
   command while an occupied cell sits inside the rectangle
   [front_offset .. front_offset+bumper_len] x [±bumper_width/2] (base_link
   frame) gets its vx zeroed — rotation and reverse stay allowed, so the robot
   can always turn or back away. Freedom preserved, face-plants not.

Manual trip: publish `true` on /safety/trip (std_msgs/Bool) — same latch,
cleared by publishing `false` (still subject to the pose being sane).
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Bool, String
from action_msgs.srv import CancelGoal
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation


class SafetyGuard(Node):
    def __init__(self):
        super().__init__('safety_guard')
        self.declare_parameter('cmd_in', '/cmd_vel_shim')
        self.declare_parameter('cmd_out', '/cmd_vel')
        self.declare_parameter('jump_max', 0.35)      # m between consecutive odom msgs
        self.declare_parameter('z_max', 0.25)         # m
        self.declare_parameter('tilt_max', 0.7)       # rad (~40 deg)
        self.declare_parameter('stable_sec', 10.0)
        self.declare_parameter('grid_topic', '/nvblox_node/static_occupancy_grid')
        self.declare_parameter('bumper_len', 0.35)    # m ahead of front_offset
        self.declare_parameter('bumper_width', 0.32)  # m (robot width + margin)
        self.declare_parameter('front_offset', 0.10)  # m base_link -> front bumper
        self.declare_parameter('occ_thresh', 60)      # occupancy 0..100
        p = lambda n: self.get_parameter(n).value
        self.jump_max = float(p('jump_max'))
        self.z_max = float(p('z_max'))
        self.tilt_max = float(p('tilt_max'))
        self.stable_sec = float(p('stable_sec'))
        self.bumper_len = float(p('bumper_len'))
        self.bumper_width = float(p('bumper_width'))
        self.front_offset = float(p('front_offset'))
        self.occ_thresh = int(p('occ_thresh'))

        self.tripped = False
        self.trip_reason = ''
        self.manual_trip = False
        self.last_pose = None            # (t_sec, x, y, z)
        self.sane_since = None           # t_sec when pose last became sane
        self.grid = None                 # latest OccupancyGrid
        self.bumper_hits = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Twist, p('cmd_out'), 10)
        self.state_pub = self.create_publisher(String, '/safety/state', 10)
        self.create_subscription(Twist, p('cmd_in'), self._cmd_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 20)
        self.create_subscription(OccupancyGrid, p('grid_topic'), self._grid_cb, 1)
        self.create_subscription(Bool, '/safety/trip', self._manual_cb, 10)
        self.cancel_cli = self.create_client(CancelGoal,
                                             '/navigate_to_pose/_action/cancel_goal')
        self.create_timer(1.0, self._state_tick)
        self.get_logger().info(
            f"safety_guard up: {p('cmd_in')} -> {p('cmd_out')} | "
            f'jump>{self.jump_max}m |z|>{self.z_max}m tilt>{self.tilt_max}rad trips; '
            f'bumper {self.bumper_len}x{self.bumper_width}m @ occ>={self.occ_thresh}')

    # ---- pose-sanity watchdog --------------------------------------------
    def _odom_cb(self, m: Odometry):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        pos = m.pose.pose.position
        q = m.pose.pose.orientation
        roll, pitch, _ = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
        bad = None
        if abs(pos.z) > self.z_max:
            bad = f'|z|={abs(pos.z):.2f}m > {self.z_max}'
        elif max(abs(roll), abs(pitch)) > self.tilt_max:
            bad = f'tilt r={roll:.2f} p={pitch:.2f} rad > {self.tilt_max}'
        elif self.last_pose is not None:
            dt = t - self.last_pose[0]
            if 0.0 < dt < 1.0:   # ignore gaps (SLAM restart) — z/tilt still checked
                d = math.dist((pos.x, pos.y, pos.z), self.last_pose[1:])
                if d > self.jump_max:
                    bad = f'jump {d:.2f}m in {dt*1000:.0f}ms > {self.jump_max}m'
        self.last_pose = (t, pos.x, pos.y, pos.z)

        if bad:
            self.sane_since = None
            if not self.tripped:
                self._trip(f'pose watchdog: {bad}')
            return
        if self.sane_since is None:
            self.sane_since = t
        if (self.tripped and not self.manual_trip
                and t - self.sane_since >= self.stable_sec):
            self.tripped = False
            self.trip_reason = ''
            self.get_logger().info(
                f'pose sane for {self.stable_sec:.0f}s — guard cleared, motion re-enabled')

    def _trip(self, reason: str):
        self.tripped = True
        self.trip_reason = reason
        self.get_logger().error(f'TRIPPED: {reason} — cancelling nav goals, wheels zeroed')
        if self.cancel_cli.service_is_ready():
            self.cancel_cli.call_async(CancelGoal.Request())   # empty = cancel all
        z = Twist()
        for _ in range(5):
            self.pub.publish(z)

    def _manual_cb(self, m: Bool):
        if m.data and not self.tripped:
            self.manual_trip = True
            self._trip('manual /safety/trip')
        elif not m.data and self.manual_trip:
            self.manual_trip = False
            self.get_logger().info('manual trip released (pose watchdog still applies)')

    # ---- virtual bumper ---------------------------------------------------
    def _grid_cb(self, m: OccupancyGrid):
        self.grid = m

    def _front_blocked(self) -> bool:
        g = self.grid
        if g is None:
            return False
        try:
            tf = self.tf_buffer.lookup_transform(g.header.frame_id, 'base_link', Time())
        except Exception:
            return False
        tr, q = tf.transform.translation, tf.transform.rotation
        R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()[:2, :2]
        base = np.array([tr.x, tr.y])
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        data = np.asarray(g.data, dtype=np.int8).reshape(g.info.height, g.info.width)
        step = max(res, 0.05)
        for dx in np.arange(self.front_offset, self.front_offset + self.bumper_len + 1e-6, step):
            for dy in np.arange(-self.bumper_width / 2, self.bumper_width / 2 + 1e-6, step):
                pt = base + R @ np.array([dx, dy])
                col = int((pt[0] - ox) / res)
                row = int((pt[1] - oy) / res)
                if 0 <= row < g.info.height and 0 <= col < g.info.width \
                        and data[row, col] >= self.occ_thresh:
                    return True
        return False

    # ---- the gate ---------------------------------------------------------
    def _cmd_cb(self, m: Twist):
        if self.tripped:
            self.pub.publish(Twist())    # hold zeros while latched
            return
        if m.linear.x > 0.0 and self._front_blocked():
            self.bumper_hits += 1
            self.get_logger().warn(
                'virtual bumper: obstacle in front box — forward blocked '
                '(rotate/reverse still allowed)', throttle_duration_sec=2.0)
            m.linear.x = 0.0
        self.pub.publish(m)

    def _state_tick(self):
        s = 'TRIPPED:' + self.trip_reason if self.tripped else 'ok'
        self.state_pub.publish(String(data=s))


def main():
    rclpy.init()
    rclpy.spin(SafetyGuard())


if __name__ == '__main__':
    main()
