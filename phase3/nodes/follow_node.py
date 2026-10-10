#!/usr/bin/env python3
"""follow_node.py — "follow me" / "come to me", live (phase3/nodes/follow.py).

    /follow/start    std_msgs/String JSON {} or {"id": 3} (a /vision/people id)
                     or {"stop_at_gap": true}: come to me -- done once at GAP
    /follow/cancel   std_msgs/Empty
        -> /cmd_vel          while following, and a stop when it ends
           /follow/status    JSON on every change and at 2 Hz while following:
                             state, target, dist, bearing, vx, wz, blocked,
                             paused; when done: result + why
                             (reached | lost | blocked | nobody | cancelled |
                              timeout | pose unsure | no people | taken over)

INPUTS, and why each
    /vision/people  detections_3d + person_tracker. Subscribed ONLY while
                    following: a subscriber is what wakes YOLO to full rate
                    (detections_3d._demand), so an idle follower costs nothing.
    /odom           the pose; the loop runs on it (20 Hz, no timers -- the
                    fusion2 lesson goal_exec_node also follows).
    /scan + fused map  the corridor and turning checks (fused_obstacles, the
                    same source goal_exec judges on).
    /fusion/status  the pose's confidence: exactly goal_exec_node's rules --
                    unsure -> zero command; unsure for PAUSE_S -> give up.

ONE DRIVER. /cmd_vel has no arbiter (ARCHITECTURE.md): nav2, reach, goal_exec
and teleop all publish on it. So a start cancels goal_exec and reach, and a
goal sent to either of them (or the teleop's cancels, which the Pi 5 also sends
to /follow/cancel) ends the follow.
"""
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parent))
from follow import GAP, DIST_DEADBAND, Follower  # noqa: E402
from fused_obstacles import make_obstacles  # noqa: E402

SD_MAX_CM = 3.0          # = goal_exec_node
SCAN_STALE_S = 0.5       # = goal_exec_node
PAUSE_S = 5.0            # = goal_exec_node
PEOPLE_STALE_S = 1.0     # no /vision/people this recent: the camera or YOLO is gone -> stop
NO_PEOPLE_S = 4.0        # ...and give up after this long
START_WAIT_S = 3.0       # after a start, wait this long for a people message to pick from
MAX_S = 600.0            # a follow ends by itself after 10 min


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class FollowNode(Node):
    def __init__(self):
        super().__init__('follow')
        self.f = Follower()
        self.pose = None
        self.pts, self.pts_t = None, 0.0
        self.mount = None
        self.fstatus, self.fstatus_t = None, 0.0
        self.people_sub = None
        self.people_t = 0.0
        self.pending = None          # a start waiting for its first people message
        self.active = False          # a follow (or a start) not yet ended and reported
        self.started = 0.0
        self.stop_at_gap = False
        self.paused_since = None
        self.last_key = None
        self.last_report = 0.0

        # cmd_topic:=/follow/cmd_vel_dry is a DRY RUN: everything live, the
        # wheels untouched (nothing listens there) -- watch what it would do.
        self.declare_parameter('cmd_topic', '/cmd_vel')
        cmd_topic = self.get_parameter('cmd_topic').value
        self.dry = cmd_topic != '/cmd_vel'
        self.cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.pub_status = self.create_publisher(String, '/follow/status', 10)
        self.ge_cancel = self.create_publisher(Empty, '/goal_exec/cancel', 10)
        self.reach_cancel = self.create_publisher(Empty, '/reach/cancel', 10)

        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.dobs = make_obstacles(self, self.buf, lambda: self.pose, active=False)

        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(String, '/fusion/status', self._fstatus, 10)
        self.create_subscription(String, '/follow/start', self._start, 10)
        self.create_subscription(Empty, '/follow/cancel', lambda _: self._end('cancelled', 'cancelled'), 10)
        # another driver taking /cmd_vel ends the follow
        for topic in ('/goal_exec/goal', '/goal_exec/turn', '/reach/goal'):
            self.create_subscription(PoseStamped, topic,
                                     lambda _, t=topic: self._end('taken over', f'a goal on {t}'), 10)
        self.create_subscription(String, '/goal_exec/pass',
                                 lambda _: self._end('taken over', 'a pass on /goal_exec/pass'), 10)
        self.get_logger().info(f'follow up; gap {GAP} m; commands on {cmd_topic}'
                               + (' (DRY RUN: wheels untouched)' if self.dry else ''))

    # ── inputs ──────────────────────────────────────────────────────────────
    def _fstatus(self, m):
        try:
            self.fstatus, self.fstatus_t = json.loads(m.data), time.time()
        except ValueError:
            pass

    def _scan(self, m):
        if self.mount is None:
            if not self.buf.can_transform('base_link', m.header.frame_id, Time()):
                return
            tr = self.buf.lookup_transform('base_link', m.header.frame_id, Time()).transform
            self.mount = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
        r = np.asarray(m.ranges)
        a = m.angle_min + np.arange(r.size) * m.angle_increment
        ok = np.isfinite(r) & (r > 0.05) & (r < 8.0)
        x, y, yaw = self.mount
        p = np.stack([r[ok] * np.cos(a[ok] + yaw) + x, r[ok] * np.sin(a[ok] + yaw) + y], 1)
        own = (p[:, 0] < 0.202) & (p[:, 0] > -0.198) & (np.abs(p[:, 1]) < 0.21)   # the rover itself
        self.pts = p[~own]
        self.pts_t = time.time()

    def _start(self, m):
        try:
            d = json.loads(m.data) if m.data.strip() else {}
        except ValueError:
            d = {}
        if self.active:
            self._end('cancelled', 'restarted', stop=False)
        if not self.dry:                       # a dry run takes nothing from the drivers
            self.ge_cancel.publish(Empty())
            self.reach_cancel.publish(Empty())
        self.f = Follower()                    # no result left over from the last follow
        self.stop_at_gap = bool(d.get('stop_at_gap'))
        self.pending = {'id': d.get('id'), 'since': time.time()}
        self.active = True
        if self.people_sub is None:
            self.people_sub = self.create_subscription(String, '/vision/people', self._people, 10)
        self.dobs.set_active(True)
        self.paused_since = None
        self.get_logger().info(f'start {d}: waiting for /vision/people')
        self._report(force=True, extra={'state': 'starting'})

    def _people(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        t = float(d.get('stamp', time.time()))
        self.people_t = time.time()
        people = d.get('people', [])
        if self.pending:
            if not self.f.start(t, people, self.pending['id']):
                if time.time() - self.pending['since'] < START_WAIT_S:
                    return                     # YOLO may need a frame or two to wake up
                self._end(self.f.result, self.f.why)   # _end clears pending
                return
            self.pending = None
            self.started = time.time()
            self.get_logger().info(f'following person {self.f.target_id}')
            self._report(force=True)
            return
        self.f.update_people(t, people)

    # ── the loop: one step per fused pose ───────────────────────────────────
    def _odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.pending and time.time() - self.pending['since'] > START_WAIT_S + 1.0:
            self._end('no people', 'no /vision/people -- is ./rover detect running?')
            return
        if self.f.state != 'following':
            return
        if time.time() - self.started > MAX_S:
            self._end('timeout', f'followed for {MAX_S / 60:.0f} min')
            return
        unsure = self._pose_unsure()
        if not unsure and time.time() - self.people_t > PEOPLE_STALE_S:
            if time.time() - self.people_t > NO_PEOPLE_S:
                self._end('no people', f'no /vision/people for {NO_PEOPLE_S:.0f} s')
                return
            unsure = f'no /vision/people for {time.time() - self.people_t:.1f} s'
        if unsure:
            self._stop()
            self.f.vx = self.f.wz = 0.0
            self.paused_since = self.paused_since or time.time()
            if time.time() - self.paused_since > PAUSE_S:
                self._end('pose unsure', unsure)
            else:
                self._report(extra={'paused': unsure})
            return
        self.paused_since = None
        pts = self.pts
        dp = self.dobs.points_base(self.pose)
        if len(dp):
            pts = dp if pts is None else np.vstack([pts, dp])
        st = m.header.stamp
        t = st.sec + st.nanosec * 1e-9 if (st.sec or st.nanosec) else time.time()
        vx, wz = self.f.step(t, self.pose, pts)
        if self.f.state == 'done':
            self._end(self.f.result, self.f.why)
            return
        if (self.stop_at_gap and vx == 0.0 and abs(getattr(self.f, 'dist', 99) - GAP) <= DIST_DEADBAND
                and abs(getattr(self.f, 'err', 1.0)) < math.radians(10)):
            self.f.state, self.f.result, self.f.why = 'done', 'reached', f'{self.f.dist:.2f} m from them'
            self._end('reached', self.f.why)
            return
        c = Twist()
        c.linear.x, c.angular.z = float(vx), float(wz)
        self.cmd.publish(c)
        self._report()

    def _pose_unsure(self):
        """goal_exec_node._pose_unsure, the same rules."""
        if time.time() - self.pts_t > SCAN_STALE_S:
            return f'no LiDAR scan for {time.time() - self.pts_t:.1f} s'
        s = self.fstatus
        if s is None or time.time() - self.fstatus_t > 2.5:
            return 'no /fusion/status'
        if not s.get('gyro_alive', True):
            return 'gyro stalled: the pose is frozen'
        if not s.get('lidar_ok', False):
            return 'LiDAR odometry unhealthy'
        if float(s.get('sd_cm', 0.0)) > SD_MAX_CM:
            return f'pose sd {s.get("sd_cm")} cm > {SD_MAX_CM}'
        return ''

    # ── outputs ─────────────────────────────────────────────────────────────
    def _stop(self):
        for _ in range(3):
            self.cmd.publish(Twist())

    def _end(self, result, why, stop=True):
        # NOT inferred from the follower's state: it finishes ITSELF on lost /
        # blocked, and this must still stop, release YOLO and report it
        if not self.active:
            return
        self.active = False
        self.pending = None
        if self.f.state == 'following':
            self.f._finish(result, why)
        else:
            self.f.state, self.f.result, self.f.why = 'done', result, why
        if stop:
            self._stop()
        self.dobs.set_active(False)
        if self.people_sub is not None:
            self.destroy_subscription(self.people_sub)   # YOLO back to idle
            self.people_sub = None
        self.get_logger().info(f'follow {result}: {why}')
        self._report(force=True)

    def _report(self, force=False, extra=None):
        d = self.f.status()
        if self.dry:
            d['dry_run'] = True
        if extra:
            d.update(extra)
        key = (d['state'], d.get('result'), d.get('target'), bool(d.get('blocked')), d.get('paused'))
        now = time.time()
        if force or key != self.last_key or (d['state'] == 'following' and now - self.last_report > 0.5):
            self.last_key, self.last_report = key, now
            self.pub_status.publish(String(data=json.dumps(d)))


def _spin(node):
    """EventsExecutor, as goal_exec_node._spin (ROVER_EXECUTOR=single: rclpy.spin)."""
    if os.environ.get("ROVER_EXECUTOR", "events") != "events":
        rclpy.spin(node)
        return
    from rclpy.experimental import EventsExecutor
    ex = EventsExecutor()
    ex.add_node(node)
    ex.spin()


def main():
    rclpy.init()
    node = FollowNode()
    try:
        _spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()


if __name__ == '__main__':
    main()
