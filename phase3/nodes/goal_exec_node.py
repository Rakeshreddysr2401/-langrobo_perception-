#!/usr/bin/env python3
"""goal_exec_node.py — exact (x, y, θ) moves, live (SENSOR_FUSION_PLAN.md M4).

    /goal_exec/goal     geometry_msgs/PoseStamped   frame odom or map
    /goal_exec/pass     std_msgs/String JSON {x, y, th, frame, L, margin, vx}:
                        a pass through a tight gap (goal_exec.py PASS MODE;
                        sent by ./rover pass, which finds the gap)
    /goal_exec/turn     geometry_msgs/PoseStamped   turn in place to the pose's
                        heading, and only that (goal_exec.py TURN-ONLY MODE;
                        the Pi 5 brain's L:/R: and "face it"). Position ignored
    /goal_exec/cancel   std_msgs/Empty
        -> /cmd_vel             while a goal is active, and a stop when it ends
           /goal_exec/status    JSON, every change: state, result, why, errors,
                                and goal_stamp: the goal message's header.stamp
                                ("sec.nanosec"), so a caller can tell its own
                                goal's result from the one it preempted
           /goal_exec/obstacles PointCloud2 (odom), 2 Hz: what the depth camera
                                has seen at the rover's height, 1 cm -- the
                                points goal_exec and ./rover pass decide on

The logic is phase3/nodes/goal_exec.py, the class the simulator proved
(phase3/tools/sim_goal_exec.py: 180 goals, 177 reached, worst 2.0 cm).

INPUTS, and why each
    /odom           fusion2: LiDAR-anchored, ~0.3 cm. The controller's pose.
    /scan           the swept-outline and corridor checks (base_link points)
    depth camera    + what it sees at the rover's height, 1 cm, remembered in
                    odom (depth_obstacles.py): the LiDAR's one plane at 25 cm
                    misses low things and passes between a stool's legs
    /fusion/status  the pose's own confidence. The rover does not move on a
                    pose it cannot trust: LiDAR unhealthy, or sd > 3 cm, or
                    the status silent, and it PAUSES (zero command); after
                    PAUSE_S it gives the goal up.

A map-frame goal is converted to odom once, through the live map -> odom, and
re-converted every time it re-plans, so a SLAM correction mid-goal is used.

The learned pivot is saved to /logs/goal_exec_pivot.json after every goal and
loaded at start, so a session begins with what the last one learned.

It drives only while it has a goal; nav2 and the phone teleop share /cmd_vel,
so do not run a nav2 goal at the same time. Teleop MANUAL overrides it.
"""
import json
import math
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
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parent))
from depth_obstacles import MIN_HITS, DepthObstacles  # noqa: E402
from goal_exec import GoalExec, wrap  # noqa: E402

PIVOT_FILE = Path('/logs/goal_exec_pivot.json')
SD_MAX_CM = 3.0
DEPTH_STALE_S = 1.0             # pass mode stops without a camera update this recent
SCAN_STALE_S = 0.5              # no scan this recent: the checks would run on a frozen view
PAUSE_S = 5.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class GoalExecNode(Node):
    def __init__(self):
        super().__init__('goal_exec')
        prior = None
        try:
            prior = json.loads(PIVOT_FILE.read_text())
        except (OSError, ValueError):
            pass
        self.ex = GoalExec(prior)
        self.pose = None
        self.pts = None
        self.pts_t = 0.0
        self.fstatus, self.fstatus_t = None, 0.0
        self.goal_msg = None
        self.paused_since = None
        self.last_status = None
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/goal_exec/status', 10)
        self.pub_obs = self.create_publisher(PointCloud2, '/goal_exec/obstacles', 1)
        self.n_odom = 0
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(String, '/fusion/status', self._fstatus, 10)
        self.create_subscription(PoseStamped, '/goal_exec/goal', self._goal, 10)
        self.create_subscription(String, '/goal_exec/pass', self._pass, 10)
        self.create_subscription(PoseStamped, '/goal_exec/turn', self._turn, 10)
        self.create_subscription(Empty, '/goal_exec/cancel', self._cancel, 10)
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.mount = None
        self.pass_ = None
        self.dobs = DepthObstacles(self, self.buf, lambda: self.pose)
        self.get_logger().info(f'goal_exec up; pivot prior {prior}')

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

    def _pass(self, m):
        try:
            d = json.loads(m.data)
            p = PoseStamped()
            p.header.frame_id = d.get('frame', 'odom')
            p.pose.position.x, p.pose.position.y = float(d['x']), float(d['y'])
            p.pose.orientation.z, p.pose.orientation.w = math.sin(d['th'] / 2), math.cos(d['th'] / 2)
            pass_ = dict(L=float(d['L']), margin=float(d['margin']), vx=float(d['vx']))
        except (ValueError, KeyError, TypeError) as e:
            self._say('refused', f'bad pass request: {e}')
            return
        if pass_['margin'] < 0.02 or pass_['vx'] > 0.08:
            self._say('refused', 'a pass needs margin >= 0.02 m and vx <= 0.08 m/s')
            return
        self._goal(p, pass_)

    def _turn(self, m):
        if self.pose is None:
            self.goal_msg = m
            self._say('refused', 'no pose yet')
            return
        self._goal(m, turn_only=True)

    def _goal(self, m, pass_=None, turn_only=False):
        self.goal_msg = m
        g = self._goal_in_odom()
        if g is None:
            self._say('refused', f'no transform {m.header.frame_id} -> odom')
            return
        if turn_only:
            g = (self.pose[0], self.pose[1], g[2])     # the turn starts here; its slide is reported
        self.ex.set_goal(g, pass_=pass_, turn_only=turn_only)
        self.paused_since = None
        self.get_logger().info(f'{"turn" if turn_only else "goal"} ({g[0]:.3f}, {g[1]:.3f}, '
                               f'{math.degrees(g[2]):.1f} deg) in odom from {m.header.frame_id}')
        self._report(force=True)

    def _cancel(self, _):
        if self.ex.state not in ('idle', 'done'):
            self.ex.cancel('cancelled')
            self._stop()
            self._report(force=True)

    def _goal_in_odom(self):
        m = self.goal_msg
        p, q = m.pose.position, m.pose.orientation
        g = (p.x, p.y, yaw_of(q))
        frame = m.header.frame_id or 'odom'
        if frame == 'odom':
            return g
        if not self.buf.can_transform('odom', frame, Time()):
            return None
        tr = self.buf.lookup_transform('odom', frame, Time()).transform
        th = yaw_of(tr.rotation)
        c, s = math.cos(th), math.sin(th)
        return (tr.translation.x + c * g[0] - s * g[1], tr.translation.y + s * g[0] + c * g[1], wrap(th + g[2]))

    # ── the loop: one step per fused pose (20 Hz) ───────────────────────────
    def _odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.n_odom += 1
        if self.n_odom % 10 == 0:                 # 2 Hz, on the pose (no timers: fusion2 lesson)
            mem = self.dobs.mem
            mem = mem[mem[:, 4] >= MIN_HITS] if len(mem) else mem
            m.header.frame_id = 'odom'
            self.pub_obs.publish(point_cloud2.create_cloud_xyz32(m.header, mem[:, :3].tolist() if len(mem) else []))
        if self.ex.state in ('idle', 'done'):
            return
        unsure = self._pose_unsure()
        if unsure:
            self._stop()
            self.paused_since = self.paused_since or time.time()
            if time.time() - self.paused_since > PAUSE_S:
                self.ex.cancel(f'pose unsure for {PAUSE_S:.0f} s: {unsure}')
                self._finish()
            else:
                self._report(extra={'paused': unsure})
            return
        self.paused_since = None
        if (self.ex.state == 'plan' and self.goal_msg is not None and self.goal_msg.header.frame_id == 'map'
                and not self.ex.turn_only):        # a turn's x, y are where it started, not the message's
            g = self._goal_in_odom()               # a SLAM correction since the goal arrived
            if g is not None:
                self.ex.goal = np.array(g)
                self.ex.u = np.array([math.cos(g[2]), math.sin(g[2])])
        if self.ex.pass_ and self.dobs.age() > DEPTH_STALE_S:
            # a pass is judged on the camera as much as the LiDAR (a stool's
            # legs are camera-only): no fresh camera view, no pass
            self.ex.cancel(f'depth camera view {self.dobs.age():.1f} s old: not passing blind')
            self._finish()
            return
        pts = self.pts
        dp = self.dobs.points_base(self.pose)
        if len(dp):
            pts = dp if pts is None else np.vstack([pts, dp])
        # the controller's clock is the POSE's stamp: arrival times on this
        # loaded Jetson bunch up (0.6-441 ms apart for poses stamped 50 ms apart)
        st = m.header.stamp
        t = st.sec + st.nanosec * 1e-9 if (st.sec or st.nanosec) else time.time()
        vx, wz = self.ex.step(t, self.pose, pts)
        if self.ex.state == 'done':
            self._finish()
            return
        t = Twist()
        t.linear.x, t.angular.z = float(vx), float(wz)
        self.cmd.publish(t)
        self._report()

    def _pose_unsure(self):
        if time.time() - self.pts_t > SCAN_STALE_S:
            return f'no LiDAR scan for {time.time() - self.pts_t:.1f} s'
        s = self.fstatus
        if s is None or time.time() - self.fstatus_t > 2.5:
            return 'no /fusion/status'
        if not s.get('lidar_ok', False):
            return 'LiDAR odometry unhealthy'
        if float(s.get('sd_cm', 0.0)) > SD_MAX_CM:
            return f'pose sd {s.get("sd_cm")} cm > {SD_MAX_CM}'
        return ''

    # ── outputs ─────────────────────────────────────────────────────────────
    def _stop(self):
        for _ in range(3):
            self.cmd.publish(Twist())

    def _finish(self):
        self._stop()
        try:
            PIVOT_FILE.write_text(json.dumps(self.ex.pivot_state()))
        except OSError:
            pass
        self.get_logger().info(f'goal {self.ex.result}: {self.ex.why}')
        self._report(force=True)

    def _say(self, result, why):
        self.ex.state, self.ex.result, self.ex.why = 'done', result, why
        self._report(force=True)

    def _report(self, force=False, extra=None):
        d = {'state': self.ex.state, 'result': self.ex.result, 'why': self.ex.why, 'tries': self.ex.tries}
        if self.goal_msg is not None:
            st = self.goal_msg.header.stamp
            d['goal_stamp'] = f'{st.sec}.{st.nanosec:09d}'
        if self.ex.turn_only:
            d['turn_only'] = True
        if self.ex.pass_:
            d['pass'] = True
            d['contacts'] = [list(x) for x in self.ex.log[-3:]]
        if self.ex.goal is not None and self.pose is not None:
            g = self.ex.goal
            d['err_cm'] = round(math.hypot(g[0] - self.pose[0], g[1] - self.pose[1]) * 100, 1)
            d['err_deg'] = round(math.degrees(wrap(g[2] - self.pose[2])), 2)
        if extra:
            d.update(extra)
        key = (d['state'], d['result'], d.get('paused'), getattr(self.ex, 'turn_purpose', None))
        if force or key != self.last_status:
            self.last_status = key
            self.pub_status.publish(String(data=json.dumps(d)))


def main():
    rclpy.init()
    n = GoalExecNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n._stop()
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
