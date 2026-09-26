#!/usr/bin/env python3
"""reach_node.py — get to the goal: retry, re-look, re-plan, until there.

    /reach/goal     geometry_msgs/PoseStamped (odom or map) -- RViz "2D Goal Pose"
    /reach/cancel   std_msgs/Empty
        -> /reach/status  JSON: attempt, phase, why, result, and goal_stamp --
                          the goal's header.stamp ("sec.nanosec"), so a caller
                          (the Pi 5 brain) can tell its goal's final line from
                          the "cancelled" of the goal it just preempted

WHY
    nav2 alone gives up on the first bad spell: in a 51 cm corridor its path
    follower saw "collision ahead" on every wobble (its padded outline leaves
    ~1.5 cm a side there), its Spin and BackUp recoveries were refused for the
    same reason, and the goal failed in 15 s (2026-09-26). The owner's rule:
    reach the goal -- look again, analyse, retry -- but never by switching a
    collision check off.

EACH ATTEMPT
    far  (> EXACT_RANGE)  nav2 to the goal, then goal_exec's exact finish
    near (<= EXACT_RANGE) goal_exec straight away (turn, straight line, 5 cm)
    fails ->
      1. stop; clear both costmaps (stale marks from people close gaps)
      2. LOOK: face the goal if it is off to the side, then ±25 deg -- the
         camera sees 87 deg and nothing at its nose; this fills in both
      3. narrow ahead? (nav2 "collision ahead", goal_exec "obstacle ... leg",
         or a strip bounded both sides toward the goal): measure the gap from
         the raw LiDAR + depth points (1 cm) and PASS it (3 cm margin, 5 cm/s)
      4. otherwise wait WAIT_S (a person, a moved chair) and go again
    until reached, MAX_ATTEMPTS, MAX_S, or a new goal / cancel.
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.time import Time
from std_msgs.msg import Empty, String

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'tools'))
import gap_pass as GP  # noqa: E402
from goal_exec import SIDE, wrap  # noqa: E402

EXACT_RANGE = 1.2      # m: closer than this, goal_exec alone (its MAX_LEG is 1.5)
MAX_ATTEMPTS = 8
MAX_S = 300.0
WAIT_S = 3.0
NAV_TIMEOUT = 120.0
PASS_D = 0.9           # m: how far a recovery pass crawls
MARGIN = 0.03
NAV_LOG = Path('/tmp/nav.log')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose_msg(frame, x, y, th):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
    return p


class Reach(GP.Pass):
    def __init__(self):
        super().__init__('reach', depth_active=False)   # depth only while pursuing a goal
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.ge_goal = self.create_publisher(PoseStamped, '/goal_exec/goal', 10)
        self.pub_status = self.create_publisher(String, '/reach/status', 10)
        self.clear_l = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.clear_g = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.pending = None
        self.goal_stamp = None
        self.stop_req = False
        self.create_subscription(PoseStamped, '/reach/goal', self._goal, 10)
        self.create_subscription(Empty, '/reach/cancel', lambda _: setattr(self, 'stop_req', True), 10)
        self.get_logger().info('reach up: /reach/goal (RViz 2D Goal Pose)')

    def _goal(self, m):
        self.pending = m
        self.stop_req = True                       # preempt whatever runs

    # ── helpers ─────────────────────────────────────────────────────────────
    def say(self, **d):
        d['t'] = round(time.time(), 1)
        if self.goal_stamp:
            d['goal_stamp'] = self.goal_stamp
        self.pub_status.publish(String(data=json.dumps(d)))
        self.get_logger().info(json.dumps(d))

    def goal_odom(self, m):
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

    def dist(self, g):
        return math.hypot(g[0] - self.pose[0], g[1] - self.pose[1])

    def stop(self):
        for _ in range(3):
            self.cmd.publish(Twist())

    def clear_costmaps(self):
        for c in (self.clear_l, self.clear_g):
            if c.service_is_ready():
                c.call_async(ClearEntireCostmap.Request())
        self.spin_for(0.5)

    def run_nav2(self, g):
        if not self.nav.wait_for_server(timeout_sec=3):
            return 'failed', 'nav2 not up'
        try:
            self.log_from = NAV_LOG.stat().st_size      # only this attempt's lines explain it
        except OSError:
            self.log_from = 0
        goal = NavigateToPose.Goal()
        goal.pose = pose_msg('odom', *g)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        fut = self.nav.send_goal_async(goal)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5:
            self.spin_for(0.05)
        gh = fut.result() if fut.done() else None
        if gh is None or not gh.accepted:
            return 'failed', 'nav2 rejected the goal'
        res = gh.get_result_async()
        while not res.done():
            self.spin_for(0.1)
            if self.stop_req or time.time() - t0 > NAV_TIMEOUT:
                gh.cancel_goal_async()
                self.spin_for(0.5)
                return 'cancelled' if self.stop_req else 'failed', 'nav2 timed out'
        st = res.result().status
        if st == GoalStatus.STATUS_SUCCEEDED:
            return 'reached', ''
        return 'failed', self.nav_reason()

    def nav_reason(self):
        """What nav2 last complained about (its log is the only place it says)."""
        try:
            with NAV_LOG.open('rb') as f:
                f.seek(getattr(self, 'log_from', 0))
                text = f.read().decode(errors='ignore')
        except OSError:
            return 'nav2 failed'
        for key, why in (('collision ahead', 'narrow: nav2 path follower saw collision ahead'),
                         ('failed to plan', 'no path: planner found none'),
                         ('patience exceeded', 'stuck: controller patience exceeded')):
            if key in text:
                return why
        return 'nav2 failed'

    def run_goal_exec(self, g):
        self.status.clear()
        self.ge_goal.publish(pose_msg('odom', *g))
        t0 = time.time()
        while time.time() - t0 < 120:
            self.spin_for(0.1)
            if self.stop_req:
                self.cancel.publish(Empty())
                return 'cancelled', ''
            for st in self.status:
                if st.get('state') == 'done':
                    return ('reached' if st['result'] == 'reached' else 'failed'), f"goal_exec {st['result']}: {st['why']}"
        self.cancel.publish(Empty())
        return 'failed', 'goal_exec timed out'

    def face(self, bearing):
        """Turn in place toward bearing (odom), outline-checked; for the look."""
        GP.look_to(self, bearing)

    # ── one goal, until reached ─────────────────────────────────────────────
    def pursue(self, m):
        self.dobs.set_active(True)
        try:
            self._pursue(m)
        finally:
            self.dobs.set_active(False)

    def _pursue(self, m):
        self.goal_stamp = f'{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}'
        self.stop_req = False
        t0 = time.time()
        g = self.goal_odom(m)
        if g is None:
            self.say(result='failed', why=f'no transform {m.header.frame_id} -> odom')
            return
        tried = []
        for attempt in range(1, MAX_ATTEMPTS + 1):
            if self.stop_req or time.time() - t0 > MAX_S:
                break
            if m.header.frame_id == 'map':
                g = self.goal_odom(m) or g          # SLAM may have corrected map -> odom
            d = self.dist(g)
            if d > EXACT_RANGE:
                self.say(attempt=attempt, phase='nav2', dist_m=round(d, 2))
                r, why = self.run_nav2(g)
                if r == 'reached':
                    self.say(attempt=attempt, phase='finish', dist_m=round(self.dist(g), 3))
                    r, why = self.run_goal_exec(g)
            else:
                self.say(attempt=attempt, phase='goal_exec', dist_m=round(d, 2))
                r, why = self.run_goal_exec(g)
            if r == 'reached':
                self.say(result='reached', attempt=attempt, dist_cm=round(self.dist(g) * 100, 1),
                         secs=round(time.time() - t0), tried=tried)
                return
            if r == 'cancelled' or self.stop_req:
                break
            tried.append(why)
            self.say(attempt=attempt, phase='recover', why=why)
            # ── recover: stop, clear, look, then pass if narrow, else wait ──
            self.stop()
            self.clear_costmaps()
            bearing = math.atan2(g[1] - self.pose[1], g[0] - self.pose[0])
            if abs(wrap(bearing - self.pose[2])) > math.radians(35) and self.dist(g) > 0.3:
                self.face(bearing)
            GP.look(self, GP.LOOK_DEG)
            narrow = why.startswith('narrow') or 'obstacle' in why or 'stuck' in why or 'no path' in why
            if narrow:
                D = min(PASS_D, max(0.4, self.dist(g)))
                self.say(attempt=attempt, phase='pass', D=round(D, 2))
                out, _ = GP.attempt(self, D, MARGIN, dry=False)
                self.say(attempt=attempt, phase='pass', outcome=out)
                if out == 'reached':
                    continue                        # through the tight bit: try the goal again
            self.say(attempt=attempt, phase='wait', secs=WAIT_S)
            self.spin_for(WAIT_S)
        self.stop()
        self.say(result='cancelled' if self.stop_req else 'failed',
                 why=f'gave up after {len(tried)} attempts, {time.time() - t0:.0f} s', tried=tried)


def main():
    rclpy.init()
    n = Reach()
    try:
        while rclpy.ok():
            n.spin_for(0.1)
            if n.pending is not None and n.pose is not None:
                m, n.pending = n.pending, None
                n.pursue(m)
    except KeyboardInterrupt:
        pass
    n.stop()


if __name__ == '__main__':
    main()
