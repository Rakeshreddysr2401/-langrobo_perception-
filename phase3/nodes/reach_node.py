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
    Within AT_GOAL_M with only the final turn refused counts as reached.
"""
import json
import math
import signal
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.action import ActionClient
from rclpy.time import Time
from std_msgs.msg import Empty, String

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'tools'))
import gap_pass as GP  # noqa: E402
from goal_exec import FRONT as GE_FRONT, REAR as GE_REAR, SIDE, wrap  # noqa: E402

EXACT_RANGE = 1.2      # m: closer than this, goal_exec alone (its MAX_LEG is 1.5)
MAX_ATTEMPTS = 8
MAX_S = 300.0
WAIT_S = 3.0
NAV_TIMEOUT = 120.0
PASS_D = 0.9           # m: how far a recovery pass crawls
# On "no path" the pass is measured from where the rover STANDS, before any
# approach: longest first, so the crawl ends past the tight bit, not in it
# (2026-10-03, 45 cm S-gap: approach drove nav2 into the gap mouth, MPPI
# turned in it and touched the low chair leg; from 35 cm back the straight
# line was 51 cm wide -- room, 44 needed).
PASS_D_FAR = (1.8, 1.3, 0.9)
MARGIN = 0.01          # = goal_exec MARGIN and nav2 footprint_padding (1 cm hard since 2026-09-27 evening, NAV_PLAN.md N1; 0.02, 0.03 before)
# A PASS keeps 2 cm a side, not MARGIN: goal_exec refuses a pass request under
# 0.02 m (its blind-ish creep past an edge at 5 cm/s), and reach sent MARGIN
# (1 cm since 2026-09-27) -- so from then on EVERY pass was refused, "a pass
# needs margin >= 0.02 m" (45 cm gap, 2026-10-03). 38 cm rover + 2 x 2 = 42 cm.
PASS_MARGIN = 0.02
# At the spot, only the final turn to the goal heading refused ("... in the
# +84 deg swing"): that IS arrival. For an approach the goal heading faces
# the object, and the thing in the swing is usually the object itself or the
# wall beside it. Before this, reach retried the refused turn 8 times over
# 3 min and reported the whole trip FAILED with the rover 1 cm from the goal
# (2026-09-27, "go to the white box"). Reported as reached, with a note.
AT_GOAL_M = 0.05
# "There" = nav2's own goal tolerances (nav2.yaml xy 0.10, yaw 0.25). Drive
# test 2026-09-27: nav2 had a goal 2 m behind the rover 5 cm away in 32 s;
# the exact finish's docking line was then blocked, the attempt counted as
# failed, and reach squeezed, waited and retried for 145 s and ended 41 cm
# off. A goal inside these is reached, whatever the exact finish said.
ARRIVED_M = 0.10
ARRIVED_YAW = 0.25
# after nav2 reports success: the exact finish refused or stuck still leaves
# the rover where nav2 put it; allow for a few cm of pose noise on top
NAV2_ARRIVED_M = 0.15
# nav2 landed this close: finish with a turn on the spot, not goal_exec's
# docking line (which backs up to a pre-goal first). = nav2's xy goal
# tolerance, so after a nav2 route the finish is ALWAYS turn-only: drive test
# 2026-09-28, nav2 landed 3.9 / 7 / 8 cm off three times and each docking
# finish backed into clutter and was refused. The price: a long route ends
# within ~10 cm, not ~1 cm. Short goals (<= EXACT_RANGE), which go to
# goal_exec from the start, keep its 1 cm docking.
FINISH_NEAR = 0.10
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
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.ge_goal = self.create_publisher(PoseStamped, '/goal_exec/goal', 10)
        self.ge_turn = self.create_publisher(PoseStamped, '/goal_exec/turn', 10)
        self.pub_status = self.create_publisher(String, '/reach/status', 10)
        self.clear_l = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.clear_g = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.pending = None
        self.goal_stamp = None
        self.stop_req = False
        self.create_subscription(PoseStamped, '/reach/goal', self._goal, 10)
        # the local costmap MPPI drives by: the escape must see what nav2 sees
        self.lcm = self.gcm = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: setattr(self, 'lcm', m), latched)
        # the planner's own costmap: a snapped goal must be one IT accepts
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: setattr(self, 'gcm', m), latched)
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

    def arrived(self, g):
        return self.dist(g) <= ARRIVED_M and abs(wrap(g[2] - self.pose[2])) <= ARRIVED_YAW

    def stop(self):
        for _ in range(3):
            self.cmd.publish(Twist())

    def clear_costmaps(self):
        # LOCAL only: nav2 segfaulted seconds after global clears (2026-09-27);
        # the global costmap clears itself (LiDAR raytracing, nvblox re-read).
        for c in (self.clear_l,):
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
        self.nav_gh = gh                           # cancelled on shutdown (main)
        res = gh.get_result_async()
        while not res.done():
            self.spin_for(0.1)
            if self.stop_req or time.time() - t0 > NAV_TIMEOUT:
                gh.cancel_goal_async()
                self.spin_for(0.5)
                # say WHY it ran out of time: a far goal whose route kept
                # closing timed out as "nav2 timed out", which hid the "no
                # path" that approach-and-look answers (2026-09-28)
                return ('cancelled', 'nav2 timed out') if self.stop_req else \
                    ('failed', f'nav2 timed out ({self.nav_reason()})')
        self.nav_gh = None
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
        # ROOT CAUSE FIRST (2026-09-27): a planner failure makes the BT spin,
        # and Spin logs "Collision Ahead" -- matched first, a goal inside an
        # obstacle was reported "narrow" and reach squeezed at it. So the
        # planner's own words win, then MPPI's ('fail to compute path': every
        # sampled motion touched something), then RPP / Spin / BackUp's
        # 'collision ahead' (lower-cased).
        text = text.lower()
        for key, why in (('start occupied', 'narrow: planner says the rover is touching something (start occupied)'),
                         ('timed out while waiting for action server', 'busy: a nav2 server answered too late (Jetson overloaded)'),
                         ('no valid path found', 'no path: planner found none'),
                         ('failed to create plan', 'no path: planner found none'),
                         ('failed to plan', 'no path: planner found none'),
                         ('exceeded limit of', 'no path: planner ran out of search'),
                         ('no valid start or goal', 'narrow: planner says the start or goal touches something'),
                         ('fail to compute path', 'narrow: nav2 controller found no motion that fits'),
                         ('collision ahead', 'narrow: nav2 path follower saw collision ahead'),
                         ('patience exceeded', 'stuck: controller patience exceeded'),
                         ('failed to make progress', 'stuck: the controller made no progress')):
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

    # ── snap: a goal on or against furniture -> the nearest pose that fits ───
    # NAV_PLAN.md N6. People click ON the sofa, or right against it: every
    # long goal on 2026-09-27 was marked solid in the planner's costmap and
    # failed 8 attempts of "no path". Now the goal moves to the nearest pose
    # (same heading) where the outline has SNAP_GAP of clearance, within
    # SNAP_R, and reach says how far. Unknown floor is left alone (build and
    # go, N8); nothing that fits within SNAP_R -> the goal is kept as clicked.
    SNAP_R = 0.50                          # m: search radius
    SNAP_GAP = 0.03                        # m: real clearance wanted at the goal

    def fit(self, g):
        """(goal, moved_m, gap_m); goal unchanged when it already fits."""
        m = self.gcm
        if m is None:
            return g, 0.0, float('nan')
        a = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        r, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
        i0, j0 = int((g[0] - ox) / r), int((g[1] - oy) / r)
        if not (0 <= i0 < m.info.width and 0 <= j0 < m.info.height) or a[j0, i0] < 0:
            return g, 0.0, float('nan')     # off the map or unseen: build-and-go's job
        k = int((self.SNAP_R + 0.6) / r)
        sub = a[max(0, j0 - k):j0 + k + 1, max(0, i0 - k):i0 + k + 1]
        jj, ii = np.nonzero(sub >= 100)
        cells = np.stack([ox + (ii + max(0, i0 - k) + 0.5) * r, oy + (jj + max(0, j0 - k) + 0.5) * r], 1)
        c, s_ = math.cos(g[2]), math.sin(g[2])

        def gap_at(x, y):
            d = cells - (x, y)
            b = np.stack([c * d[:, 0] + s_ * d[:, 1], -s_ * d[:, 0] + c * d[:, 1]], 1)
            return self.body_gap(b) - self.CELL_SLACK

        g0 = gap_at(g[0], g[1])
        if g0 >= self.SNAP_GAP:
            return g, 0.0, g0
        n = int(self.SNAP_R / r)
        offs = sorted(((di * r, dj * r) for di in range(-n, n + 1) for dj in range(-n, n + 1)
                       if 0 < math.hypot(di, dj) * r <= self.SNAP_R), key=lambda o: math.hypot(*o))
        for dx, dy in offs:
            x, y = g[0] + dx, g[1] + dy
            i, j = int((x - ox) / r), int((y - oy) / r)
            if not (0 <= i < m.info.width and 0 <= j < m.info.height) or a[j, i] < 0 or a[j, i] >= 99:
                continue
            gp = gap_at(x, y)
            if gp >= self.SNAP_GAP:
                return (x, y, g[2]), math.hypot(dx, dy), gp
        return g, 0.0, g0

    # ── escape: step off the wall the controller parked us against ──────────
    # NAV_PLAN.md N4. MPPI may drive the outline to within footprint_padding
    # (1 cm) of a wall; the lattice planner then refuses that pose as its
    # START ("Start occupied"), and every retry fails the same way -- drive
    # test 2026-09-27: 24 cm from the goal, 8 attempts, 82 s. The way out is
    # a few cm STRAIGHT (a turn sweeps the corners into the wall): forward
    # first, back only a little (blind behind below the LiDAR plane), and
    # only a move whose whole sweep keeps at least today's gap.
    ESCAPE_NEAR = 0.03                     # m: pinned = something this close to the outline
    ESCAPE_STEPS = (0.05, 0.10, -0.05)     # m along the heading

    @staticmethod
    def body_gap(pts, dx=0.0):
        """Signed distance from the outline, moved dx straight ahead, to the
        nearest point (base_link); negative = inside the body."""
        if pts is None or not len(pts):
            return float('inf')
        x, y = pts[:, 0] - dx, np.abs(pts[:, 1])
        ox = np.maximum(np.maximum(-GE_REAR - x, x - GE_FRONT), 0.0)
        oy = np.maximum(y - SIDE, 0.0)
        out = np.hypot(ox, oy)
        inside = (ox == 0) & (oy == 0)
        depth = np.minimum(np.minimum(GE_FRONT - x, x + GE_REAR), SIDE - y)
        return float(np.min(np.where(inside, -depth, out)))

    CELL_SLACK = 0.018                     # m: a cell's edge is up to half a diagonal from its centre

    def costmap_cells(self):
        """LETHAL cells of the local costmap within 0.8 m, base_link, N x 2."""
        m, pose = self.lcm, self.pose
        if m is None or pose is None:
            return np.zeros((0, 2))
        a = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        j, i = np.nonzero(a >= 100)
        r = m.info.resolution
        wx = m.info.origin.position.x + (i + 0.5) * r - pose[0]
        wy = m.info.origin.position.y + (j + 0.5) * r - pose[1]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        b = np.stack([c * wx + s * wy, -s * wx + c * wy], 1)
        return b[np.hypot(b[:, 0], b[:, 1]) < 0.8]

    def gap_all(self, pts, cells, dx=0.0):
        """The closer of: raw points, and costmap cells less CELL_SLACK --
        the planner refuses what its costmap says, not what the LiDAR says."""
        return min(self.body_gap(pts, dx), self.body_gap(cells, dx) - self.CELL_SLACK)

    @staticmethod
    def outside_body(b):
        """Drop points INSIDE the outline: a real obstacle cannot be inside the
        rover. Drive test 2026-09-28: goal_exec's depth memory (anything 2 cm
        off the floor) held floor bumps the rover had driven over, the escape
        read a -11 cm "gap" with nav2's costmap clear for 59 cm, and backed up
        5 cm twice for nothing. gap_pass drops its own-chassis LiDAR points the
        same way."""
        if b is None or not len(b):
            return b
        inside = (b[:, 0] < GE_FRONT) & (b[:, 0] > -GE_REAR) & (np.abs(b[:, 1]) < SIDE)
        return b[~inside]

    def near_now(self, scans):
        """Obstacle points + costmap cells around the rover, base_link, body excluded."""
        pts = np.vstack(scans + [self.dobs.points_base(self.pose)]) if scans else None
        return self.outside_body(pts), self.outside_body(self.costmap_cells())

    def escape(self, attempt):
        """One short straight move away from contact. True if it moved."""
        pts, cells = self.near_now(self.scans)
        g0 = self.gap_all(pts, cells)
        if g0 > self.ESCAPE_NEAR:
            return False
        best = None
        for step in self.ESCAPE_STEPS:
            sweep = min(self.gap_all(pts, cells, s) for s in np.linspace(0.01 * np.sign(step), step, 6))
            end = self.gap_all(pts, cells, step)
            if sweep >= g0 - 0.002 and end > g0 + 0.01 and (best is None or end > best[1]):
                best = (step, end)
        if best is None:
            self.say(attempt=attempt, phase='escape', gap_cm=round(g0 * 100, 1), outcome='no straight move opens space')
            return False
        step, end = best
        self.say(attempt=attempt, phase='escape', gap_cm=round(g0 * 100, 1), move_cm=round(step * 100), to_gap_cm=round(end * 100, 1))
        r, why = self.drive_straight(step, g0)
        self.say(attempt=attempt, phase='escape', outcome=r, why=why)
        return r == 'reached'

    ESCAPE_VX = 0.05                       # m/s, goal_exec's pass speed

    def drive_straight(self, step, g0):
        """Exactly straight, closed on fresh points every tick. NOT goal_exec:
        its docking turns to line up first when the pose has slid (drive test
        2026-09-27: a 5 cm reverse became a +30 deg turn, refused by its own
        swing check). Stops the moment the gap to anything shrinks."""
        x0, y0, _ = self.pose
        sgn = 1.0 if step > 0 else -1.0
        t0 = time.time()
        try:
            while time.time() - t0 < abs(step) / self.ESCAPE_VX + 2.0:
                if self.stop_req:
                    return 'cancelled', ''
                moved = math.hypot(self.pose[0] - x0, self.pose[1] - y0)
                if moved >= abs(step) - 0.005:
                    return 'reached', f'{moved * 100:.1f} cm straight'
                # the LATEST scan only: the kept ones are in base_link as it was
                # up to 1 s ago, 5 cm off at this speed
                pts, cells = self.near_now(self.scans[-1:])
                if self.gap_all(pts, cells) < g0 - 0.005:
                    return 'failed', f'gap closing after {moved * 100:.1f} cm -- stopped'
                cmd = Twist()
                cmd.linear.x = sgn * self.ESCAPE_VX
                self.cmd.publish(cmd)
                self.spin_for(0.05)
            return 'failed', 'timed out'
        finally:
            self.stop()

    def run_goal_exec_turn(self, heading):
        """goal_exec TURN-ONLY: face heading where the rover stands, no drive."""
        self.status.clear()
        self.ge_turn.publish(pose_msg('odom', self.pose[0], self.pose[1], heading))
        t0 = time.time()
        while time.time() - t0 < 30:
            self.spin_for(0.1)
            if self.stop_req:
                self.cancel.publish(Empty())
                return 'cancelled', ''
            for st in self.status:
                if st.get('state') == 'done':
                    return ('reached' if st['result'] == 'reached' else 'failed'), f"goal_exec {st['result']}: {st['why']}"
        self.cancel.publish(Empty())
        return 'failed', 'goal_exec turn timed out'

    # ── approach and look: no route to the goal -> get as near as a route goes,
    # face it, let the camera map it, plan again (NAV_PLAN.md N8) ─────────────
    # Drive test 2026-09-28 (a far goal "through a critical area"): the route
    # closed as the camera saw more, and reach could only recover on the spot.
    # The owner: "it needs to go near and see the way". A route that the map
    # has not seen yet is found by going to where it can be seen.
    APPROACH_BACK = (0.5, 0.9, 1.3, 1.8, 2.4)   # m short of the goal, nearest first
    APPROACH_GAIN = 0.30                       # m closer each round, or stop
    LOOK_S = 2.5                               # s facing the goal for nvblox

    def plan_ok(self, g):
        """Does the planner have a route to g (odom)? Planning only, no motion."""
        if not self.planner.wait_for_server(timeout_sec=2):
            return False
        goal = ComputePathToPose.Goal()
        goal.goal = pose_msg('odom', *g)
        goal.use_start = False
        f = self.planner.send_goal_async(goal)
        t0 = time.time()
        while not f.done() and time.time() - t0 < 5:
            self.spin_for(0.05)
        if not f.done() or not f.result().accepted:
            return False
        r = f.result().get_result_async()
        while not r.done() and time.time() - t0 < 10:
            self.spin_for(0.05)
        return r.done() and len(r.result().result.path.poses) > 0

    def approach_and_look(self, attempt, g):
        """True if it got meaningfully nearer and looked (retry the goal now)."""
        d0 = self.dist(g)
        best = getattr(self, 'approach_best', float('inf'))
        if d0 < 0.8 or d0 > best - self.APPROACH_GAIN / 2:
            return False                        # close already, or the last round did not gain
        bearing = math.atan2(g[1] - self.pose[1], g[0] - self.pose[0])
        ux, uy = math.cos(bearing), math.sin(bearing)
        target = None
        for back in self.APPROACH_BACK:
            if back > d0 - self.APPROACH_GAIN:
                break
            cand, _, gap = self.fit((g[0] - back * ux, g[1] - back * uy, bearing))
            if gap == gap and gap < self.SNAP_GAP:   # (nan = unseen floor: let the planner judge)
                continue
            if self.plan_ok(cand):
                target = (cand, back)
                break
        if target is None:
            self.say(attempt=attempt, phase='approach', outcome='no reachable point toward the goal')
            return False
        (tx, ty, tth), back = target
        self.say(attempt=attempt, phase='approach', short_of_goal_m=back,
                 dist_m=round(math.hypot(tx - self.pose[0], ty - self.pose[1]), 2))
        r, why = self.run_nav2((tx, ty, tth))
        self.approach_best = min(best, self.dist(g))
        if self.stop_req:
            return False
        # look: face the goal and hold still so the camera maps the way
        self.face(math.atan2(g[1] - self.pose[1], g[0] - self.pose[0]))
        self.spin_for(self.LOOK_S)
        gained = d0 - self.dist(g)
        self.say(attempt=attempt, phase='approach', outcome=r, gained_m=round(gained, 2), why=why)
        return gained >= self.APPROACH_GAIN

    def look_and_pass_far(self, attempt, g):
        """Face the goal, look both ways (outline-checked), then measure a
        straight pass from here, longest first and never past the goal; crawl
        the first that fits. Returns the pass outcome ('reached', 'tight',
        'none', 'blocked', 'failed', 'cancelled')."""
        bearing = math.atan2(g[1] - self.pose[1], g[0] - self.pose[0])
        if abs(wrap(bearing - self.pose[2])) > math.radians(35):
            self.face(bearing)
        GP.look(self, GP.LOOK_DEG)
        out = 'none'
        for D in PASS_D_FAR:
            if self.stop_req:
                return 'cancelled'
            D = min(D, self.dist(g))
            out, _ = GP.attempt(self, D, PASS_MARGIN, dry=True)
            self.say(attempt=attempt, phase='pass_far', D=round(D, 2), measured=out)
            if out == 'dry':
                break
        if out != 'dry':
            return out
        out, _ = GP.attempt(self, D, PASS_MARGIN, dry=False)
        self.say(attempt=attempt, phase='pass_far', D=round(D, 2), outcome=out)
        return out

    def face(self, bearing):
        """Turn in place toward bearing (odom), outline-checked; for the look."""
        GP.look_to(self, bearing)

    # ── one goal, until reached ─────────────────────────────────────────────
    def pursue(self, m):
        # Depth is subscribed only while RECOVERING (escape / look / pass use
        # it; nav2 and goal_exec do not): ~30% of a core for the whole drive
        # before (2026-09-28, load 9 on 6 cores, MPPI starved to 8.7 Hz).
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
        self.approach_best = float('inf')
        g, moved, gap = self.fit(g)
        if moved > 0:
            self.say(phase='snap', moved_cm=round(moved * 100), gap_cm=round(gap * 100, 1),
                     why='the goal as clicked touches something; nearest pose that fits')
        snap = (g[0] - (self.goal_odom(m) or g)[0], g[1] - (self.goal_odom(m) or g)[1])
        tried = []
        # A near goal whose docking line was refused goes to nav2 next: the
        # same drive test had a goal 0.76 m behind in open floor refused 7
        # times on its line (squeeze, wait, same line) and never tried nav2.
        go_nav2 = False
        for attempt in range(1, MAX_ATTEMPTS + 1):
            self.dobs.set_active(False)             # driving: nav2 / goal_exec need none
            if self.stop_req or time.time() - t0 > MAX_S:
                break
            if m.header.frame_id == 'map':
                gm = self.goal_odom(m)              # SLAM may have corrected map -> odom
                if gm is not None:                  # (keeping the snap's offset)
                    g = (gm[0] + snap[0], gm[1] + snap[1], gm[2])
            d = self.dist(g)
            line_refused = False
            if d > EXACT_RANGE or go_nav2:
                self.say(attempt=attempt, phase='nav2', dist_m=round(d, 2))
                r, why = self.run_nav2(g)
                if r == 'reached':
                    self.say(attempt=attempt, phase='finish', dist_m=round(self.dist(g), 3))
                    # Within FINISH_NEAR a full docking finish does more harm
                    # than good: goal_exec lines up by backing to a pre-goal
                    # 35 cm out, which in clutter hits something (2026-09-28:
                    # nav2 left it 3.9 cm off, the finish backed 25 cm and was
                    # refused, and the goal took two more attempts). There,
                    # turn on the spot to the goal heading -- or nothing.
                    if self.dist(g) <= FINISH_NEAR:
                        yerr = abs(wrap(g[2] - self.pose[2]))
                        r2, why2 = (('reached', 'heading already within 3 deg') if yerr <= math.radians(3)
                                    else self.run_goal_exec_turn(g[2]))
                    else:
                        r2, why2 = self.run_goal_exec(g)
                    # nav2 has ARRIVED (its goal checker: 10 cm, 14 deg). The
                    # exact finish only improves on that; refused or stuck, it
                    # never undoes it -- drive test 2026-09-27: refused
                    # finishes re-ran nav2 + squeeze 5 times, 8.3 m driven for
                    # a 1.5 m route, 181 s, timed out 4 cm from the goal.
                    if r2 != 'reached' and self.dist(g) <= NAV2_ARRIVED_M:
                        self.stop()
                        self.say(result='reached', attempt=attempt, dist_cm=round(self.dist(g) * 100, 1),
                                 secs=round(time.time() - t0), tried=tried,
                                 note=f'nav2 arrived; the exact finish did not fit ({why2})')
                        return
                    r, why = r2, why2
            else:
                self.say(attempt=attempt, phase='goal_exec', dist_m=round(d, 2))
                r, why = self.run_goal_exec(g)
                line_refused = r == 'failed' and 'refused' in why
            if r == 'failed' and self.arrived(g):
                self.stop()
                self.say(result='reached', attempt=attempt, dist_cm=round(self.dist(g) * 100, 1),
                         secs=round(time.time() - t0), tried=tried,
                         note=f'within nav2 tolerance; the exact finish did not fit ({why})')
                return
            if r == 'reached':
                self.say(result='reached', attempt=attempt, dist_cm=round(self.dist(g) * 100, 1),
                         secs=round(time.time() - t0), tried=tried)
                return
            if r == 'cancelled' or self.stop_req:
                break
            if 'swing' in why and self.dist(g) <= AT_GOAL_M:
                self.stop()
                self.say(result='reached', attempt=attempt, dist_cm=round(self.dist(g) * 100, 1),
                         secs=round(time.time() - t0), tried=tried,
                         note=f'at the spot, but could not turn fully to the goal heading ({why})')
                return
            tried.append(why)
            if line_refused and not go_nav2:
                go_nav2 = True
                self.say(attempt=attempt, phase='recover', why=why + ' -- nav2 next')
                continue
            self.say(attempt=attempt, phase='recover', why=why)
            # ── recover: stop, clear, look, then pass if narrow, else wait ──
            self.stop()
            self.dobs.set_active(True)              # recovering: fresh depth first
            self.clear_costmaps()                   # (spins 0.5 s)
            self.spin_for(1.0)
            # parked against something (MPPI may go to the 1 cm line; the
            # planner will not start from it): step off first, then retry at
            # once -- no look, no pass, no wait
            if self.escape(attempt):
                continue
            # no route to the goal: look both ways and try a straight pass from
            # HERE first; only if no line fits, go as near as a route goes
            looked = False
            if 'no path' in why:
                out = self.look_and_pass_far(attempt, g)
                if out == 'reached':
                    continue
                if out == 'cancelled' or self.stop_req:
                    break
                looked = True
                # moved partway ('failed'/'blocked'): wait, then re-plan from there
                if out not in ('failed', 'blocked') and self.approach_and_look(attempt, g):
                    continue
            bearing = math.atan2(g[1] - self.pose[1], g[0] - self.pose[0])
            if not looked:
                if abs(wrap(bearing - self.pose[2])) > math.radians(35) and self.dist(g) > 0.3:
                    self.face(bearing)
                GP.look(self, GP.LOOK_DEG)
            narrow = why.startswith('narrow') or 'obstacle' in why or 'stuck' in why
            if narrow:
                D = min(PASS_D, max(0.4, self.dist(g)))
                self.say(attempt=attempt, phase='pass', D=round(D, 2))
                out, _ = GP.attempt(self, D, PASS_MARGIN, dry=False)
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
    # A goal with no pose used to wait here SILENTLY, forever: 2026-09-27 a
    # reach started during a nav2 restart never received one /odom message
    # (a fresh process in the same container got 20 Hz), and four RViz goals
    # vanished with nothing in any log. Now it says so and gives the goal up.
    NO_POSE_S = 3.0
    waiting_since = None
    # Say so at STARTUP too, not only when a goal arrives: `./rover logs reach`
    # is then the whole check after a restart (STARTUP.md, "reach has no pose").
    started, warned = time.time(), False
    # A killed reach used to leave its nav2 goal DRIVING with nobody watching
    # (2026-09-28: restarted mid-goal; nav2 ran on 20 s). SIGTERM (pkill,
    # ./rover nav) now takes the same way out as Ctrl-C: cancel, then stop.
    n.nav_gh = None
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        while rclpy.ok():
            n.spin_for(0.1)
            if started is not None and n.pose is not None:
                n.get_logger().info(f'pose OK: /odom arriving ({time.time() - started:.1f} s after start)')
                started = None
            elif started is not None and not warned and time.time() - started > 5.0:
                warned = True
                n.get_logger().error('NO POSE: no /odom in 5 s since start -- goals will be dropped; restart reach (./rover nav)')
            if n.pending is not None and n.pose is not None:
                m, n.pending, waiting_since = n.pending, None, None
                n.pursue(m)
            elif n.pending is not None:
                waiting_since = waiting_since or time.time()
                if time.time() - waiting_since > NO_POSE_S:
                    m, n.pending, waiting_since = n.pending, None, None
                    n.goal_stamp = f'{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}'
                    n.get_logger().error('goal dropped: no pose from /odom since reach started -- restart it (./rover nav)')
                    n.say(result='failed', why='no pose: reach has received no /odom')
    except KeyboardInterrupt:
        pass
    if n.nav_gh is not None:
        n.nav_gh.cancel_goal_async()
        n.spin_for(0.3)
    n.cancel.publish(Empty())                  # goal_exec, if it was driving for us
    n.stop()


if __name__ == '__main__':
    main()
