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
      0. "no path" or narrow: the TIGHT ROUTE first (tight_route.py) -- turns
         and short drives that fit the outline, searched on the raw points,
         driven one goal_exec move at a time, re-planned after each. nav2's
         lattice cannot turn in place near a wall; this can
         (2026-10-10, the bedroom door). Then:
         "no path": the WAY BACK (trail.py) -- look down the route the
         rover drove, then retrace it in ~1 m nav2 hops to its point nearest
         the goal. The same failure SAME_FAILS times running from one spot
         ends the goal: more rounds would be the same move again
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
import os
import signal
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.action import ActionClient
from rclpy.time import Time
from std_msgs.msg import ColorRGBA, Empty, String
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'tools'))
import gap_pass as GP  # noqa: E402
import tight_route as TRT  # noqa: E402
import trail as TR  # noqa: E402
import escape_moves as EM  # noqa: E402
import house_route as HR  # noqa: E402
from goal_exec import FRONT as GE_FRONT, REAR as GE_REAR, SIDE, GoalExec, wrap  # noqa: E402

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
# A goal beyond the planner's window goes in LEGS. The global costmap is an
# 8 x 8 m window that follows the rover (there is no map frame): a goal more
# than ~4 m away is "outside bounds" and nav2 fails at once. Found in the
# The twin, 2026-10-04: bedroom -> living room, 6.2 m, eight instant
# "nav2 failed" in 117 s; in a real home, any room past ~4 m. Each leg: the
# farthest point toward the goal, up to LEG_MAX, that fits and plans.
LEG_MAX = 3.2          # m: well inside the 4 m half-window
# A leg's end is a WAYPOINT, not a goal: within LEG_DONE_M it is done, any heading.
# Twin 2026-10-19 (apt_s1, recorded): each leg went to nav2 as a full goal (10 cm
# and 14 deg), the waypoint fell between a ball, a bin and chair legs, and MPPI
# spent 76 s within 8-23 cm of it, turning 170 deg back and forth to satisfy the
# heading -- then the next leg re-planned from there anyway.
LEG_DONE_M = 0.35
LEG_MIN = 1.0
LEG_STEP = 0.4
MAX_LEGS = 12
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
# The way back (trail.py): seconds facing down the trail before driving it, so
# the camera can clear stale marks it sees past (nvblox clears a cell only by
# seeing beyond it, 0.4-3 m out); and at most this many per goal.
WAY_BACK_LOOK_S = 4.0
WAY_BACKS = 2
# a hop is ~1 m: nav2 gets this long, not NAV_TIMEOUT -- live test 2026-10-04,
# two blocked hops at 120 s each took the goal to 541 s
WAY_BACK_HOP_S = 45.0
# The same failure from the same spot this many times in a row ends the goal:
# one repeat is allowed (a person in the way moves on during the wait), more
# is the same move again.
SAME_FAILS = 3
NAV_LOG = Path('/tmp/nav.log')
# The tight route (tight_route.py). Rounds per goal; moves per round; the
# target is the goal within TIGHT_REACH, else the farthest fitting point
# toward it. After every move the rover holds still TIGHT_SETTLE_S:
# depth_gate passes frames again 0.8 s after a turn, and nvblox must take one
# before the map shows what the turn brought into view -- that is how the
# glossy-floor speck in the door gap went away (2026-10-10).
TIGHT_ROUNDS = 2
TIGHT_MOVES = 10
TIGHT_REACH = (2.5, 2.1, 1.7, 1.3)
TIGHT_SETTLE_S = 1.5
# A plan the camera's map does not allow (LiDAR-only, see tight_round) drives
# at most this far before it looks and plans again; and gives up after this
# many moves goal_exec refused on the camera's points.
TIGHT_LIDAR_LEG = 0.5
TIGHT_REFUSALS = 2
PIVOT_FILE = Path(os.environ.get('ROVER_STATE_DIR', '/logs')) / 'goal_exec_pivot.json'


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
        # RViz: what reach is doing, over the rover, and the tight route's plan
        self.pub_mk = self.create_publisher(MarkerArray, '/reach/markers', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
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
        # the HOUSE map (slam_toolbox): rooms and doors beyond nav2's 8 m window, for routing legs
        self.house = None
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'house', m), latched)
        self.create_subscription(Empty, '/reach/cancel', lambda _: setattr(self, 'stop_req', True), 10)
        # where reach has driven: its own drives only, so a carry is not a road
        self.trail = TR.Trail()
        self.pursuing = False
        self.get_logger().info('reach up: /reach/goal (RViz 2D Goal Pose)')

    def _odom(self, m):
        super()._odom(m)
        if self.pursuing:
            self.trail.add(self.pose[0], self.pose[1])

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
        self.show_status(d)

    # ── RViz (/reach/markers, display "Reach -- what it is doing") ───────────
    # 2026-10-10: the "come to me" that failed was three "no path" and two
    # way-back hops, readable only in /tmp/reach.log. Now a line over the
    # rover says which step reach is on, and the tight route draws its plan:
    # the centre's path and the outline at the end of every move (green:
    # planned on LiDAR + camera; amber: on the LiDAR alone), and its target.
    WHITE, GREEN, AMBER, RED, CYAN = ((1.0, 1.0, 1.0), (0.2, 0.9, 0.3), (1.0, 0.65, 0.0),
                                      (1.0, 0.25, 0.2), (0.2, 0.85, 1.0))

    def _mk(self, ns, i, typ, scale, rgb, a=1.0):
        m = Marker()
        m.header.frame_id, m.header.stamp = 'odom', self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, i, typ, Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=a)
        m.pose.orientation.w = 1.0
        return m

    @staticmethod
    def status_text(d):
        """One short line from a status dict."""
        if 'result' in d:
            s = d['result'] + (f" {d['dist_cm']} cm" if 'dist_cm' in d else '')
            why = d.get('why') or d.get('note') or ''
        else:
            s = f"try {d.get('attempt', '-')}: {d.get('phase', '')}"
            why = (d.get('plan') and f"{d['plan']} ({d.get('mode')})") or \
                (d.get('move') and f"{d['move']} {d.get('outcome', '')}") or \
                d.get('why') or d.get('outcome') or ''
        s = f'{s} -- {why}' if why else s
        return s if len(s) <= 72 else s[:69] + '...'

    def show_status(self, d):
        if self.pose is None:
            return
        rgb = self.GREEN if d.get('result') == 'reached' else \
            self.RED if d.get('result') in ('failed', 'cancelled') else self.WHITE
        m = self._mk('reach_status', 0, Marker.TEXT_VIEW_FACING, 0.07, rgb)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.pose[0], self.pose[1], 0.55
        m.text = self.status_text(d)
        self.pub_mk.publish(MarkerArray(markers=[m]))

    def show_tight(self, steps, mode, target, ge):
        """The plan from where the rover stands; steps None clears it."""
        out = []
        if not steps:
            for i in range(3):
                m = self._mk('tight', i, Marker.LINE_STRIP, 0.01, self.WHITE)
                m.action = Marker.DELETE
                out.append(m)
            self.pub_mk.publish(MarkerArray(markers=out))
            return
        rgb = self.GREEN if mode == 'all' else self.AMBER
        poses = TRT.poses_of(steps, tuple(self.pose), ge)
        path = self._mk('tight', 0, Marker.LINE_STRIP, 0.015, rgb)
        path.points = [Point(x=p[0], y=p[1], z=0.02) for p in poses]
        boxes = self._mk('tight', 1, Marker.LINE_LIST, 0.006, rgb, 0.7)
        corners = ((GE_FRONT, SIDE), (GE_FRONT, -SIDE), (-GE_REAR, -SIDE), (-GE_REAR, SIDE))
        for x, y, th in poses[1:]:
            c, s_ = math.cos(th), math.sin(th)
            w = [Point(x=x + c * a - s_ * b, y=y + s_ * a + c * b, z=0.02) for a, b in corners]
            for k in range(4):
                boxes.points += [w[k], w[(k + 1) % 4]]
        tgt = self._mk('tight', 2, Marker.SPHERE, 0.06, self.CYAN)
        tgt.pose.position.x, tgt.pose.position.y, tgt.pose.position.z = target[0], target[1], 0.03
        self.pub_mk.publish(MarkerArray(markers=[path, boxes, tgt]))

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

    def run_nav2(self, g, timeout=NAV_TIMEOUT, done_within=None):
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
        watch = []                                 # (t, x, y, th): the dither watch
        while not res.done():
            self.spin_for(0.1)
            if self.pose is not None:
                if done_within is not None and self.dist(g) <= done_within:
                    gh.cancel_goal_async()          # a waypoint: near is enough
                    self.spin_for(0.3)
                    self.nav_gh = None
                    return 'reached', f'waypoint within {done_within * 100:.0f} cm'
                watch.append((time.time(),) + tuple(self.pose[:3]))
                why = self.dithering(watch, g)
                if why:
                    gh.cancel_goal_async()
                    self.spin_for(0.5)
                    self.nav_gh = None
                    self.stop()
                    return 'failed', why
            if self.stop_req or time.time() - t0 > timeout:
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

    # ── dither watch ──────────────────────────────────────────────────────────
    # Twin, apt_s1 2026-10-19: the lattice routed through the 57 cm squeeze by
    # the sofa (38 cm rover: room); at its mouth MPPI swung the heading back and
    # forth (up to 80 deg) for ~30 s, 0.1 m of progress, then nav2 re-planned
    # the long way round: 14.5 m and 123 s for a 4.9 m goal. nav2's progress
    # checker cannot see it: a 29 deg turn COUNTS as progress (it has to, for
    # rotate-to-heading). So reach watches: DITHER_S with no net DITHER_GAIN_M
    # nearer the goal, inside DITHER_BOX_M, the heading reversing DITHER_FLIPS
    # times and DITHER_RAD of turning in all (tested on the apt_s1 track: fires
    # at 26 s; not on driving, creeping, re-aiming, slaloms or detours) -> cancel, "narrow: dithering", and the tight route
    # (straight moves that fit) takes the gap instead.
    DITHER_S = 20.0
    DITHER_BOX_M = 0.50                    # stays within this of where the window began ...
    DITHER_GAIN_M = 0.15                   # ... and is no nearer the goal, start to end, than this
    DITHER_FLIPS = 2
    DITHER_RAD = 1.5
    DITHER_GOAL_M = 0.35                   # at the goal, turning to its heading is the job

    def dithering(self, watch, g):
        now = watch[-1][0]
        while watch and watch[0][0] < now - self.DITHER_S - 0.5:
            watch.pop(0)
        if now - watch[0][0] < self.DITHER_S or self.dist(g) < self.DITHER_GOAL_M:
            return ''
        x0, y0 = watch[0][1], watch[0][2]
        if max(math.hypot(x - x0, y - y0) for _, x, y, _ in watch) > self.DITHER_BOX_M:
            return ''
        d0 = math.hypot(g[0] - x0, g[1] - y0)
        if d0 - math.hypot(g[0] - watch[-1][1], g[1] - watch[-1][2]) > self.DITHER_GAIN_M:
            return ''
        flips, turned, sign = 0, 0.0, 0
        for a, b in zip(watch, watch[1:]):
            d = wrap(b[3] - a[3])
            turned += abs(d)
            if abs(d) > 0.02:
                sg = 1 if d > 0 else -1
                if sign and sg != sign:
                    flips += 1
                sign = sg
        if flips >= self.DITHER_FLIPS and turned >= self.DITHER_RAD:
            return (f'narrow: dithering ({math.degrees(turned):.0f} deg of turning, {flips} reversals, '
                    f'no nearer the goal in {self.DITHER_S:.0f} s)')
        return ''

    def snap_note(self, snapped):
        """{'snapped_cm': n, 'taken': ...} when the goal had to move off something:
        the brain says "that spot is taken; I stopped next to it" instead of
        pretending it stands where it was asked to."""
        if not snapped or self.pose is None:
            return {}
        a = snapped[0]
        d = math.hypot(a[0] - self.pose[0], a[1] - self.pose[1])
        if d <= ARRIVED_M:
            return {}
        return {'snapped_cm': round(d * 100),
                'taken': 'the spot asked for is occupied; stopped at the nearest place that fits'}

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

    ESCAPE_CLEAR = 0.06                    # m: done when this much is open (the planner pads 4 cm)
    ESCAPE_MOVES = 3

    def pivots(self):
        """Where this rover turns about, per direction (goal_exec learns it)."""
        try:
            P = json.loads(PIVOT_FILE.read_text())
            return {+1: np.array(P['1'], float), -1: np.array(P['-1'], float)}
        except (OSError, ValueError, KeyError):
            return {+1: np.zeros(2), -1: np.zeros(2)}

    def escape(self, attempt):
        """Short moves off what the rover is parked against -- straight OR a turn
        on the spot (escape_moves.py has why: a thing beside the body is not
        cleared by any straight move) -- until ESCAPE_CLEAR is open or nothing
        helps. True if it moved."""
        moved_any = False
        P = self.pivots()
        for k in range(self.ESCAPE_MOVES):
            pts, cells = self.near_now(self.scans)
            g0 = self.gap_all(pts, cells)
            if g0 > (self.ESCAPE_NEAR if k == 0 else self.ESCAPE_CLEAR):
                break

            def gap_of(dx=0.0, a=0.0, P=(0.0, 0.0)):
                return min(EM.body_gap(EM.moved(pts, dx, a, P)),
                           EM.body_gap(EM.moved(cells, dx, a, P)) - self.CELL_SLACK)
            best = EM.best_move(gap_of, g0, P)
            if best is None:
                self.say(attempt=attempt, phase='escape', gap_cm=round(g0 * 100, 1),
                         outcome='no short move or turn opens space')
                break
            kind, v, end = best
            self.say(attempt=attempt, phase='escape', gap_cm=round(g0 * 100, 1), to_gap_cm=round(end * 100, 1),
                     **({'move_cm': round(v * 100)} if kind == 'drive' else {'turn_deg': round(math.degrees(v))}))
            r, why = self.drive_straight(v, g0) if kind == 'drive' else self.turn_in_place(v, g0)
            self.say(attempt=attempt, phase='escape', outcome=r, why=why)
            if r != 'reached':
                break
            moved_any = True
        return moved_any

    ESCAPE_WZ = 1.0                        # rad/s commanded = goal_exec WZ_MIN (~0.3 rad/s actual)

    def turn_in_place(self, a, g0):
        """Turn a (rad) on the spot, closed on fresh points every tick; stops the
        moment the gap to anything shrinks (the slide is real, not modelled here)."""
        th0 = self.pose[2]
        sgn = 1.0 if a > 0 else -1.0
        t0 = time.time()
        try:
            while time.time() - t0 < abs(a) / 0.15 + 3.0:
                if self.stop_req:
                    return 'cancelled', ''
                done = abs(wrap(self.pose[2] - th0))
                if done >= abs(a) - math.radians(2):
                    return 'reached', f'turned {math.degrees(done):.0f} deg'
                pts, cells = self.near_now(self.scans[-1:])
                if self.gap_all(pts, cells) < g0 - 0.005:
                    return 'failed', f'gap closing after {math.degrees(done):.0f} deg -- stopped'
                cmd = Twist()
                cmd.angular.z = sgn * self.ESCAPE_WZ
                self.cmd.publish(cmd)
                self.spin_for(0.05)
            return 'failed', 'timed out'
        finally:
            self.stop()

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

    def planner_says_no(self, g):
        """True only when the planner ANSWERED and found no route to g. No server,
        no answer in time (a busy CPU) or a refused request is not a no: then nav2
        is asked as before, so a slow planner never turns into a false refusal."""
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
        return r.done() and len(r.result().result.path.poses) == 0

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

    def way_back(self, attempt, g, deadline):
        """Retrace the trail toward g (trail.py) until deadline (time.time()).
        Returns ('reached' | 'none' | 'blocked' | 'cancelled', why) -- why is
        the blocked hop's failure."""
        hops = self.trail.way_back(self.pose, g)
        if not hops:
            self.say(attempt=attempt, phase='way_back', outcome='none', why='no trail leads nearer the goal')
            return 'none', ''
        end = hops[-1]
        self.say(attempt=attempt, phase='way_back', hops=len(hops),
                 to=[round(end[0], 2), round(end[1], 2)],
                 gain_m=round(self.dist(g) - math.hypot(g[0] - end[0], g[1] - end[1]), 2))
        for k, h in enumerate(hops, 1):
            for look in (True, False):
                if self.stop_req:
                    return 'cancelled', ''
                if time.time() > deadline:
                    return 'blocked', 'out of time for this goal'
                if look:
                    # face the hop and hold still: the camera re-sees the way
                    self.dobs.set_active(True)
                    self.face(math.atan2(h[1] - self.pose[1], h[0] - self.pose[0]))
                    self.spin_for(WAY_BACK_LOOK_S)
                    self.dobs.set_active(False)
                hop, _, _ = self.fit(h)
                r, why = self.run_nav2(hop, timeout=WAY_BACK_HOP_S)
                if r == 'reached' or self.stop_req:
                    break
                self.say(attempt=attempt, phase='way_back', hop=k, outcome=r, why=why,
                         retry='after a look' if look else 'no')
                if not look:
                    return 'blocked', why
                # blocked after the look: one more try, then give the way back up
            if self.stop_req:
                return 'cancelled', ''
        self.say(attempt=attempt, phase='way_back', outcome='reached', dist_m=round(self.dist(g), 2))
        return 'reached', ''

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

    def house_route(self, g):
        """(route [(x, y) odom], length m) through the house map to g (odom), or (None, why).
        house_route.py has why: legs on the straight line ran into walls and furniture."""
        # OFF unless REACH_HOUSE_ROUTE=1: not yet proven on the twin (apt_s1 2026-10-19: both
        # cross-room goals still failed with it on). Off = the straight-line legs as before.
        if os.environ.get('REACH_HOUSE_ROUTE', '0') != '1':
            return None, 'house routing off'
        m = self.house
        if m is None or self.pose is None:
            return None, 'no house map'
        try:
            tr = self.buf.lookup_transform(m.header.frame_id or 'map', 'odom', Time()).transform
        except Exception:  # noqa: BLE001 - no map -> odom yet
            return None, 'no map -> odom transform'
        th = yaw_of(tr.rotation)
        c, s_ = math.cos(th), math.sin(th)
        tx, ty = tr.translation.x, tr.translation.y
        to_map = lambda x, y: (tx + c * x - s_ * y, ty + s_ * x + c * y)
        to_odom = lambda x, y: (c * (x - tx) + s_ * (y - ty), -s_ * (x - tx) + c * (y - ty))
        data = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        pts, L = HR.route(data, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y,
                          to_map(*self.pose[:2]), to_map(*g[:2]))
        if pts is None:
            return None, L
        return [to_odom(*p) for p in pts], L

    def leg_point(self, g):
        """The farthest point along the HOUSE ROUTE to g, at most LEG_MAX along it, that
        fits the rover and the planner can reach; the straight line when there is no
        route. None if neither has one."""
        route, L = self.house_route(g)
        if route is not None:
            r = min(LEG_MAX, L)
            while r >= LEG_MIN:
                (x, y), th = HR.along(route, r)
                cand, _, gap = self.fit((x, y, th))
                if not (gap == gap and gap < self.SNAP_GAP) and self.plan_ok(cand):
                    return cand
                r -= LEG_STEP
        bearing = math.atan2(g[1] - self.pose[1], g[0] - self.pose[0])
        ux, uy = math.cos(bearing), math.sin(bearing)
        r = min(LEG_MAX, self.dist(g))
        while r >= LEG_MIN:
            cand, _, gap = self.fit((self.pose[0] + r * ux, self.pose[1] + r * uy, bearing))
            if not (gap == gap and gap < self.SNAP_GAP) and self.plan_ok(cand):
                return cand
            r -= LEG_STEP
        return None

    def legs_toward(self, g, attempt):
        """Drive legs until g is within LEG_MAX. (True, '') or (False, why)."""
        for k in range(MAX_LEGS):
            d0 = self.dist(g)
            route, L = self.house_route(g)
            # near in a straight line but far by the route (behind a wall): legs too
            far = L if route is not None else d0
            if far <= LEG_MAX or self.stop_req:
                return True, ''
            leg = self.leg_point(g)
            if leg is None:
                return False, 'no reachable point toward the goal (legs)'
            self.say(attempt=attempt, phase='leg', leg=k + 1, dist_m=round(d0, 2),
                     leg_m=round(math.hypot(leg[0] - self.pose[0], leg[1] - self.pose[1]), 2))
            r, why = self.run_nav2(leg, done_within=LEG_DONE_M)
            if self.stop_req:
                return False, 'cancelled'
            route2, L2 = self.house_route(g)
            gained = (far - L2) if route is not None and route2 is not None else d0 - self.dist(g)
            if gained < 0.3:
                return False, f'leg made no progress ({r}: {why})'
        return self.dist(g) <= LEG_MAX, f'still {self.dist(g):.1f} m away after {MAX_LEGS} legs'

    def face(self, bearing):
        """Turn in place toward bearing (odom), outline-checked; for the look."""
        GP.look_to(self, bearing)

    # ── tight route: no route on the map -> turns and short drives that fit ──
    # tight_route.py has the why (2026-10-10, the bedroom door). Two point
    # sets: ALL (LiDAR + the fused camera map, what goal_exec decides on) and
    # the LiDAR alone. A plan on ALL is driven as planned. When ALL has none,
    # a plan on the LiDAR alone may still be right: the camera map holds
    # glossy-floor specks that go away once the camera looks at them (the
    # door gap's one 7 cm speck did, after the first turn). Such a plan only
    # picks the DIRECTION: each move still goes through goal_exec, which
    # refuses one that touches any camera point, so a real shoe still stops
    # it; drives are cut to TIGHT_LIDAR_LEG, and every move is followed by a
    # look and a new plan.
    def tight_target(self, g):
        """(x, y) odom: the goal if within reach, else the farthest point
        toward it that fits; None if none does."""
        d = self.dist(g)
        if d <= TIGHT_REACH[0]:
            return g[0], g[1]
        bearing = math.atan2(g[1] - self.pose[1], g[0] - self.pose[0])
        ux, uy = math.cos(bearing), math.sin(bearing)
        for r in TIGHT_REACH:
            cand, _, gap = self.fit((self.pose[0] + r * ux, self.pose[1] + r * uy, bearing))
            if not (gap == gap and gap < self.SNAP_GAP):
                return cand[0], cand[1]
        return None

    def tight_round(self, attempt, g, deadline):
        """Up to TIGHT_MOVES moves toward the tight target, until deadline
        (time.time(), the goal's MAX_S). Returns 'reached' (at the target: try
        the goal again), 'none' (no plan from the start), 'failed' (moved,
        then stuck, or out of time) or 'cancelled'."""
        try:
            ge = GoalExec(P_prior=json.loads(PIVOT_FILE.read_text()))
        except (OSError, ValueError):
            ge = GoalExec()                      # unlearned: plans with more margin
        self.dobs.set_active(True)
        moves, refused = 0, 0
        while moves < TIGHT_MOVES:
            self.spin_for(TIGHT_SETTLE_S)
            if self.stop_req:
                return 'cancelled'
            if time.time() > deadline:
                self.say(attempt=attempt, phase='tight', outcome='failed', why='out of time for this goal')
                return 'none' if not moves else 'failed'
            tgt = self.tight_target(g)
            if tgt is None:
                self.say(attempt=attempt, phase='tight', outcome='none', why='no point toward the goal fits')
                return 'none' if not moves else 'failed'
            if math.hypot(tgt[0] - self.pose[0], tgt[1] - self.pose[1]) <= TRT.GOAL_TOL:
                self.say(attempt=attempt, phase='tight', outcome='reached', moves=moves,
                         dist_m=round(self.dist(g), 2))
                return 'reached'
            c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
            dx, dy = tgt[0] - self.pose[0], tgt[1] - self.pose[1]
            tb = (c * dx + s * dy, -s * dx + c * dy)
            lidar = self.outside_body(np.vstack(self.scans)) if self.scans else np.zeros((0, 2))
            camera = self.outside_body(self.dobs.points_base(self.pose))
            every = np.vstack([lidar, camera]) if len(camera) else lidar
            steps, info = TRT.plan(every, tb, ge, budget_s=3.0)
            mode = 'all'
            if steps is None:
                steps, info = TRT.plan(lidar, tb, ge, budget_s=3.0)
                mode = 'lidar'
            self.say(attempt=attempt, phase='tight', mode=mode if steps else 'none',
                     plan=TRT.describe(steps) if steps else '', ms=info.get('ms'),
                     expanded=info.get('expanded'), why=info.get('why', ''),
                     target=[round(tgt[0], 2), round(tgt[1], 2)])
            self.show_tight(steps, mode, tgt, ge)
            if not steps:
                return 'none' if not moves else 'failed'
            kind, v = steps[0]
            # the camera map just switched on may not have arrived yet: then
            # ALL was the LiDAR alone, and is driven like a LiDAR plan
            if mode == 'all' and self.dobs.age() > 3.0:
                mode = 'lidar'
            if mode == 'lidar' and kind == 'drive':
                v = math.copysign(min(abs(v), TIGHT_LIDAR_LEG), v)
            if kind == 'turn':
                r, why = self.run_goal_exec_turn(wrap(self.pose[2] + v))
            else:
                p = self.pose
                r, why = self.run_goal_exec((p[0] + v * math.cos(p[2]), p[1] + v * math.sin(p[2]), p[2]))
            moves += 1
            self.say(attempt=attempt, phase='tight', move=TRT.describe([(kind, v)]), outcome=r, why=why)
            if r == 'cancelled' or self.stop_req:
                return 'cancelled'
            if r != 'reached':
                refused += 1
                if refused >= TIGHT_REFUSALS:
                    return 'failed'
                # refused (on a camera point, or what the move brought into
                # view): look -- the settle at the top of the loop -- and plan
                # again from what it sees now
        return 'failed'

    # ── one goal, until reached ─────────────────────────────────────────────
    def pursue(self, m):
        # Depth is subscribed only while RECOVERING (escape / look / pass use
        # it; nav2 and goal_exec do not): ~30% of a core for the whole drive
        # before (2026-09-28, load 9 on 6 cores, MPPI starved to 8.7 Hz).
        self.pursuing = True
        try:
            self._pursue(m)
        finally:
            self.pursuing = False
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
        g_asked = g if moved == 0 else self.goal_odom(m) or g
        snapped = [g_asked]                     # [the goal as asked]; snap_note measures from it
        if moved > 0:
            self.say(phase='snap', moved_cm=round(moved * 100), gap_cm=round(gap * 100, 1),
                     why='the goal as clicked touches something; nearest pose that fits')
        snap = (g[0] - (self.goal_odom(m) or g)[0], g[1] - (self.goal_odom(m) or g)[1])
        tried = []
        # A near goal whose docking line was refused goes to nav2 next: the
        # same drive test had a goal 0.76 m behind in open floor refused 7
        # times on its line (squeeze, wait, same line) and never tried nav2.
        go_nav2 = False
        prev_fail, same = None, 1   # last failed attempt (why, (x, y)); how many alike in a row
        way_backs, way_note = 0, ''
        tight_rounds = 0
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
            legs_failed = ''
            if d > LEG_MAX:
                ok, legs_failed = self.legs_toward(g, attempt)
                d = self.dist(g)
            if legs_failed:
                r, why = 'failed', legs_failed
            elif (d > EXACT_RANGE or go_nav2) and self.planner_says_no(g):
                # Ask the planner first (~1-2 s). Without it a goal with no
                # route cost a whole nav2 run -- its behaviour tree replans
                # and backs up for ~33 s before saying "no path" (real robot
                # 2026-10-18: 4 such runs, 250 s, for one occupied goal).
                r, why = 'failed', 'no path: planner found none (asked before driving)'
            elif d > EXACT_RANGE or go_nav2:
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
                         secs=round(time.time() - t0), tried=tried, **self.snap_note(snapped),
                         note=f'within nav2 tolerance; the exact finish did not fit ({why})')
                return
            # AT the spot, only the heading left: that is arrival. Twin 2026-10-19
            # (ledge_probe, the real robot's shelf-ledge goal): 2-5 cm from the goal
            # from 121 s on, the ledge in the way of the last turn; nav2 "no motion
            # that fits" three times and the goal ended FAILED at 267 s -- while
            # standing where it was sent. "Go near X" is about the place.
            if r == 'failed' and self.dist(g) <= ARRIVED_M:
                self.stop()
                off = abs(math.degrees(wrap(g[2] - self.pose[2])))
                self.say(result='reached', attempt=attempt, dist_cm=round(self.dist(g) * 100, 1),
                         secs=round(time.time() - t0), tried=tried, **self.snap_note(snapped),
                         note=f'at the spot; could not turn to the goal heading ({off:.0f} deg off): {why}')
                return
            if r == 'reached':
                self.say(result='reached', attempt=attempt, dist_cm=round(self.dist(g) * 100, 1),
                         secs=round(time.time() - t0), tried=tried, **self.snap_note(snapped))
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
            # The same failure from the same spot, recovered from and back
            # again: another round is the same move again. 2026-10-04: four
            # "no path" from one spot behind a cot, 316 s, before reach gave up.
            same = same + 1 if TR.same_failure(prev_fail, why, self.pose[:2]) else 1
            if same >= SAME_FAILS:
                self.stop()
                self.say(result='failed', attempt=attempt,
                         why=f'no way from here: {why} -- {same} times from the same spot',
                         secs=round(time.time() - t0), tried=tried,
                         **({'note': way_note} if way_note else {}))
                return
            prev_fail = (why, self.pose[:2])
            # The goal may have turned out to be occupied once seen: fit() ran
            # when the goal came in, on the map of THAT moment, and unseen floor
            # is left alone. Real robot 2026-10-18: a goal on a shelf ledge the
            # camera had not seen yet -- 4 x "no path", 250 s, failed. Fit again
            # on what the map shows now.
            g2, moved2, gap2 = self.fit(g)
            if moved2 > 0:
                g = g2
                if m.header.frame_id == 'map':
                    gm = self.goal_odom(m)
                    snap = (g[0] - gm[0], g[1] - gm[1]) if gm else snap
                same = 1
                self.say(attempt=attempt, phase='snap', moved_cm=round(moved2 * 100), gap_cm=round(gap2 * 100, 1),
                         why='the goal touches something now that it has been seen; nearest pose that fits')
                continue
            if gap2 == gap2 and gap2 < 0:           # not NaN: inside something, and nothing fits near it
                self.stop()
                self.say(result='failed', attempt=attempt, secs=round(time.time() - t0), tried=tried,
                         why=f'the goal is inside something (now seen: {round(-gap2 * 100)} cm into it) '
                             f'and no spot within {round(self.SNAP_R * 100)} cm fits the rover')
                return
            self.say(attempt=attempt, phase='recover', why=why)
            # ── recover: stop, clear, look, then pass if narrow, else wait ──
            self.stop()
            self.dobs.set_active(True)              # recovering: fresh depth first
            self.clear_costmaps()                   # (spins 0.5 s)
            self.spin_for(1.0)
            # parked against something (MPPI may go to the 1 cm line; the
            # planner will not start from it): step off first. Then retry at
            # once -- unless nav2 could not fit or found no route: then retrying
            # it from 5 cm back is the same move again (2026-10-03: escape +
            # nav2 five times at the gap mouth, touching the chair leg each
            # time). Those go on to look and a straight pass.
            jammed = why.startswith('narrow') or 'stuck' in why or 'no path' in why
            if self.escape(attempt) and not jammed:
                continue
            # no route on the map, or nav2 cannot fit: turns and short drives
            # on the raw points (tight_round). Before the way back: on
            # 2026-10-10 the way back ran twice, nav2 failed after each, and
            # the goal ended "3 times from the same spot" without one move
            # through the door the rover had just driven out of.
            if jammed and tight_rounds < TIGHT_ROUNDS:
                tight_rounds += 1
                out = self.tight_round(attempt, g, t0 + MAX_S)
                self.show_tight(None, '', None, None)
                if out == 'reached':
                    continue
                if out == 'cancelled' or self.stop_req:
                    break
            # no route to the goal: look both ways and try a straight pass from
            # HERE first; only if no line fits, go as near as a route goes
            looked = False
            if 'no path' in why and way_backs < WAY_BACKS:
                way_backs += 1
                out, hop_why = self.way_back(attempt, g, t0 + MAX_S)
                if out == 'reached':
                    continue
                if out == 'cancelled' or self.stop_req:
                    break
                if out == 'blocked':
                    way_note = TR.way_back_note(hop_why)
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
                 why=f'gave up after {len(tried)} attempts, {time.time() - t0:.0f} s', tried=tried,
                 **({'note': way_note} if way_note and not self.stop_req else {}))


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
