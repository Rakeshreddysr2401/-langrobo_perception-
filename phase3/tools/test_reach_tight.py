#!/usr/bin/env python3
"""test_reach_tight.py — reach's tight_round() driven against a simulated room.

    python3 phase3/tools/test_reach_tight.py   (in the rover container: needs rclpy for the import; no robot)

The 2026-10-10 bedroom door again (fixture_door_inside): the LiDAR's points
and the camera map's, with the one glossy-floor speck in the gap. The rover
moves exactly as asked; "goal_exec" refuses a move its own checks refuse, on
whatever the camera map holds at that moment.

  real speck      it never goes away: the LiDAR plan heads for the gap,
                  goal_exec refuses at the speck, twice -> 'failed', and the
                  rover never touched it
  phantom speck   it goes once the camera has faced it (nvblox clears what it
                  sees past): out of the door -> 'reached'
  deadline        no time left: 'none', no move
  cancel          a stop request mid-round: 'cancelled'
"""
import math
import sys
import time
from pathlib import Path

import numpy as np
from rclpy.clock import Clock

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'nodes'))
sys.path.insert(0, str(HERE))
import goal_exec as G  # noqa: E402
import reach_node as RN  # noqa: E402

PIVOT = {"1": [-0.012, 0.021], "-1": [0.0114, -0.0304], "n": {"1": 3, "-1": 3}}
FIX = np.load(HERE / 'fixture_door_inside_2026-10-10.npy').astype(float)
LIDAR, CAMERA = FIX[:653], FIX[653:]
SPECK = (0.53, 0.27)


class Cams:
    """The camera map: world points; age() fresh."""
    def __init__(self, pts):
        self.pts = pts

    def set_active(self, on):
        pass

    def age(self):
        return 0.5

    def points_base(self, pose):
        return to_base(self.pts, pose)


def to_base(w, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    q = w - pose[:2]
    return np.stack([c * q[:, 0] + s * q[:, 1], -s * q[:, 0] + c * q[:, 1]], 1)


class Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class FakeReach:
    """Just what tight_round touches, as Reach has it -- the drawing too."""
    SNAP_GAP = RN.Reach.SNAP_GAP
    WHITE, GREEN, AMBER, RED, CYAN = RN.Reach.WHITE, RN.Reach.GREEN, RN.Reach.AMBER, RN.Reach.RED, RN.Reach.CYAN
    outside_body = staticmethod(RN.Reach.outside_body)
    status_text = staticmethod(RN.Reach.status_text)
    tight_round = RN.Reach.tight_round
    tight_target = RN.Reach.tight_target
    on_plan = RN.Reach.on_plan
    TIGHT_FOLLOW_M, TIGHT_FOLLOW_DEG = RN.Reach.TIGHT_FOLLOW_M, RN.Reach.TIGHT_FOLLOW_DEG

    def house_route(self, g):
        return None, "no house map in this test"

    class sonar:                       # no sonars in this test (as with an old URDF)
        used = 0

        @staticmethod
        def points(known):
            return np.zeros((0, 2))

        @staticmethod
        def vouch(camera, lidar):
            return np.zeros(len(camera), bool)

    show_tight = RN.Reach.show_tight
    _mk = RN.Reach._mk

    def __init__(self, phantom, cancel_after=None):
        self.pose = np.zeros(3)
        self.dobs = Cams(CAMERA.copy())
        self.phantom = phantom
        self.stop_req = False
        self.cancel_after = cancel_after
        self.moves, self.log = 0, []
        self.pub_mk = Pub()
        self.ge = G.GoalExec(P_prior=PIVOT)
        RN.PIVOT_FILE = Path('/nonexistent')       # GoalExec() unlearned -> patched below
        RN.GoalExec = lambda P_prior=None: G.GoalExec(P_prior=PIVOT)
        RN.TIGHT_SETTLE_S = 0.0
        RN.TIGHT_FOLLOW_SETTLE_S = 0.0

    @property
    def scans(self):
        return [to_base(LIDAR, self.pose)]

    def spin_for(self, s):
        pass

    def get_clock(self):
        return Clock()

    def dist(self, g):
        return math.hypot(g[0] - self.pose[0], g[1] - self.pose[1])

    def fit(self, g):
        return g, 0.0, float('nan')

    def say(self, **d):
        self.log.append(d)

    def _all(self):
        return np.vstack([to_base(LIDAR, self.pose), self.dobs.points_base(self.pose)])

    def _look(self):
        # nvblox clears a phantom once the camera faces it (within ~40 deg, 0.3-3 m)
        if not self.phantom:
            return
        b = self.dobs.points_base(self.pose)
        rng = np.hypot(b[:, 0], b[:, 1])
        ang = np.abs(np.degrees(np.arctan2(b[:, 1], b[:, 0])))
        near_speck = np.hypot(self.dobs.pts[:, 0] - SPECK[0], self.dobs.pts[:, 1] - SPECK[1]) < 0.05
        self.dobs.pts = self.dobs.pts[~(near_speck & (ang < 40) & (rng > 0.3))]

    def _count(self):
        self.moves += 1
        if self.cancel_after and self.moves >= self.cancel_after:
            self.stop_req = True

    def run_goal_exec_turn(self, heading):
        a = G.wrap(heading - self.pose[2])
        ok, why = self.ge.turn_clear(self._all(), a)
        self._count()
        if not ok:
            return 'failed', 'goal_exec refused: ' + why
        sl = G.R(self.pose[2]) @ ((np.eye(2) - G.R(a)) @ self.ge.P[1 if a > 0 else -1])
        self.pose = self.pose + [sl[0], sl[1], a]
        self._look()
        return 'reached', ''

    def run_goal_exec(self, g):
        d = math.hypot(g[0] - self.pose[0], g[1] - self.pose[1])
        d = d if math.cos(math.atan2(g[1] - self.pose[1], g[0] - self.pose[0]) - self.pose[2]) > 0 else -d
        ok, why = self.ge.drive_clear(self._all(), d)
        self._count()
        if not ok:
            return 'failed', 'goal_exec refused: ' + why
        self.pose = np.array([g[0], g[1], self.pose[2]])
        self._look()
        return 'reached', ''


GOAL = (1.3, 1.2, math.radians(45))


def test_phantom_speck_goes_out_of_the_door():
    r = FakeReach(phantom=True)
    out = r.tight_round(1, GOAL, time.time() + 60)
    assert out == 'reached', (out, r.log[-3:])
    assert r.dist(GOAL) <= 0.13, r.pose
    plans = [d['plan'] for d in r.log if d.get('plan')]
    print(f'  phantom: reached in {r.moves} moves; plans {plans[:3]} ...')
    # drawn: one plan per planning, path from the rover's pose, an outline
    # (4 edges) per step end, amber while the plan is the LiDAR's alone
    drawn = [m for m in r.pub_mk.sent if m.markers and m.markers[0].action == m.markers[0].ADD]
    assert len(drawn) == len(plans), (len(drawn), len(plans))
    path, boxes, tgt = drawn[0].markers
    assert len(path.points) == len(plans[0].split()) + 1, (len(path.points), plans[0])
    assert abs(path.points[0].x) < 1e-6 and abs(path.points[0].y) < 1e-6
    assert len(boxes.points) == 8 * (len(path.points) - 1)
    assert (path.color.r, path.color.g) == RN.Reach.AMBER[:2], path.color      # LiDAR-only plan
    assert abs(tgt.pose.position.x - GOAL[0]) < 1e-6


def test_real_speck_stops_before_it():
    r = FakeReach(phantom=False)
    out = r.tight_round(1, GOAL, time.time() + 60)
    assert out == 'failed', (out, r.log[-3:])
    # never inside the speck: goal_exec's own 1 cm check refused before it
    b = to_base(np.array([SPECK]), r.pose)[0]
    assert not ((b[0] < G.FRONT) and (b[0] > -G.REAR) and abs(b[1]) < G.SIDE), r.pose
    refusals = [d for d in r.log if d.get('outcome') == 'failed']
    print(f'  real speck: stopped after {r.moves} moves, {len(refusals)} refusals, '
          f'at ({r.pose[0]:.2f}, {r.pose[1]:.2f})')


def test_status_line():
    t = RN.Reach.status_text
    assert t({'attempt': 2, 'phase': 'tight', 'plan': 'F10 L45 F160', 'mode': 'lidar'}) == \
        'try 2: tight -- F10 L45 F160 (lidar)'
    assert t({'attempt': 2, 'phase': 'tight', 'move': 'L45', 'outcome': 'reached'}) == 'try 2: tight -- L45 reached'
    assert t({'result': 'reached', 'dist_cm': 1.2}) == 'reached 1.2 cm'
    long = t({'result': 'failed', 'why': 'no way from here: ' + 'x' * 200})
    assert long.startswith('failed -- no way') and len(long) <= 72


def test_no_time_no_move():
    r = FakeReach(phantom=True)
    out = r.tight_round(1, GOAL, time.time() - 1)
    assert out == 'none' and r.moves == 0, (out, r.moves)


def test_cancel_mid_round():
    r = FakeReach(phantom=True, cancel_after=2)
    out = r.tight_round(1, GOAL, time.time() + 60)
    assert out == 'cancelled' and r.moves == 2, (out, r.moves)


if __name__ == '__main__':
    fails = 0
    for name, fn in list(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print(f'PASS {name}')
            except AssertionError as e:
                fails += 1
                print(f'FAIL {name}: {e}')
    print('all passed' if not fails else f'{fails} failed')
    sys.exit(1 if fails else 0)
