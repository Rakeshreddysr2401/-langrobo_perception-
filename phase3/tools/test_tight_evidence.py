#!/usr/bin/env python3
"""test_tight_evidence.py — the camera's points are checked before a LiDAR plan is driven.

    python3 phase3/tools/test_tight_evidence.py   (needs rclpy for the reach import; no robot)

The real robot, 2026-10-20 (reach log 14:53-16:11): when no plan fitted ALL
points, the LiDAR's plan was driven and goal_exec refused its first move 7 of
15 times on a camera point (a chair's splayed foot, a stool seat); twice the
same plan was made again and refused again. Here, a wall with a 1 m opening:

  STOOL   a low box (camera only, under the LiDAR's plane) in the opening:
          y -0.24 .. 0.00 -- real, it never goes
  SPECK   a glossy-floor speck in the 50 cm way left of it -- gone once seen
          from another angle
  right   the way right of the stool is 26 cm: too narrow

ALL points: no way (stool + speck). LiDAR alone: straight through the middle.
Expected: no move into a refusal; the speck looked at and gone, through on the
left.  And with the speck real: an honest 'none' / 'failed' with NO refused
move and no plan made twice.
"""
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'nodes'))
sys.path.insert(0, str(HERE))
import goal_exec as G  # noqa: E402
import test_reach_tight as T  # noqa: E402
import tight_route as TRT  # noqa: E402


def line(x0, y0, x1, y1, step=0.02):
    n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step) + 1)
    return np.stack([np.linspace(x0, x1, n), np.linspace(y0, y1, n)], 1)


def box(cx, cy, sx, sy):
    return np.vstack([line(cx - sx / 2, cy - sy / 2, cx + sx / 2, cy - sy / 2), line(cx - sx / 2, cy + sy / 2, cx + sx / 2, cy + sy / 2),
                      line(cx - sx / 2, cy - sy / 2, cx - sx / 2, cy + sy / 2), line(cx + sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2)])


WALL_X = 1.0
# opening y -0.50 .. +0.50; the LiDAR sees the wall either side
LIDAR = np.vstack([line(WALL_X, -1.6, WALL_X, -0.50), line(WALL_X, 0.50, WALL_X, 1.6),
                   line(-0.6, -1.6, 2.6, -1.6), line(-0.6, 1.6, 2.6, 1.6), line(-0.6, -1.6, -0.6, 1.6), line(2.6, -1.6, 2.6, 1.6)])
STOOL = box(WALL_X, -0.12, 0.30, 0.24)        # y -0.24 .. 0.00: left way 0.00 .. 0.50 = 50 cm, right 26 cm
SPECK = (WALL_X - 0.05, 0.25)
GOAL = (2.0, 0.25, 0.0)


class Room(T.FakeReach):
    def __init__(self, phantom):
        super().__init__(phantom=phantom)
        self.dobs = T.Cams(np.vstack([STOOL, box(*SPECK, 0.05, 0.05)]))

    @property
    def scans(self):
        return [T.to_base(LIDAR, self.pose)]

    def _all(self):
        return np.vstack([T.to_base(LIDAR, self.pose), self.dobs.points_base(self.pose)])

    def _look(self):
        # glossy speck: gone once the camera sees it from a second pose
        if not self.phantom:
            return
        b = self.dobs.points_base(self.pose)
        near = np.hypot(self.dobs.pts[:, 0] - SPECK[0], self.dobs.pts[:, 1] - SPECK[1]) < 0.06
        if (near & TRT.in_view(b)).any():          # (_look runs after a move: a new viewpoint)
            self.dobs.pts = self.dobs.pts[~near]


def run(phantom):
    r = Room(phantom)
    out = r.tight_round(1, GOAL, time.time() + 120)
    refused = [d for d in r.log if d.get('outcome') == 'failed']
    plans = [d['plan'] for d in r.log if d.get('plan')]
    return r, out, refused, plans


def test_speck_and_stool_through_on_the_left():
    r, out, refused, plans = run(phantom=True)
    print(f'  phantom speck: {out} in {r.moves} moves, {len(refused)} refused; plans {plans}')
    assert out == 'reached', (out, r.log[-4:])
    assert not refused, refused
    assert r.moves <= 8, r.moves


def test_real_speck_honest_no_way_without_refusals():
    r, out, refused, plans = run(phantom=False)
    print(f'  real speck: {out} after {r.moves} moves, {len(refused)} refused, at ({r.pose[0]:.2f}, {r.pose[1]:.2f}); '
          f'plans {plans}')
    assert out in ('none', 'failed'), out
    assert not refused, refused
    for pts in (STOOL, np.array([SPECK])):          # never inside the stool or the speck
        b = T.to_base(pts, r.pose)
        assert not ((b[:, 0] < G.FRONT) & (b[:, 0] > -G.REAR) & (np.abs(b[:, 1]) < G.SIDE)).any()


def test_evidence_rules():
    ev = TRT.Evidence()
    ge = G.GoalExec(P_prior=T.PIVOT)
    p0 = (0.0, 0.0, 0.0)
    ahead = np.array([[0.8, 0.0]])
    beside = np.array([[0.0, 0.25]])
    assert ev.check(beside, p0) == (1, 0)                      # out of view: avoided now ...
    assert len(ev.unseen) == 1 and len(ev.confirmed) == 0      # ... but not believed for the goal
    ev.moved()
    assert len(ev.unseen) == 0                                 # re-read after a move
    assert ev.check(ahead, p0) == (0, 1)                       # in view: a suspect
    assert ev.check(ahead, p0) == (0, 1) and len(ev.sus) == 1  # same viewpoint: still a suspect, not doubled
    p1 = (0.0, 0.0, math.radians(15))
    b1 = TRT.to_base(TRT.to_odom(ahead, p0), p1)
    assert ev.check(b1, p1) == (1, 0)                          # seen again from 15 deg: real
    assert len(ev.confirmed) == 1
    # first_blocked: a drive into a point, the turn before it fine
    k, hits = TRT.first_blocked([('turn', math.radians(15)), ('drive', 1.0)], np.array([[0.7, 0.18]]), ge)
    assert k == 1 and len(hits) == 1, (k, hits)
    # move_hits agrees with goal_exec's own checks
    rng = np.random.default_rng(1)
    pts = rng.uniform(-0.8, 0.8, (400, 2))
    pts = pts[~((pts[:, 0] < G.FRONT + 0.01) & (pts[:, 0] > -G.REAR - 0.01) & (np.abs(pts[:, 1]) < G.SIDE + 0.01))]
    for kind, v in (('drive', 0.4), ('drive', -0.1), ('turn', 0.5), ('turn', -1.2)):
        for i in range(0, len(pts), 7):
            q = pts[i:i + 1]
            ok = (ge.drive_clear(q, v) if kind == 'drive' else ge.turn_clear(q, v))[0]
            assert ok == (not TRT.move_hits(ge, q, kind, v).any()), (kind, v, q)


class Glitch(Room):
    """Twin chairs44 2026-10-20: a camera point ahead appears just as
    goal_exec checks the straight drive, refuses it, and is gone a moment later
    (no stool, no speck here: an open opening). AHEAD: m beyond the front edge;
    0.12 puts it where the camera cannot see the floor (< CAM_NEAR from base)."""
    AHEAD = 0.30

    def __init__(self):
        super().__init__(phantom=True)
        self.dobs = T.Cams(np.zeros((0, 2)))
        self.glitched = False

    def run_goal_exec(self, g):
        if not self.glitched:
            self.glitched = True
            c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
            p = self.pose[:2] + (G.FRONT + self.AHEAD) * np.array([c, s])
            self.dobs.pts = np.array([p])
            out = super().run_goal_exec(g)
            # a refusal: the planner's reaction is measured from here; the
            # point is still in the map at the refusal, gone right after
            self._pending_clear = True
            return out
        if getattr(self, '_pending_clear', False):
            self.dobs.pts = np.zeros((0, 2))
            self._pending_clear = False
        return super().run_goal_exec(g)

    def spin_for(self, s):
        if getattr(self, '_pending_clear', False):
            self.dobs.pts = np.zeros((0, 2))
            self._pending_clear = False


class NearGlitch(Glitch):
    AHEAD = 0.12


def test_a_passing_glitch_by_the_bumper_is_not_a_wall():
    """The same glitch where the camera cannot see (12 cm off the bumper): the
    first version believed such points at once and for the whole goal --
    chairs44 then failed its re-plans (8 s 'no fit' each, 144-182 s goals)."""
    r = NearGlitch()
    out = r.tight_round(1, GOAL, time.time() + 120)
    refused = [d for d in r.log if d.get('outcome') == 'failed']
    plans = [d['plan'] for d in r.log if d.get('plan')]
    print(f'  glitch by the bumper: {out} in {r.moves} moves, {len(refused)} refused; plans {plans}')
    assert out == 'reached', (out, r.log[-4:])
    assert len(r.tight_ev.confirmed) == 0, r.tight_ev.confirmed


def test_a_passing_glitch_is_not_a_wall():
    r = Glitch()
    out = r.tight_round(1, GOAL, time.time() + 120)
    refused = [d for d in r.log if d.get('outcome') == 'failed']
    plans = [d['plan'] for d in r.log if d.get('plan')]
    print(f'  glitch: {out} in {r.moves} moves, {len(refused)} refused; plans {plans}')
    assert out == 'reached', (out, r.log[-4:])
    assert len(refused) <= 1, refused
    assert len(r.tight_ev.confirmed) == 0, r.tight_ev.confirmed      # never believed: seen from one pose only


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
