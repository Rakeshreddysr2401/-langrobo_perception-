#!/usr/bin/env python3
"""test_tight_route.py — offline checks of reach's tight route (phase3/nodes/tight_route.py).

    python3 phase3/tools/test_tight_route.py      (no robot, no ROS)

The 2026-10-10 bedroom door, replayed from the points measured there
(LiDAR + the fused camera map, base_link, as goal_exec reads them):

  fixture_door_inside   the rover 30 cm from the open door leaf, its left
                        side 5 cm from a wall corner; the way out is past
                        the leaf's free edge, 77 cm. nav2: no path. Driven
                        that day: F10 L45 F160, out of the door.
  fixture_door_hall     from the hall, after "come to me": back through the
                        same door. nav2: no path, 3 times, 188 s, no move.

Every plan is then re-checked move by move with goal_exec's OWN checks
(turn_clear / drive_clear at its 1 cm) -- the plan must never ask goal_exec
for a move it would refuse.
"""
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'nodes'))
import goal_exec as G  # noqa: E402
import tight_route as T  # noqa: E402

# the pivot goal_exec had learned that day (/logs/goal_exec_pivot.json)
PIVOT = {"1": [-0.012, 0.021], "-1": [0.0114, -0.0304], "n": {"1": 3, "-1": 3}}


def ge():
    return G.GoalExec(P_prior=PIVOT)


def replay(pts, steps, e):
    """Drive the plan through goal_exec's own checks. Returns the end pose,
    or raises with the move goal_exec would refuse."""
    st = np.zeros(3)
    for kind, v in steps:
        c, s = math.cos(st[2]), math.sin(st[2])
        q = pts - st[:2]
        p = np.stack([c * q[:, 0] + s * q[:, 1], -s * q[:, 0] + c * q[:, 1]], 1)
        if kind == 'drive':
            ok, why = e.drive_clear(p, v)
            assert ok, f'goal_exec would refuse {T.describe([(kind, v)])}: {why}'
            st = st + [v * c, v * s, 0.0]
        else:
            ok, why = e.turn_clear(p, v)
            assert ok, f'goal_exec would refuse {T.describe([(kind, v)])}: {why}'
            sl = G.R(st[2]) @ ((np.eye(2) - G.R(v)) @ e.P[1 if v > 0 else -1])
            st = st + [sl[0], sl[1], v]
    return st


def wall(x0, y0, x1, y1, step=0.01):
    n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step))
    return np.stack([np.linspace(x0, x1, n), np.linspace(y0, y1, n)], 1)


def load(name, lidar_n):
    """The fixtures are the LiDAR's points first, then the camera map's."""
    p = np.load(HERE / name)
    return p, p[:lidar_n]


def test_out_of_the_door_on_the_lidar():
    # all points: the speck in the gap closes it (it went away once looked at)
    full, lid = load('fixture_door_inside_2026-10-10.npy', 653)
    steps, info = T.plan(full, (1.3, 1.2), ge(), budget_s=2.0)
    assert steps is None, f'the speck closes the gap on ALL points, got {T.describe(steps)}'
    steps, info = T.plan(lid, (1.3, 1.2), ge())
    assert steps, info
    plan = T.describe(steps)
    first_turn = next(v for k, v in steps if k == 'turn')
    assert 30 <= math.degrees(first_turn) <= 60, plan        # left, like the floor run
    assert all(v > -T.BACK_MAX - 1e-9 for k, v in steps if k == 'drive'), plan
    end = replay(lid, steps, ge())
    assert math.hypot(end[0] - 1.3, end[1] - 1.2) <= T.GOAL_TOL + 0.01, end
    print(f'  inside -> out: {plan} ({info["ms"]} ms)')


def test_back_through_the_door_from_the_hall():
    full, lid = load('fixture_door_hall_2026-10-10.npy', 629)
    steps, info = T.plan(lid, (1.98, 0.56), ge())
    assert steps, info
    end = replay(lid, steps, ge())
    assert math.hypot(end[0] - 1.98, end[1] - 0.56) <= T.GOAL_TOL + 0.01, end
    print(f'  hall -> bedroom: {T.describe(steps)} ({info["ms"]} ms)')


def test_turn_into_a_side_doorway():
    # a corridor 70 cm wide along +x; a 60 cm doorway in its left wall at
    # x 0.9-1.5; the target is 0.8 m through it. No straight line; one turn.
    pts = np.vstack([wall(-0.5, -0.35, 2.5, -0.35),
                     wall(-0.5, 0.35, 0.9, 0.35), wall(1.5, 0.35, 2.5, 0.35),
                     wall(0.9, 0.35, 0.9, 1.6), wall(1.5, 0.35, 1.5, 1.6)])
    steps, info = T.plan(pts, (1.2, 1.15), ge())
    assert steps, info
    left = sum(v for k, v in steps if k == 'turn')
    assert math.radians(75) <= left <= math.radians(105), T.describe(steps)   # about a quarter turn, in all
    replay(pts, steps, ge())
    print(f'  side doorway: {T.describe(steps)} ({info["ms"]} ms)')


def test_a_gap_two_cm_too_narrow_is_refused():
    # 38 cm rover, 2 cm a side at planning: 42 cm passes, 40 does not. A
    # wall across the room with a 1 m deep slot in it -- the only way through
    for width, fits in ((0.44, True), (0.40, False)):
        h = width / 2
        pts = np.vstack([wall(0.5, h, 1.5, h), wall(0.5, -h, 1.5, -h),
                         wall(0.5, h, 0.5, 2.8), wall(0.5, -h, 0.5, -2.8)])
        steps, info = T.plan(pts, (2.0, 0.0), ge(), budget_s=1.5)
        assert (steps is not None) == fits, (width, T.describe(steps), info)
        if fits:
            replay(pts, steps, ge())
    print('  44 cm gap: through; 40 cm: refused')


def test_no_way_through_a_wall():
    pts = np.vstack([wall(0.6, -2.5, 0.6, 2.5)])
    steps, info = T.plan(pts, (1.5, 0.0), ge(), budget_s=1.0)
    assert steps is None
    assert info['ms'] <= 1500, info
    print(f'  wall: none, {info["why"]} ({info["ms"]} ms)')


def test_start_touching_is_reported():
    pts = wall(0.19, -0.3, 0.19, 0.3)              # 0.8 cm off the nose
    steps, info = T.plan(pts, (1.0, 0.0), ge())
    assert steps is None and 'start' in info['why'], info


def test_checks_match_goal_exec():
    # the planner's fast checks must say exactly what goal_exec's say, at
    # goal_exec's own margin: a plan goal_exec then refuses is a stuck rover
    rng = np.random.default_rng(3)
    e = ge()
    keep = T.PLAN_MARGIN
    T.PLAN_MARGIN = G.MARGIN
    try:
        bad = []
        for _ in range(2000):
            p = rng.uniform(-0.7, 0.7, (int(rng.integers(1, 40)), 2))
            a = math.radians(float(rng.choice([15, 30, 45, 90]))) * float(rng.choice([1, -1]))
            d = float(rng.choice([0.05, 0.2, 0.5, 1.0, -0.05, -0.1]))
            if T._turn_ok(e, p, a) != e.turn_clear(p, a)[0]:
                bad.append(('turn', a))
            if T._drive_ok(e, p, d) != e.drive_clear(p, d)[0]:
                bad.append(('drive', d))
        assert not bad, bad[:5]
    finally:
        T.PLAN_MARGIN = keep


def test_merge_and_describe():
    s = [('drive', 0.05), ('drive', 0.05), ('turn', math.radians(30)), ('turn', math.radians(15)),
         ('drive', 0.5), ('drive', 0.5), ('drive', 0.5), ('drive', 0.2)]
    m = T.merge(s)
    assert T.describe(m) == 'F10 L45 F150 F20', T.describe(m)


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
