#!/usr/bin/env python3
"""test_known_floor_map.py — the reverse rule's known floor also from the camera map (2026-11-13).

    python3 nav/tests/test_known_floor_map.py      (numpy + scipy; no robot, no ROS)

Real robot 2026-11-10: reach was restarted to deploy a fix, its known-floor memory restarted
empty, and four rl2 rounds minutes later ended pinned with every reverse refused. The fixture is
nvblox's ghost-cleared grid rebuilt from the recorded bag (rover mapping/tools/replay_nvblox.sh,
t = 327 s), 3 x 3 m round the rover, and the rover's pose there.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
import known_floor as KF  # noqa: E402
import rl_drive as RD  # noqa: E402

FIX = np.load(Path(__file__).resolve().parent / 'fixtures' / 'real_free_grid_20261110_327.npz')
PACK = (np.array([-0.0411, -0.0287]), np.array([-0.0345, 0.0099]), 0.85)      # the real rover's pivots
BACK = (-0.08, 0.0)


def reverse_allowed(k, pose):
    return not RD.sweep_hits(np.zeros((0, 2)), *BACK, (0.0, 0.0), PACK, known=k.known_base(pose))


def test_a_just_restarted_reach_may_back_out_on_the_real_map():
    pose = tuple(float(v) for v in FIX['pose'])
    k = KF.KnownFloor()
    k.mark(pose)
    assert not reverse_allowed(k, pose), 'own memory only (just restarted): reversing is refused -- the bug'
    n = k.mark_free_grid(FIX['grid'], float(FIX['res']), FIX['origin'], pose)
    assert n > 500, n
    assert reverse_allowed(k, pose), 'with the camera map\'s observed-free floor it may back out'


def test_unseen_and_occupied_never_count():
    pose = (0.0, 0.0, 0.0)
    g = np.full((80, 80), -1, np.int8)            # 2.5 cm cells, -1 m .. +1 m
    g[:, 40:] = 0                                  # only x >= 0 observed free
    g[38:42, 60] = 100                             # one occupied cell ahead
    k = KF.KnownFloor()
    k.mark_free_grid(g, 0.025, (-1.0, -1.0), pose)
    kb = k.known_base(pose)
    assert not kb(np.array([-0.3]), np.array([0.0]))[0], 'never-seen floor behind stays unknown'
    assert not kb(np.array([0.5]), np.array([0.0]))[0], 'the occupied cell (and its border) is not known floor'
    assert kb(np.array([0.3]), np.array([0.5]))[0], 'observed-free floor is known'


def test_only_near_the_rover():
    g = np.zeros((400, 400), np.int8)              # 10 x 10 m all free
    k = KF.KnownFloor()
    k.mark_free_grid(g, 0.025, (-5.0, -5.0), (0.0, 0.0, 0.0), r_max=3.0)
    kb = k.known_base((0.0, 0.0, 0.0))
    assert kb(np.array([2.5]), np.array([0.0]))[0] and not kb(np.array([4.0]), np.array([0.0]))[0]


if __name__ == '__main__':
    for f in (test_a_just_restarted_reach_may_back_out_on_the_real_map, test_unseen_and_occupied_never_count, test_only_near_the_rover):
        f()
        print('ok  ', f.__name__)
    print('ALL PASS')
