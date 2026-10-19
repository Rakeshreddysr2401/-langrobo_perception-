"""python3 -m pytest phase3/nodes/test_escape_moves.py"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import escape_moves as E  # noqa: E402

PIV = {+1: np.array([0.0506, 0.0379]), -1: np.array([0.0343, -0.0154])}   # the twin's learned pivot


def rack_points(x0, y0, x1, y1, pose, step=0.01):
    """A box outline (world) -> base_link points for a rover at pose."""
    xs, ys = np.arange(x0, x1 + 1e-9, step), np.arange(y0, y1 + 1e-9, step)
    w = np.vstack([np.c_[xs, np.full_like(xs, y0)], np.c_[xs, np.full_like(xs, y1)],
                   np.c_[np.full_like(ys, x0), ys], np.c_[np.full_like(ys, x1), ys]])
    c, s = math.cos(pose[2]), math.sin(pose[2])
    d = w - np.array(pose[:2])
    return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], 1)


def gap_fn(pts):
    return lambda dx=0.0, a=0.0, P=(0.0, 0.0): E.body_gap(E.moved(pts, dx, a, P))


def test_shoe_rack_apt_s1_finds_a_way_off():
    # apt_s1 2026-10-19: shoe rack 3.09..4.01 x 1.05..1.35, rover a little off its stuck pose
    th = -1.013
    pose = (2.843 + 0.028 * math.sin(th), 1.16 - 0.028 * math.cos(th), th)
    pts = rack_points(3.09, 1.05, 4.01, 1.35, pose)
    g0 = E.body_gap(pts)
    assert 0.0 < g0 < 0.03                       # pinned
    m = E.best_move(gap_fn(pts), g0, PIV)
    assert m is not None and m[2] > g0 + 0.02, m


def test_alongside_a_long_shelf_with_wall_behind_no_short_move():
    # A shelf the length of the left side, a wall 1 cm behind. A skid-steer turning right
    # swings its REAR left (into the shelf), turning left swings its nose in: no turn, and
    # no straight move, opens the gap -- only driving on along the shelf does, which is
    # nav2's job. best_move must say so (None), not pick a move that scrapes.
    side = np.c_[np.linspace(0.0, 0.6, 61), np.full(61, 0.20)]
    back = np.c_[np.full(41, -0.188), np.linspace(-0.4, 0.4, 41)]
    pts = np.vstack([side, back])
    assert E.best_move(gap_fn(pts), E.body_gap(pts), PIV) is None


def test_wall_ahead_backs_off():
    pts = np.c_[np.full(41, 0.192), np.linspace(-0.4, 0.4, 41)]      # a wall 1 cm in front
    m = E.best_move(gap_fn(pts), E.body_gap(pts), PIV)
    assert m == ('drive', -0.05, m[2]) and m[2] > 0.05


def test_box_at_front_corner_drives_or_turns_away_never_into_it():
    pts = np.c_[np.linspace(0.19, 0.3, 12), np.full(12, 0.20)]       # beside the front-left corner
    f = gap_fn(pts)
    m = E.best_move(f, E.body_gap(pts), PIV)
    assert m is not None
    if m[0] == 'turn':
        assert m[1] < 0                          # right: away from it


def test_boxed_in_says_none():
    # a 40 x 38 cm hole: nothing fits better
    xs = np.linspace(-0.2, 0.2, 41)
    pts = np.vstack([np.c_[xs, np.full(41, 0.20)], np.c_[xs, np.full(41, -0.20)],
                     np.c_[np.full(41, 0.192), np.linspace(-0.2, 0.2, 41)],
                     np.c_[np.full(41, -0.188), np.linspace(-0.2, 0.2, 41)]])
    assert E.best_move(gap_fn(pts), E.body_gap(pts), PIV) is None


def test_turn_about_pivot_moves_the_base():
    p = np.array([[1.0, 0.0]])
    q = E.moved(p, a=math.pi / 2, P=(0.0, 0.0))
    assert np.allclose(q, [[0.0, -1.0]])
    q2 = E.moved(np.array([[0.0, 0.0]]), a=math.pi, P=(0.1, 0.0))   # half turn about 10 cm ahead: base ends 20 cm ahead
    assert np.allclose(q2, [[0.2, 0.0]], atol=1e-9)
