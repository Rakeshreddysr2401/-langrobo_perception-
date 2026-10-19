"""python3 phase3/nodes/test_house_route.py  (or pytest)"""
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import house_route as H  # noqa: E402

RES = 0.05


def house(door_w=0.78, unknown_right=False):
    """Two 4 x 4 m rooms side by side, a wall at x = 4 with one door at y 3.0..3.0+door_w."""
    W, Hh = int(8.4 / RES), int(4.4 / RES)
    g = np.zeros((Hh, W), np.int16)
    xs = (np.arange(W) + .5) * RES - 0.2
    ys = (np.arange(Hh) + .5) * RES - 0.2
    X, Y = np.meshgrid(xs, ys)
    wall = (X < 0) | (X > 8.0) | (Y < 0) | (Y > 4.0)
    wall |= (np.abs(X - 4.0) < 0.05) & ~((Y > 3.0) & (Y < 3.0 + door_w))
    g[wall] = 100
    if unknown_right:
        g[(X > 4.05) & ~wall] = -1
    return g, -0.2, -0.2


def test_through_the_door_not_the_wall():
    g, ox, oy = house()
    pts, L = H.route(g, RES, ox, oy, (2.0, 0.8), (6.0, 0.8))
    assert pts is not None, L
    # it goes up to the door (y ~3.4) and back down: much longer than the 4 m straight line
    assert max(p[1] for p in pts) > 3.0 and 7.0 < L < 10.0, L
    crossing = [p for a, p in zip(pts, pts[1:]) if (a[0] - 4.0) * (p[0] - 4.0) <= 0]
    assert all(3.0 < p[1] < 3.78 for p in crossing)


def test_keeps_to_the_middle_of_the_door():
    g, ox, oy = house()
    pts, _ = H.route(g, RES, ox, oy, (2.0, 0.8), (6.0, 0.8))
    at = min(pts, key=lambda p: abs(p[0] - 4.0))
    assert abs(at[1] - 3.39) < 0.12, at


def test_narrow_door_closed_for_this_body():
    g, ox, oy = house(door_w=0.36)
    pts, why = H.route(g, RES, ox, oy, (2.0, 0.8), (6.0, 0.8))
    assert pts is None and 'no route' in why


def test_unseen_room_is_allowed():
    g, ox, oy = house(unknown_right=True)
    pts, L = H.route(g, RES, ox, oy, (2.0, 0.8), (6.0, 0.8))
    assert pts is not None


def test_goal_against_furniture_snaps_to_open_floor():
    g, ox, oy = house()
    pts, L = H.route(g, RES, ox, oy, (2.0, 2.0), (3.95, 1.0))     # 5 cm off the wall
    assert pts is not None


def test_along():
    p, th = H.along([(0, 0), (1, 0), (1, 1)], 1.5)
    assert np.allclose(p, (1, 0.5)) and abs(th - math.pi / 2) < 1e-9


def test_fast_enough_for_a_house():
    g, ox, oy = house()
    big = np.zeros((400, 400), np.int16)                  # 20 x 20 m
    big[:g.shape[0], :g.shape[1]] = g
    t = time.time()
    H.route(big, RES, ox, oy, (0.5, 0.5), (19.0, 19.0))
    assert time.time() - t < 2.0


if __name__ == '__main__':
    for n, f in sorted(globals().items()):
        if n.startswith('test_'):
            f()
            print('PASS', n)
