#!/usr/bin/env python3
"""test_widest.py — the widest way, on synthetic LiDAR points (no ROS).

    python3 phase3/tools/test_widest.py

gap36 (twin 2026-10-20): a board wall with a 36 cm gap -> must read under the
rover's 42 cm; the 44 and 49 cm gaps (chairs44, door49) must read over it
(never refuse a gap that fits); an open room reads "nothing in the way".
"""
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
import widest as WD  # noqa: E402

NEED = 0.38 + 2 * 0.02


def line(x0, y0, x1, y1, step=0.01):
    n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step) + 1)
    return np.stack([np.linspace(x0, x1, n), np.linspace(y0, y1, n)], 1)


def room_with_gap(gap):
    """A 4 x 3 m room, a wall across it at x 1.2 with ONE gap of `gap` m at y=0
    (both faces of a 5 cm board, as a LiDAR sees it), the rover at the origin."""
    h = gap / 2
    walls = [line(-0.8, -1.5, 3.2, -1.5), line(-0.8, 1.5, 3.2, 1.5), line(-0.8, -1.5, -0.8, 1.5), line(3.2, -1.5, 3.2, 1.5)]
    for x in (1.175, 1.225):
        walls += [line(x, -1.5, x, -h), line(x, h, x, 1.5)]
    walls += [line(1.175, -h, 1.225, -h), line(1.175, h, 1.225, h)]
    return np.vstack(walls)


def test_gap36_is_too_tight():
    w, info = WD.widest(room_with_gap(0.36), (2.2, 0.0))
    print(f'  36 cm gap: widest {w * 100:.1f} cm ({info["ms"]} ms, {info["cells"]} cells)')
    assert w is not None and w < NEED, w


def test_gaps_that_fit_are_never_refused():
    for gap in (0.42, 0.44, 0.49, 0.60):
        w, _ = WD.widest(room_with_gap(gap), (2.2, 0.0))
        print(f'  {gap * 100:.0f} cm gap: widest {w * 100:.1f} cm')
        assert w >= NEED, (gap, w)


def test_off_centre_and_angled_goal():
    w, _ = WD.widest(room_with_gap(0.36), (2.5, 1.0))
    assert w < NEED, w
    w, _ = WD.widest(room_with_gap(0.50), (2.5, -1.0))
    assert w >= NEED, w


def test_open_floor_is_none():
    w, _ = WD.widest(np.array([[10.0, 10.0]]), (2.0, 0.0))
    assert w is None, w


def test_speed():
    t = time.time()
    WD.widest(room_with_gap(0.36), (3.0, 1.2))
    dt = time.time() - t
    print(f'  3.2 m goal: {dt * 1000:.0f} ms on this computer')
    assert dt < 2.0, dt


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
