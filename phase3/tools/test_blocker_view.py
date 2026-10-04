#!/usr/bin/env python3
"""test_blocker_view.py — offline checks of the blocker view's pure parts.

    python3 phase3/tools/test_blocker_view.py      (in the container: imports rclpy messages)

classify: each lethal cell by who says so. coarsen: the room view's 10 cm
squares (owner, 2026-10-04, "only able to see blue" at whole-room zoom) --
one colour per square, both > LiDAR > camera > erased, edge squares kept.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
import blocker_view as B  # noqa: E402

W = H = 8      # an 8 x 8 grid of cells, in 4 x 4 blocks -> 2 x 2 blocks


def flat(x, y, w=W):
    return y * w + x


def test_classify_splits_by_source():
    li = np.zeros(W * H, np.int16)
    ca = np.zeros(W * H, np.int16)
    li[[flat(0, 0), flat(1, 0)]] = 100
    ca[[flat(1, 0), flat(2, 0)]] = 100
    g = B.classify(li, ca)
    assert g['lidar'].tolist() == [flat(0, 0)]
    assert g['both'].tolist() == [flat(1, 0)]
    assert g['camera'].tolist() == [flat(2, 0)]


def test_a_square_takes_its_most_important_colour():
    g = {'camera': [flat(0, 0), flat(1, 1),       # block (0,0), with LiDAR below
                    flat(6, 6)],                  # block (1,1), camera alone
         'lidar': [flat(2, 2)],                   # block (0,0)
         'erased': [flat(5, 1)],                  # block (1,0)
         'both': [flat(1, 6)]}                    # block (0,1)
    c = B.coarsen(g, W, H, 4)
    assert c['lidar'].tolist() == [0], 'legs must not hide behind a low thing'
    assert c['erased'].tolist() == [1]
    assert c['both'].tolist() == [2]
    assert c['camera'].tolist() == [3]


def test_both_beats_everything():
    g = {'both': [flat(3, 3)], 'lidar': [flat(0, 0)], 'camera': [flat(1, 1)], 'erased': [flat(2, 2)]}
    c = B.coarsen(g, W, H, 4)
    assert c['both'].tolist() == [0] and not c['lidar'].size and not c['camera'].size


def test_edge_squares_are_kept():
    w, h = 6, 5                                        # not a multiple of 4
    g = {'camera': [flat(5, 4, w)]}
    c = B.coarsen(g, w, h, 4)
    assert c['camera'].tolist() == [1 * 2 + 1]         # block (1,1) of a 2 x 2 block grid


if __name__ == '__main__':
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print(f'  PASS  {name}')
            except AssertionError as e:
                fails += 1
                print(f'  FAIL  {name}: {e}')
    print('ALL PASS' if not fails else f'{fails} FAILED')
    sys.exit(1 if fails else 0)
