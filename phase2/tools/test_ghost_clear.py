#!/usr/bin/env python3
"""test_ghost_clear.py — offline checks of the ghost rule (phase2/nodes/ghost_clear.py).

    python3 phase2/tools/test_ghost_clear.py      (no robot, no ROS)

A room with a wall 5 m ahead along +x; the laser at the origin. Each test is
one of the owner's cases (2026-10-04): a person walks off -> erased; a shoe
-> kept; a shoe against the wall -> kept; the person comes back -> marked
again; marks under the rover -> erased; an erased mark the camera keeps
seeing -> real, restored.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
import ghost_clear as G  # noqa: E402

R = 0.025


def cells(x0, y0, x1, y1):
    """Global indices of the cells covering [x0, x1] x [y0, y1]."""
    xs = np.arange(math.floor(x0 / R), math.floor(x1 / R) + 1)
    ys = np.arange(math.floor(y0 / R), math.floor(y1 / R) + 1)
    return np.array([(i, j) for i in xs for j in ys])


def fan(ranges_fn, n=720):
    """A full scan from the origin: returns (laser_xy, hits)."""
    a = np.linspace(-math.pi, math.pi, n, endpoint=False)
    r = np.array([ranges_fn(t) for t in a])
    return np.zeros(2), np.stack([r * np.cos(a), r * np.sin(a)], 1)


def wall_only(t):
    return 5.0 / max(math.cos(t), 1e-3) if abs(t) < math.radians(30) else 3.0


def person_at_2m(t):
    return 2.0 if abs(t) < math.radians(1.5) else wall_only(t)


LEG = cells(1.98, -0.03, 2.05, 0.03)          # the camera's mark of the legs
SHOE = cells(2.0, 0.5, 2.08, 0.56)            # low: the LiDAR beam passes over it
SHOE_AT_WALL = cells(4.92, -0.03, 4.97, 0.03)


def erased(g, c):
    return g.erase_mask(G.keys_of(c[:, 0], c[:, 1]))


def test_a_person_who_walked_off_is_erased():
    g = G.GhostMap()
    g.on_scan(*fan(person_at_2m), now=0.0)
    g.on_grid(LEG, now=0.5)                    # nvblox marks the legs; LiDAR hit them
    assert erased(g, LEG).sum() == 0, 'still there: nothing erased yet'
    g.on_scan(*fan(wall_only), now=10.0)       # gone: beams pass through to the wall
    assert erased(g, LEG).all()


def test_a_shoe_is_kept():
    g = G.GhostMap()
    g.on_scan(*fan(wall_only), now=0.0)        # the LiDAR never hits a shoe
    g.on_grid(SHOE, now=0.5)
    for t in range(1, 20):
        g.on_scan(*fan(wall_only), now=float(t))
        g.on_grid(SHOE, now=t + 0.5)
    assert not erased(g, SHOE).any()


def test_a_shoe_against_the_wall_is_kept():
    """Tagged tall (the wall is within 5 cm) but no beam vouches that far."""
    g = G.GhostMap()
    for t in range(20):
        g.on_scan(*fan(wall_only), now=float(t))
        g.on_grid(SHOE_AT_WALL, now=t + 0.5)
    assert not erased(g, SHOE_AT_WALL).any()


def test_the_person_coming_back_is_marked_again():
    g = G.GhostMap()
    g.on_scan(*fan(person_at_2m), now=0.0)
    g.on_grid(LEG, now=0.5)
    g.on_scan(*fan(wall_only), now=10.0)
    assert erased(g, LEG).all()
    g.on_scan(*fan(person_at_2m), now=11.0)    # back in the same spot
    assert not erased(g, LEG).any()


def test_marks_under_the_rover_are_erased():
    g = G.GhostMap()
    under = cells(-0.05, -0.05, 0.05, 0.05)
    beside = cells(0.40, -0.02, 0.45, 0.02)     # outside the body: not this rule's business
    g.on_grid(np.vstack([under, beside]), now=0.0)
    g.on_pose((0.0, 0.0, 0.0), camera_live=False, dt=0.2)
    assert erased(g, under).all() and not erased(g, beside).any()


def test_an_erased_mark_in_plain_view_is_restored():
    """A low thing put down where a person stood: the camera keeps seeing it."""
    g = G.GhostMap()
    g.on_scan(*fan(person_at_2m), now=0.0)
    g.on_grid(LEG, now=0.5)
    g.on_scan(*fan(wall_only), now=10.0)
    assert erased(g, LEG).all()
    for _ in range(int(G.VIEW_S / 0.2) + 1):   # facing it (+x), camera feeding nvblox
        g.on_pose((0.0, 0.0, 0.0), camera_live=True, dt=0.2)
    assert not erased(g, LEG).any()
    g.on_scan(*fan(wall_only), now=20.0)       # and the LiDAR passing over it no longer erases it
    assert not erased(g, LEG).any()


def test_nothing_counts_in_view_while_turning():
    g = G.GhostMap()
    g.on_scan(*fan(person_at_2m), now=0.0)
    g.on_grid(LEG, now=0.5)
    g.on_scan(*fan(wall_only), now=10.0)
    for _ in range(40):
        g.on_pose((0.0, 0.0, 0.0), camera_live=False, dt=0.2)
    assert erased(g, LEG).all()


def test_state_follows_nvblox_letting_go():
    g = G.GhostMap()
    g.on_scan(*fan(person_at_2m), now=0.0)
    g.on_grid(LEG, now=0.5)
    g.on_scan(*fan(wall_only), now=10.0)
    g.on_grid(np.zeros((0, 2), int), now=10.5)  # nvblox cleared it itself
    assert not g.tall and not g.cleared


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
