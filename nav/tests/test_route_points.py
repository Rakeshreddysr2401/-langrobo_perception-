#!/usr/bin/env python3
"""test_route_points.py — the rl2 route's obstacle points, on three REAL failures.

    python3 nav/tests/test_route_points.py      (numpy + scipy; no robot, no ROS)

Real robot, 2026-11-10 evening, three rl2 rounds that ended "the goal is not reachable on the
points seen" although the LiDAR measured the way 59 / 55 / 59 cm wide (kino_sim
docs/ROBUSTNESS.md R2). fixtures/real_door_20261110.npz holds, per round (seconds into the
recorded bag: 94, 159, 327), in base_link at the round's first step:
  lidar   the scan reach saved
  thin    the robot's own depth memory (depth_obstacles.py) replayed on the bag's depth frames
  cells   nvblox's occupied cells (corners) rebuilt from the same bag -- what reach read as
          "the camera's precise points" that evening
  goal    the round's target
The decision under test is reach's own: tight_route._guide at the rl2 settings, "blocked" when
the rover's place has no finite walking distance.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
import memory_cells as MC  # noqa: E402
import tight_route as TRT  # noqa: E402

FIX = np.load(Path(__file__).resolve().parent / 'fixtures' / 'real_door_20261110.npz')
R_NEAR, CLEAR, HALF = 2.4, 0.22, 3.9      # reach_node RL2_PRECISE_R, RL2_GUIDE_CLEAR, RL2_GUIDE_HALF
RES = 0.02                                 # reach_node RL2_GUIDE_RES (0.04 until 2026-11-15)


def case(t):
    return (FIX[f'lidar_{t}'].astype(float), FIX[f'thin_cm_{t}'] / 100.0, FIX[f'cells_8cm_{t}'] / 800.0,
            tuple(float(v) for v in FIX[f'goal_{t}']))


def walk(pts, goal):
    """Walking distance from where the rover stands, or None: reach's 'not reachable'."""
    d = math.hypot(*goal)
    h = TRT._guide(np.asarray(pts, float), goal, half=min(HALF, max(TRT.SEARCH_R + 0.2, d + 1.0)),
                   res=RES, clear=CLEAR, extend=True)
    if h is None:
        return None
    best = min(h(r * math.cos(k * math.pi / 4), r * math.sin(k * math.pi / 4))
               for r in (0.0, 0.05, 0.10, 0.15, 0.20) for k in range(8))
    return best if math.isfinite(best) else None


def test_the_real_doors_are_reachable_on_the_route_points():
    for t in (94, 159, 327):
        lidar, thin, cells, goal = case(t)
        w = walk(MC.route_points(lidar, thin, cells, R_NEAR), goal)
        straight = math.hypot(*goal)
        assert w is not None, f't={t}: the way the LiDAR measures 55-62 cm wide must be reachable'
        assert straight - 0.30 <= w <= straight + 0.8, f't={t}: walking {w:.2f} m for {straight:.2f} m straight (a detour = a closed door)'


def test_nvblox_cells_alone_close_them():
    """Why route_points exists: on the cells alone (as on 2026-11-10) two of the three are closed."""
    closed = [t for t in (94, 159, 327) if walk(np.vstack(case(t)[::2]), case(t)[3]) is None]
    assert closed == [94, 327], closed


def test_what_the_camera_cannot_see_now_stays():
    """Beside and behind, only the costmap knows the low things: its cells there are kept."""
    lidar, thin, cells, goal = case(94)
    pts = MC.route_points(lidar, thin, cells, R_NEAR)
    behind = cells[(cells[:, 0] < -0.3) & (np.hypot(cells[:, 0], cells[:, 1]) < R_NEAR)]
    assert len(behind) > 50, 'the fixture has cells behind the rover'
    kept = {(round(x, 4), round(y, 4)) for x, y in pts}
    assert all((round(x, 4), round(y, 4)) in kept for x, y in behind), 'cells out of the camera\'s view must stay'
    ahead = cells[TRT.in_view(cells) & (np.hypot(cells[:, 0], cells[:, 1]) <= R_NEAR)]
    assert len(ahead) > 50 and not any((round(x, 4), round(y, 4)) in kept for x, y in ahead[:200]), 'cells in view, near: replaced by the precise points'


def test_a_goal_deep_in_something_is_still_unreachable():
    """The end of the route may move to the nearest place that fits -- not through a wall."""
    wall = np.array([[x, y] for x in np.arange(0.8, 2.2, 0.02) for y in np.arange(-1.5, 1.5, 0.02)])
    assert walk(wall, (1.5, 0.0)) is None, 'a goal 70 cm inside a solid block'
    # and a goal 12 cm from a lone thing is reached at the nearest place that fits
    w = walk(np.array([[1.5, 0.12], [1.5, 0.13]]), (1.5, 0.0))
    assert w is not None and 1.2 < w < 1.8, w




# ── 2026-11-15, real robot, rl2_r011: two rounds quit "not reachable" at a door it then went through ──
# fixtures/real_guide_20261115.npz, base_link at the step the round quit (session bag 08:25:32 and
# 08:33:10 UTC; kino_sim build/real_20261115/session): lid = the last scan, pts = the shield's points
# (LiDAR + camera + sonar; stands in for the depth memory, which the dump does not keep), cells = the
# global costmap's lethal cells from the recorded /global_costmap/costmap, gb = the goal.
NOV15 = np.load(Path(__file__).resolve().parent / 'fixtures' / 'real_guide_20261115.npz')
FAT = 0.05                                  # reach_node RL2_CELL_FAT


def walk_split(pts, cells, goal, cells_clear, res=RES):
    d = math.hypot(*goal)
    h = TRT._guide(np.asarray(pts, float), goal, half=min(HALF, max(TRT.SEARCH_R + 0.2, d + 1.0)),
                   res=res, clear=CLEAR, extend=True, cells=cells, cells_clear=cells_clear)
    if h is None:
        return None
    best = min(h(r * math.cos(k * math.pi / 4), r * math.sin(k * math.pi / 4))
               for r in (0.0, 0.05, 0.10, 0.15, 0.20) for k in range(8))
    return best if math.isfinite(best) else None


def test_nov15_what_the_robot_did_closed_the_door():
    """The robot's guide then: 4 cm grid, cells at the full 22 cm -> no way (both rounds quit)."""
    for st in ('082532', '083310'):
        pts, cells = MC.route_split(NOV15[f'lid_{st}'], NOV15[f'pts_{st}'], NOV15[f'cells_{st}'], R_NEAR)
        assert walk_split(pts, cells, tuple(NOV15[f'gb_{st}']), CLEAR, res=0.04) is None, st


def test_nov15_each_part_of_the_fix_is_needed():
    """08:25 is closed by the 4 cm grid (open at 2 cm even with the cells at 22 cm); 08:33 by the cells'
    extra width (closed at 2 cm with the cells at 22 cm)."""
    def w(st, cc, res):
        pts, cells = MC.route_split(NOV15[f'lid_{st}'], NOV15[f'pts_{st}'], NOV15[f'cells_{st}'], R_NEAR)
        return walk_split(pts, cells, tuple(float(v) for v in NOV15[f'gb_{st}']), cc, res=res)
    assert w('082532', CLEAR, 0.04) is None and w('082532', CLEAR, 0.02) is not None
    assert w('083310', CLEAR, 0.02) is None and w('083310', CLEAR - FAT, 0.02) is not None


def test_nov15_cells_less_their_extra_width_leave_it_open():
    """The fix: the cells kept FAT closer -> a way, and not a detour round the house."""
    for st in ('082532', '083310'):
        goal = tuple(float(v) for v in NOV15[f'gb_{st}'])
        pts, cells = MC.route_split(NOV15[f'lid_{st}'], NOV15[f'pts_{st}'], NOV15[f'cells_{st}'], R_NEAR)
        w = walk_split(pts, cells, goal, CLEAR - FAT)
        assert w is not None, f'{st}: the door the rover went through must be reachable'
        assert w <= math.hypot(*goal) + 0.3, f'{st}: walking {w:.2f} m for {math.hypot(*goal):.2f} m: a detour, not the door'


def test_cells_still_close_a_real_wall():
    """Less room from cells is not a way through them: a block of cells is still a block."""
    wall = np.array([[x, y] for x in np.arange(0.8, 2.2, 0.025) for y in np.arange(-1.5, 1.5, 0.025)])
    lid = np.array([[3.0, 3.0]])
    assert walk_split(lid, wall, (1.5, 0.0), CLEAR - FAT) is None
    wall = np.array([[x, y] for x in np.arange(0.8, 1.0, 0.025) for y in np.arange(-4.5, 4.5, 0.025)])  # wall to wall: no way round
    # a 24 cm slot between cells = ~34 cm really (cells are >= 5 cm too wide a side): the rover is 38 cm
    gap = wall[np.abs(wall[:, 1]) > 0.12]
    assert walk_split(lid, gap, (2.6, 0.0), CLEAR - FAT) is None, 'a slot narrower than the rover stays closed'
    # ... and one with room (44 cm between cells = ~54 cm really) is open
    gap = wall[np.abs(wall[:, 1]) > 0.22]
    assert walk_split(lid, gap, (2.6, 0.0), CLEAR - FAT) is not None, 'a slot the rover fits is open'


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok  ', name)
    print('ALL PASS')
