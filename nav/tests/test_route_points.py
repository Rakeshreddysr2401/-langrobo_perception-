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


def case(t):
    return (FIX[f'lidar_{t}'].astype(float), FIX[f'thin_cm_{t}'] / 100.0, FIX[f'cells_8cm_{t}'] / 800.0,
            tuple(float(v) for v in FIX[f'goal_{t}']))


def walk(pts, goal):
    """Walking distance from where the rover stands, or None: reach's 'not reachable'."""
    d = math.hypot(*goal)
    h = TRT._guide(np.asarray(pts, float), goal, half=min(HALF, max(TRT.SEARCH_R + 0.2, d + 1.0)),
                   res=0.04, clear=CLEAR, extend=True)
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


if __name__ == '__main__':
    for f in (test_the_real_doors_are_reachable_on_the_route_points, test_nvblox_cells_alone_close_them,
              test_what_the_camera_cannot_see_now_stays, test_a_goal_deep_in_something_is_still_unreachable):
        f()
        print('ok  ', f.__name__)
    print('ALL PASS')
