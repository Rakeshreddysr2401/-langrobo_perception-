#!/usr/bin/env python3
"""test_escape.py — offline checks of reach_node's escape (straight, then turn).

    python3 phase3/tools/test_escape.py      (in the rover container; no robot)

2026-10-03 night: right side 4.9 cm from a chest of drawers, left open, 25 cm
ahead, 22 cm behind. The planner refused the start 8 times; escape tried only
straight moves (all closed in) and only below 3 cm. Now: pinned below 5 cm, and
a turn in place that opens space is taken -- toward the open side, preferring
the goal. Between two walls (a corridor) no turn fits and none is tried.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "nodes"))
import reach_node as RN  # noqa: E402
from goal_exec import SIDE  # noqa: E402


def wall(x0, x1, y, n=60):
    return np.column_stack([np.linspace(x0, x1, n), np.full(n, y)])


class Fake(RN.Reach):
    def __init__(self, pts, cells=None):            # no ROS
        self.pose = (0.0, 0.0, 0.0)
        self._pts, self._cells = pts, (np.zeros((0, 2)) if cells is None else cells)
        self.said, self.turned, self.straight = [], [], None

    def near_now(self, scans):
        return self._pts, self._cells

    def say(self, **d):
        self.said.append(d)

    def run_goal_exec_turn(self, heading):
        self.turned.append(math.degrees(heading))
        return 'reached', ''

    def drive_straight(self, step, g0, floor=None):
        self.straight = step
        return 'reached', ''


HERE = Path(__file__).resolve().parent


def chest_on_right():
    """The real obstacle points (base_link) where it was refused 8 times:
    goal_exec's /goal_exec/obstacles at pose (1.65, 1.54, 177 deg)."""
    return np.load(HERE / "fixture_chest_right_2026-10-03.npy").astype(np.float64)


def test_turns_toward_open_side():
    f = Fake(chest_on_right())
    f.scans = []
    assert f.escape(1, g=(2.0, 1.0)) is True, f.said
    plan = next(d["steps"] for d in f.said if "steps" in d)
    assert plan[0][0] == "turn" and plan[0][1] > 0, plan               # first: left, away from the chest
    assert f.turned and f.turned[0] > 0, (f.turned, f.said)


def test_turns_with_live_costmap_cells():
    # the same spot, with the local costmap's lethal cells as reach saw them
    # live: g0 3.0 cm (cells less their rounding); the turn must still be found
    cells = np.load(HERE / "fixture_chest_right_cells_2026-10-03.npy").astype(np.float64)
    f = Fake(chest_on_right(), cells)
    f.scans = []
    assert f.escape(1, g=(2.0, 1.0)) is True, f.said
    assert f.turned and f.turned[0] > 0, (f.turned, f.said)


def test_long_wall_alongside_no_turn():
    # a wall the whole length of the right side, 4.9 cm off: a corner swings
    # out to 0.263 m, the wall is 0.239 m away -- no turn fits, none is tried
    pts = np.vstack([wall(-0.6, 0.6, -(SIDE + 0.049), 120),
                     np.column_stack([np.full(20, 0.182 + 0.246), np.linspace(-0.6, -0.25, 20)]),
                     np.column_stack([np.full(20, -0.178 - 0.22), np.linspace(-0.6, -0.25, 20)])])
    f = Fake(pts)
    f.scans = []
    f.escape(1, g=(2.0, 1.0))
    assert not f.turned, f.said


def test_prefers_goal_side_on_ties():
    # walls on neither side within reach, pinned by a point at the front-left corner only
    pts = np.array([[0.182 + 0.02, SIDE - 0.05]])
    f = Fake(pts)
    f.scans = []
    f.escape(1, g=(0.0, -2.0))                                    # goal to the right
    turns = [d.get("turn_deg") for d in f.said if "turn_deg" in d]
    # a straight move may solve it first; if a turn is chosen it must not be toward the point
    assert not turns or turns[0] < 0, f.said


def test_corridor_no_turn():
    pts = np.vstack([wall(-0.6, 0.6, -(SIDE + 0.03)), wall(-0.6, 0.6, SIDE + 0.03),
                     np.column_stack([np.full(20, 0.182 + 0.04), np.linspace(-0.2, 0.2, 20)]),
                     np.column_stack([np.full(20, -0.178 - 0.04), np.linspace(-0.2, 0.2, 20)])])
    f = Fake(pts)
    f.scans = []
    assert f.escape(1, g=(2.0, 0.0)) is False
    assert not f.turned
    assert "opens space" in f.said[-1].get("outcome", ""), f.said


def test_not_pinned_does_nothing():
    f = Fake(wall(-0.4, 0.4, -(SIDE + 0.20)))
    f.scans = []
    assert f.escape(1, g=(2.0, 0.0)) is False and not f.said


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"ok  {t.__name__}")
    print(f"{len(tests)} passed")
