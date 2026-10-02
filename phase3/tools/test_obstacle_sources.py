#!/usr/bin/env python3
"""test_obstacle_sources.py — offline checks of fused_obstacles + turn_gate.

    python3 phase3/tools/test_obstacle_sources.py      (no robot, no ROS graph)

fused_obstacles: each occupied cell is its four corners; a cell under the
rover is ignored, one just past its side is kept; a grid without a recent
depth frame handed to nvblox (or an old grid) is not a fresh view.
turn_gate: a frame inside a turn, or within BEFORE_S after it, is dropped;
the first moments after a start (no history yet) are dropped quietly; a
dead gyro lets frames through (the maps must not go blind with it).
"""
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parent / 'nodes'))
for d in (Path('/opt/rover2/nodes'), here.parent.parent / 'phase2' / 'nodes'):
    if (d / 'turn_gate.py').exists():
        sys.path.insert(0, str(d))
        break
import fused_obstacles as F  # noqa: E402
import turn_gate as TG  # noqa: E402


class FakeNode:
    def __init__(self):
        self.now = 100.0

    def create_subscription(self, *a, **k):
        return object()

    def destroy_subscription(self, s):
        pass

    def get_logger(self):
        return NS(info=lambda *a: None, warn=lambda *a: None)

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=int(self.now * 1e9)))


def test_fused():
    o = F.FusedObstacles(FakeNode(), lambda: (1.0, 2.0, math.radians(90)))
    assert o.age() == float('inf'), 'no grid yet = never fresh'
    data = np.zeros((60, 60), np.int8)

    def cell(x, y):
        data[int((y - 1.5) / 0.025), int((x - 0.5) / 0.025)] = 100
    cell(1.0, 2.5)                 # 0.5 m ahead of the rover (facing +y)
    cell(1.0, 2.0)                 # under the rover
    cell(1.0 - 0.23, 2.0)          # 4 cm past its left side
    o._grid(NS(info=NS(height=60, width=60, resolution=0.025, origin=NS(position=NS(x=0.5, y=1.5))),
               data=data.tobytes()))
    assert o.age() == float('inf'), 'no frame handed to nvblox yet: not fresh'
    o._info(None)
    assert o.age() < 0.1
    p = o.points_base()
    ahead, left = p[p[:, 0] > 0.4], p[p[:, 1] > 0.15]
    centre = p[(np.abs(p[:, 0]) < 0.1) & (np.abs(p[:, 1]) < 0.1)]
    assert len(ahead) == 4 and abs(np.ptp(ahead[:, 0]) - 0.023) < 1e-6, 'corners span the cell'
    assert len(left) == 4, 'a cell just past the side is kept'
    assert len(centre) == 0, 'a cell under the rover is ignored'
    assert o.mem.shape[1] == 5 and (o.mem[:, 4] >= F.MIN_HITS).all()
    o.info_t -= 5
    assert o.age() > 4.9, 'no frame to nvblox for 5 s: stale even with a fresh grid'
    print('  ok  fused_obstacles: corners, own outline, freshness')


def test_gate():
    n = FakeNode()
    g = TG.TurnGate(n)
    t0 = n.now

    def feed(a, b, wz):                                    # gyro at 200 Hz from t0+a to t0+b
        for k in range(int(round((b - a) / 0.005))):
            s = t0 + a + k * 0.005
            g._gyro(NS(header=NS(stamp=NS(sec=int(s), nanosec=int(round((s % 1) * 1e9)))),
                       angular_velocity=NS(z=wz)))
    assert not g.still(t0 + 1.0), 'starting up, no history yet: dropped quietly'
    feed(1.0, 2.0, 0.0)
    assert g.still(t0 + 1.95), 'standing still'
    feed(2.0, 2.5, 1.0)
    assert not g.still(t0 + 2.3), 'inside a turn'
    feed(2.5, 4.0, 0.0)
    assert not g.still(t0 + 2.5 + TG.BEFORE_S - 0.2), 'stopped, but the slide fix has not landed yet'
    assert g.still(t0 + 3.9), 'settled'
    assert g.still(t0 + 30.0), 'gyro dead (no sample for 26 s): frames pass, never blind'
    print('  ok  turn_gate: start-up, still, turn, settle, dead gyro')


if __name__ == '__main__':
    test_fused()
    test_gate()
    print('SUITE PASS')
