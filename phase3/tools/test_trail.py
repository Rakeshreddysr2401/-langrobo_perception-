#!/usr/bin/env python3
"""test_trail.py — offline checks of reach's trail (phase3/nodes/trail.py).

    python3 phase3/tools/test_trail.py      (no robot, no ROS)

The 2026-10-04 drive, replayed: the rover drove in along a corridor
(-0.39,-0.03) -> (0.87,-0.66) -> (2.06,-0.63) -> (3.46,-0.41); the goal was
back at (-0.73,-0.68). The way back must follow the corridor to its far end,
in hops of ~1 m, not head off sideways. And a pose jump (carried, or a pose
reset) must not be read as a road.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
import trail as T  # noqa: E402


def drive(tr, pts, step=0.05):
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        for k in range(n + 1):
            tr.add(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n)


CORRIDOR = [(-0.39, -0.03), (0.87, -0.66), (2.06, -0.63), (3.46, -0.41)]


def test_way_back_follows_the_corridor():
    tr = T.Trail()
    drive(tr, CORRIDOR)
    hops = tr.way_back((3.46, -0.41, 0.0), (-0.73, -0.68))
    assert hops, 'the trail leads nearer the goal: there must be a way back'
    end = hops[-1]
    assert math.hypot(end[0] + 0.39, end[1] + 0.03) < 0.2, end   # the far end of the trail
    for (x, y, _) in hops:                                        # never off the corridor
        assert -0.8 < y < 0.1, (x, y)
    gaps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(hops, hops[1:])]
    assert all(g <= T.HOP_M + T.SPACING + 1e-6 for g in gaps), gaps
    first = hops[0]
    assert math.cos(first[2]) < -0.9, 'first hop heads back west, the way it came'


def test_no_way_back_when_the_trail_leads_away():
    tr = T.Trail()
    drive(tr, CORRIDOR)
    assert tr.way_back((3.46, -0.41, 0.0), (5.0, -0.4)) == []


def test_no_way_back_when_the_rover_is_off_its_trail():
    tr = T.Trail()
    drive(tr, CORRIDOR)
    assert tr.way_back((0.95, -2.10, 0.0), (-0.73, -0.68)) == []


def test_a_jump_starts_a_new_trail():
    tr = T.Trail()
    drive(tr, CORRIDOR[:2])
    drive(tr, [(0.47, -1.04), (1.2, -1.04)])     # set down somewhere else, drove on
    assert all(x >= 0.47 - 1e-6 for x, _ in tr.pts), 'the old trail must be gone'


def test_a_carry_is_not_a_road():
    """Carried while reach was idle (no samples), then driven again: the first
    new sample is far from the last trail point -- not joined to it."""
    tr = T.Trail()
    drive(tr, CORRIDOR[:2])
    tr.add(3.0, -2.0)
    assert tr.pts == [(3.0, -2.0)]


def test_same_failure_from_the_same_spot():
    prev = ('no path: planner found none', (0.81, -1.97))
    assert T.same_failure(prev, 'no path: planner found none', (0.82, -1.95))
    assert not T.same_failure(prev, 'no path: planner found none', (1.5, -1.97))
    assert not T.same_failure(prev, 'narrow: nav2 path follower saw collision ahead', (0.81, -1.97))
    assert not T.same_failure(None, 'no path: planner found none', (0.81, -1.97))


def test_way_back_note_blames_ghosts_only_for_no_path():
    assert 'marks left by people' in T.way_back_note('no path: planner found none')
    tight = T.way_back_note('narrow: nav2 controller found no motion that fits')
    assert 'too tight' in tight and 'people' not in tight
    assert 'could not drive back' in T.way_back_note('out of time for this goal')


def test_the_shortest_way_not_the_whole_history():
    """Real robot 2026-10-20: 24- and 32-hop way backs for ~2 m nearer. The
    rover went out along y=0, round a 3 x 2 m loop twice (the second lap 15 cm
    beside the first, as real driving is), and back near where the loop began;
    the goal is behind the start. The way back must cut across, not replay
    both laps (the old code: 12 hops, 12.2 m)."""
    tr = T.Trail()
    loop = [(1.0, 0.0), (4.0, 0.0), (4.0, 2.0), (1.0, 2.0), (1.0, 0.0)]
    lap2 = [(x, y + 0.15) for x, y in loop]
    drive(tr, [(-1.0, 0.0)] + loop + lap2 + [(2.0, 0.15)])
    hops = tr.way_back((2.0, 0.15, 0.0), (-2.0, 0.0))
    assert hops, tr.last
    L = tr.last['route_m']
    assert L < 3.5, (L, len(hops))
    assert len(hops) <= 4, hops


def test_a_way_back_that_wanders_is_not_taken():
    """A long detour that gains little: say so, do not drive it."""
    tr = T.Trail()
    drive(tr, [(0.0, 0.0), (0.0, 3.0), (3.0, 3.0), (3.0, 0.6), (1.2, 0.6)])
    hops = tr.way_back((1.2, 0.6, 0.0), (-0.6, 0.0))
    assert not hops, hops
    assert 'drives' in tr.last.get('why', ''), tr.last


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
    print(f'{"ALL PASS" if not fails else f"{fails} FAILED"}')
    sys.exit(1 if fails else 0)
