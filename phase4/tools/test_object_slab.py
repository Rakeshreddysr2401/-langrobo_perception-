#!/usr/bin/env python3
"""test_object_slab.py — offline checks of pixel_to_goal.object_slab.

    python3 phase4/tools/test_object_slab.py      (no robot, no ROS graph)

door: a thin leg in front of a big door must not be the object (2026-10-03,
the goal landed in the furniture). bottle: a bottle in front of a wall still
is. Plus: one object alone, too few points, and the nearest of two real ones.
"""
import importlib.util
import sys
import types
from pathlib import Path

import numpy as np

here = Path(__file__).resolve().parent
# pixel_to_goal imports rclpy & co at module level; the function under test
# needs none of them, so stub what is missing (no ROS on a dev box).
for name in ("rclpy", "rclpy.node", "rclpy.time", "rclpy.duration", "rclpy.qos",
             "tf2_ros", "geometry_msgs", "geometry_msgs.msg", "sensor_msgs",
             "sensor_msgs.msg", "std_msgs", "std_msgs.msg", "cv_bridge"):
    if name not in sys.modules:
        try:
            importlib.import_module(name)
        except Exception:
            m = types.ModuleType(name)
            m.__getattr__ = lambda attr: type(attr, (), {})
            sys.modules[name] = m
spec = importlib.util.spec_from_file_location("p2g", here.parent / "nodes" / "pixel_to_goal.py")
p2g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p2g)

rng = np.random.default_rng(0)


def wall(x, n, y=(-0.4, 0.4), z=(0.05, 1.4)):
    """n points on a plane facing the camera at range x (camera at origin)."""
    return np.stack([np.full(n, x) + rng.normal(0, 0.01, n),
                     rng.uniform(*y, n), rng.uniform(*z, n)], 1)


def test_door_behind_leg():
    leg = wall(0.80, 86, y=(0.10, 0.13), z=(0.05, 0.45))     # chair leg, 86 points
    door = wall(3.0, 9000)                                    # the door
    c, idx, info = p2g.object_slab(np.vstack([leg, door]), (0.0, 0.0))
    assert abs(c[0] - 3.0) < 0.05, c
    assert info["skipped"]["points"] == 86 and info["share"] > 0.9, info   # the nearest BIG slab
    # what the old rule did, so the test shows the change
    old, _ = p2g.nearest_slab(np.vstack([leg, door]), (0.0, 0.0))
    assert abs(old[0] - 0.80) < 0.05, old


def test_bottle_in_front_of_wall():
    bottle = wall(1.5, 600, y=(-0.035, 0.035), z=(0.05, 0.30))   # ~20% of the box
    back = wall(2.5, 2400, y=(-0.15, 0.15))
    c, _, info = p2g.object_slab(np.vstack([bottle, back]), (0.0, 0.0))
    assert abs(c[0] - 1.5) < 0.05, c
    assert info["slab"] == "nearest" and "skipped" not in info, info


def test_scattered_clutter_takes_largest():
    # no slab is 8% of the box: many small things at different ranges
    parts = [wall(0.6 + 0.3 * k, 50, y=(-0.4, 0.4)) for k in range(20)]
    parts.append(wall(4.0, 70))   # the biggest, still only 6.5% of the box
    c, _, info = p2g.object_slab(np.vstack(parts), (0.0, 0.0))
    assert info["slab"] == "largest" and abs(c[0] - 4.0) < 0.05, (c, info)


def mid(pts, x0=-0.4, x1=0.4, band=0.5):
    """the centre mask _ground_box builds, here on y (camera at the origin)"""
    return np.abs(pts[:, 1] - (x0 + x1) / 2) <= band * (x1 - x0) / 2


def test_pillar_beside_door():
    # live 2026-10-03: pillar 1.61 m (887) clipped at the box's left edge,
    # the door 2.43 m (1119) behind it, centred
    pillar = wall(1.61, 887, y=(0.25, 0.4))
    door = wall(2.43, 1119, y=(-0.4, 0.3))
    pts = np.vstack([pillar, door])
    c, _, info = p2g.object_slab(pts, (0.0, 0.0))
    assert abs(c[0] - 1.61) < 0.05, "without the centre rule the pillar wins"
    c, _, info = p2g.object_slab(pts, (0.0, 0.0), centre=mid(pts))
    assert abs(c[0] - 2.43) < 0.05 and info["skipped"]["range_m"] < 1.7, (c, info)


def test_frame_round_door():
    # live 2026-10-03: frame 0.75 m (981) at both edges, the leaf 1.05 m (2810)
    frame = np.vstack([wall(0.75, 490, y=(-0.4, -0.32)), wall(0.75, 491, y=(0.32, 0.4))])
    leaf = wall(1.05, 2810, y=(-0.32, 0.32))
    pts = np.vstack([frame, leaf])
    c, _, info = p2g.object_slab(pts, (0.0, 0.0), centre=mid(pts))
    assert abs(c[0] - 1.05) < 0.05, (c, info)


def test_open_chair_still_the_chair():
    # legs at the edges, seat + backrest across the middle, wall behind
    legs = np.vstack([wall(1.0, 150, y=(-0.4, -0.35)), wall(1.0, 150, y=(0.35, 0.4))])
    seat = wall(1.05, 400, y=(-0.25, 0.25))
    back = wall(2.0, 1500)
    pts = np.vstack([legs, seat, back])
    c, _, info = p2g.object_slab(pts, (0.0, 0.0), centre=mid(pts))
    assert c[0] < 1.2, (c, info)


def test_bottle_with_centre():
    bottle = wall(1.5, 600, y=(-0.035, 0.035), z=(0.05, 0.30))
    back = wall(2.5, 2400, y=(-0.4, 0.4))
    pts = np.vstack([bottle, back])
    c, _, info = p2g.object_slab(pts, (0.0, 0.0), centre=mid(pts))
    assert abs(c[0] - 1.5) < 0.05, (c, info)


def test_single_object():
    c, idx, info = p2g.object_slab(wall(2.0, 500), (0.0, 0.0))
    assert abs(c[0] - 2.0) < 0.05 and info["slab"] == "nearest" and "skipped" not in info


def test_too_few_points():
    c, idx, info = p2g.object_slab(wall(2.0, 5), (0.0, 0.0))
    assert c is None and len(idx) == 0 and info == {}


def test_two_real_objects_nearest_wins():
    a = wall(1.0, 1000, y=(-0.4, -0.1))
    b = wall(2.0, 3000, y=(0.1, 0.4))
    c, _, info = p2g.object_slab(np.vstack([a, b]), (0.0, 0.0))
    assert abs(c[0] - 1.0) < 0.05 and info["slab"] == "nearest", (c, info)


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"ok  {t.__name__}")
    print(f"{len(tests)} passed")
