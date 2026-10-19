"""python3 -m pytest phase3/nodes/test_sonar_witness.py"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sonar_witness as W  # noqa: E402

FOV = 0.52                     # params.yaml sonar.fov (the FULL cone)
LEFT, RIGHT = (0.182, 0.14, 0.0), (0.182, -0.14, 0.0)


def test_unseen_slipper_ahead_becomes_a_point():
    # the map has nothing ahead; the left sonar hears something 15 cm out
    p = W.witness_points([(0.15, LEFT), (math.inf, RIGHT)], np.zeros((0, 2)), FOV)
    assert p.shape == (1, 2) and abs(p[0, 0] - (0.182 + 0.15)) < 1e-9 and abs(p[0, 1] - 0.14) < 1e-9


def test_doorframe_the_map_already_has_is_not_added():
    # a jamb 30 cm ahead, 8 cm outside the left sensor's axis -- inside its cone -- and
    # the LiDAR has it: the echo is explained, nothing added (no wall across the path)
    jamb = np.array([[0.182 + 0.30, 0.14 + 0.08 + d] for d in np.linspace(0, 0.1, 6)])
    assert W.witness_points([(0.31, LEFT)], jamb, FOV).shape == (0, 2)


def test_known_point_behind_the_echo_does_not_explain_it():
    # the map has a wall 50 cm ahead; the sonar hears something at 15 cm: a low thing in front of it
    wall = np.array([[0.182 + 0.50, y] for y in np.linspace(-0.4, 0.4, 41)])
    p = W.witness_points([(0.15, LEFT), (0.50, RIGHT)], wall, FOV)
    assert p.shape == (1, 2) and abs(p[0, 1] - 0.14) < 1e-9          # only the left one; the right is the wall


def test_far_echoes_and_no_echo_are_ignored():
    assert W.witness_points([(0.9, LEFT), (math.nan, RIGHT), (math.inf, LEFT)], None, FOV).shape == (0, 2)


def test_a_point_outside_the_cone_does_not_explain():
    # something known 15 cm ahead but 25 cm to the side of the sensor axis: not what it heard
    side = np.array([[0.182 + 0.15, 0.14 + 0.25]])
    assert W.witness_points([(0.15, LEFT)], side, FOV).shape == (1, 2)


# ── the sonar vouching a camera speck away ───────────────────────────────────
FIXTURE = Path(__file__).resolve().parent / 'fixtures' / 'door_speck_20261019.npz'


def _real_door():
    d = np.load(FIXTURE)
    return d['camera'].astype(float), d['lidar'].astype(float)


def _corridor(p, length=0.92):
    return (p[:, 0] > 0.182) & (p[:, 0] < 0.182 + length) & (np.abs(p[:, 1]) < 0.20)


def test_real_door_reflection_is_overruled_and_nothing_else_near():
    # the recorded moment: ONE cell in the 90 cm drive; the right sonar hears nothing
    cam, lid = _real_door()
    assert _corridor(cam).sum() == 4                       # one cell = its four corners
    drop = W.vouched_specks(cam, lid, [(RIGHT, [math.inf] * 3), (LEFT, [1.8, 1.8, 1.8])])
    assert not _corridor(cam[~drop]).any()                 # the drive is clear now
    assert drop.sum() <= 8                                 # the speck only, nothing of the door frames
    near = np.hypot(cam[:, 0], cam[:, 1]) < 1.0
    assert (drop & near).sum() == drop.sum()


def test_real_door_kept_when_the_sonar_hears_something_there():
    cam, lid = _real_door()
    assert not W.vouched_specks(cam, lid, [(RIGHT, [0.36, 0.37, 0.36])]).any()


def test_needs_enough_readings():
    cam, lid = _real_door()
    assert not W.vouched_specks(cam, lid, [(RIGHT, [math.inf, math.inf])]).any()


def _cells(x0, x1, y0, y1, step=0.025):
    xs, ys = np.arange(x0, x1 + 1e-9, step), np.arange(y0, y1 + 1e-9, step)
    return np.array([(x, y) for x in xs for y in ys])


def test_tabletop_edge_is_never_overruled():
    # a camera-only LINE of cells 30 cm ahead (an overhang the cone is under): silence proves nothing
    edge = _cells(0.48, 0.50, -0.6, 0.6)
    assert not W.vouched_specks(edge, None, [(LEFT, [math.inf] * 3), (RIGHT, [math.inf] * 3)]).any()


def test_slipper_patch_is_never_overruled():
    slipper = _cells(0.40, 0.50, -0.27, -0.01)              # 10 x 26 cm
    assert not W.vouched_specks(slipper, None, [(RIGHT, [math.inf] * 3)]).any()


def test_speck_off_the_sonar_axis_is_kept():
    speck = _cells(0.50, 0.52, 0.0, 0.02)                   # straight ahead of the centre, 20+ deg off both axes
    assert not W.vouched_specks(speck, None, [(LEFT, [math.inf] * 3), (RIGHT, [math.inf] * 3)]).any()
