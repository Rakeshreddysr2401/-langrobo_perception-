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
