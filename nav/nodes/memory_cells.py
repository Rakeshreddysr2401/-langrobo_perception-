"""memory_cells.py -- what the costmap remembers BESIDE and BEHIND the rover, for the
checks of turns and reverses. No ROS (takes a nav_msgs/OccupancyGrid-like object).

WHY (twin chairs44, 2026-11-01 code + 2026-11-06 run): coming back through the 44 cm
way, reach's look turns (gap_pass.turn_to) swung the outline 0.8 cm INTO a chair's
splayed foot (10 cm tall: below the LiDAR's 25 cm plane, camera only). The foot was
beside the rover, outside the camera's 90 deg view, and the depth memory those checks
use starts empty for each goal -- so for them nothing was there. The local costmap
(nvblox) still held it from the first goal.

What is added: LETHAL cells within R_MAX that the camera cannot vouch for right now
(outside tight_route.in_view: beside, behind, under its near view), pulled CELL_SLACK
toward base_link (a cell's centre is up to half a diagonal from the thing's edge).
Cells inside the outline are dropped (a real thing cannot be inside the rover; nvblox
noise under it would freeze every move). What the camera sees now is left to the
fresh depth points: the costmap's memory there holds glossy-floor specks (corridor_s).
"""
import math

import numpy as np

import tight_route as TRT
from goal_exec import FRONT, REAR, SIDE

CELL_SLACK = 0.018
R_MAX = 0.8


def lethal_cells(m, pose, r_max=R_MAX):
    """LETHAL (100) cells of grid m within r_max of pose (x, y, th), base_link, N x 2."""
    if m is None or pose is None:
        return np.zeros((0, 2))
    a = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    j, i = np.nonzero(a >= 100)
    if not len(i):
        return np.zeros((0, 2))
    r = m.info.resolution
    wx = m.info.origin.position.x + (i + 0.5) * r - pose[0]
    wy = m.info.origin.position.y + (j + 0.5) * r - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    b = np.stack([c * wx + s * wy, -s * wx + c * wy], 1)
    return b[np.hypot(b[:, 0], b[:, 1]) < r_max]


def unseen(cells):
    """The cells the camera cannot vouch for now, pulled CELL_SLACK toward base_link,
    none inside the outline."""
    if cells is None or not len(cells):
        return np.zeros((0, 2))
    b = np.asarray(cells, float)
    b = b[~TRT.in_view(b)]
    if not len(b):
        return b
    r = np.hypot(b[:, 0], b[:, 1])
    b = b * np.maximum(0.0, 1.0 - CELL_SLACK / np.maximum(r, 1e-6))[:, None]
    inside = (b[:, 0] < FRONT) & (b[:, 0] > -REAR) & (np.abs(b[:, 1]) < SIDE)
    return b[~inside]


def for_checks(m, pose):
    """lethal_cells + unseen: what turn / reverse checks add to their points."""
    return unseen(lethal_cells(m, pose))
