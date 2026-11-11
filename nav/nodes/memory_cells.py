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


def route_points(lidar, precise, cells, r_near):
    """The obstacle points a ROUTE is laid on (reach rl_round, rl2), base_link, N x 2.

    Two kinds of camera knowledge, each used for what it is good at (real robot 2026-11-10,
    kino_sim docs/ROBUSTNESS.md R2): the costmap's cells (nvblox, 2.5 cm, every height in one
    slice) cost 3-8 cm on EACH side of a way -- a 55-62 cm door read 43-47 cm and the route
    (44 cm) called the goal unreachable -- while the live depth memory draws the same things
    1-3 cm thin but knows only what the camera has looked at since it was switched on.
      within r_near, in the camera's view:   LiDAR + the precise points
      within r_near, out of its view:        LiDAR + the costmap's cells (beside, behind, under the nose)
      beyond r_near:                         LiDAR + the costmap's cells
    The SHIELD does not use this: every command is still checked on all points."""
    lidar = np.asarray(lidar, float).reshape(-1, 2)
    precise = np.asarray(precise, float).reshape(-1, 2)
    cells = np.asarray(cells, float).reshape(-1, 2)
    parts = [lidar]
    if len(precise):
        parts.append(precise[(np.hypot(precise[:, 0], precise[:, 1]) <= r_near) & TRT.in_view(precise)])
    if len(cells):
        far = np.hypot(cells[:, 0], cells[:, 1]) > r_near
        parts.append(cells[far | ~TRT.in_view(cells)])
    return np.vstack(parts)
