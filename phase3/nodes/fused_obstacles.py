"""fused_obstacles.py — obstacles from nvblox's FUSED map, for exact moves.

Same surface as DepthObstacles (set_active, age, points_base, mem, frames),
so goal_exec, gap_pass and reach take either (make_obstacles below;
ROVER_OBSTACLES=depth goes back to the live 1 cm memory).

WHY (2026-10-03, the 45 cm gap, owner watching). The live memory reads one
frame at a time, and with the D555's emitter off (load-bearing for cuVSLAM,
phase1/README.md) passive stereo guesses on the white wall and the glossy
floor: 73-646 points in mid-air 0.5-0.9 m ahead, 3-30 cm high, where the
floor was empty. Every depth preset that removed them removed the chair leg
too (presets 0/4/3/2: leg 7/1/5/0 points, phantoms 73/3/12/0). nvblox fuses
every frame from every pose (TSDF), so a guess that moves with the view
averages out and a thing seen well from further back stays: at the same
spot, leg 10 cells, bottle 6, open floor 9 -- and LiDAR + this map found the
gap's straight line, 44.5 cm. It is also the owner's idea of a robot that
"sees from far away and remembers where things are".

Each occupied cell (2.5 cm) is its four corners, so no cell reads smaller
than it is. Points deep inside the rover outline are ignored (the rover is
standing there; same rule as DepthObstacles.points_base). Fresh = a grid
recently AND a depth frame recently handed to nvblox (depth_gate's
/depth_gate/passed): the grid keeps publishing when the camera or the gate
has died, and a frozen map must not read as a fresh view. While the rover
turns the gate passes nothing, so the view ages -- as it should.
"""
import math
import os
import time

import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

GRID = '/nvblox_node/static_occupancy_grid'
PASSED = '/depth_gate/passed'    # depth_gate.py: a frame went to nvblox
RANGE = 2.5                      # m around the rover
OCC = 50                         # occupancy >= this is an obstacle (nvblox: 0 / 100 / -1)
OWN_FRONT, OWN_REAR, OWN_SIDE = 0.202, 0.198, 0.21
OWN_INSET = 0.05
MIN_HITS = 2                     # mem rows carry this, for code that filters on hits


def make_obstacles(node, tf_buffer, get_pose, active=True):
    """The obstacle source for exact moves: the fused map (default) or, with
    ROVER_OBSTACLES=depth, the live 1 cm depth memory."""
    if os.environ.get('ROVER_OBSTACLES', 'fused') == 'depth':
        from depth_obstacles import DepthObstacles
        node.get_logger().info('obstacles: LIVE depth memory (ROVER_OBSTACLES=depth)')
        return DepthObstacles(node, tf_buffer, get_pose, active=active)
    node.get_logger().info('obstacles: nvblox fused map + LiDAR')
    return FusedObstacles(node, get_pose, active=active)


class FusedObstacles:
    def __init__(self, node, get_pose, active=True):
        self.node, self.get_pose = node, get_pose
        self.cells = np.zeros((0, 2))     # occupied cell centres, odom
        self.res = 0.025
        self.updated = 0.0                # time of the last grid
        self.info_t = 0.0                 # time a depth frame last went to nvblox
        self.frames = 0
        self.sub = self.info_sub = None
        self.set_active(active)

    def set_active(self, on: bool) -> None:
        if on and self.sub is None:
            self.sub = self.node.create_subscription(
                OccupancyGrid, GRID, self._grid, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
            self.info_sub = self.node.create_subscription(Header, PASSED, self._info, 10)
        elif not on and self.sub is not None:
            self.node.destroy_subscription(self.sub)
            self.node.destroy_subscription(self.info_sub)
            self.sub = self.info_sub = None

    def _info(self, _):
        self.info_t = time.time()

    def _grid(self, m):
        info = m.info
        g = np.frombuffer(m.data, dtype=np.int8).reshape(info.height, info.width)
        ii, jj = np.nonzero(g >= OCC)
        r = info.resolution
        self.cells = np.stack([info.origin.position.x + (jj + 0.5) * r,
                               info.origin.position.y + (ii + 0.5) * r], 1)
        self.res = r
        self.updated = time.time()
        self.frames += 1

    def age(self):
        """Seconds since the view was last fresh (inf: never)."""
        if not self.updated or not self.info_t:
            return float('inf')
        return time.time() - min(self.updated, self.info_t)

    @property
    def mem(self):
        """N x 5 like DepthObstacles.mem (x, y, z, t, hits), for publishing."""
        c = self.cells
        if not len(c):
            return np.zeros((0, 5))
        return np.column_stack([c, np.full(len(c), 0.1), np.full(len(c), self.updated),
                                np.full(len(c), MIN_HITS)])

    def points_base(self, pose=None):
        """N x 2 obstacle points in base_link: every occupied cell's corners
        within RANGE, minus those deep inside the rover's own outline."""
        pose = pose if pose is not None else self.get_pose()
        c = self.cells
        if pose is None or not len(c):
            return np.zeros((0, 2))
        d = c - np.array(pose[:2])
        near = (np.abs(d[:, 0]) < RANGE) & (np.abs(d[:, 1]) < RANGE)
        d = d[near]
        h = self.res / 2 - 0.001
        d = np.vstack([d + [sx * h, sy * h] for sx in (-1, 1) for sy in (-1, 1)])
        cs, sn = math.cos(pose[2]), math.sin(pose[2])
        b = np.stack([d[:, 0] * cs + d[:, 1] * sn, -d[:, 0] * sn + d[:, 1] * cs], 1)
        own = ((b[:, 0] < OWN_FRONT - OWN_INSET) & (b[:, 0] > -OWN_REAR + OWN_INSET)
               & (np.abs(b[:, 1]) < OWN_SIDE - OWN_INSET))
        return b[~own]
