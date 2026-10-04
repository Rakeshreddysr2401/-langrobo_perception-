#!/usr/bin/env python3
"""ghost_clear_node.py — nvblox's grid with the marks of moved things erased.

    /nvblox_node/static_occupancy_grid  (nvblox)  ->  /ghost_clear/grid   (nav2, goal_exec, reach)
    /scan, /odom, /depth_gate/passed                   /ghost_clear/erased (RViz: what was erased)

The rule is ghost_clear.py (why, and what is never erased). ROVER_GHOST_CLEAR=off
passes the grid through unchanged -- the old behaviour, same topic.
"""
import array
import math
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

import ghost_clear as G

GRID_IN = '/nvblox_node/static_occupancy_grid'
GRID_OUT = '/ghost_clear/grid'
ERASED_OUT = '/ghost_clear/erased'
PASSED = '/depth_gate/passed'     # depth_gate.py: a frame just went to nvblox
LIVE_S = 0.5                      # s since the last such frame = the camera is mapping now
OCC = 50
ERASED_EVERY_S = 1.0              # the RViz view of what was erased: once a second is plenty


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class GhostClear(Node):
    def __init__(self):
        super().__init__('ghost_clear')
        self.on = os.environ.get('ROVER_GHOST_CLEAR', 'on') != 'off'
        self.g = G.GhostMap()
        self.pose, self.passed_t, self.last_grid_t, self.erased_t = None, 0.0, None, 0.0
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        rel = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, GRID_OUT, rel)
        self.pub_erased = self.create_publisher(OccupancyGrid, ERASED_OUT, rel)
        self.create_subscription(OccupancyGrid, GRID_IN, self._grid, rel)
        self.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(Header, PASSED, lambda _: setattr(self, 'passed_t', time.time()), 10)
        self.ms, self.n = 0.0, 0
        self.create_timer(10.0, self._report)
        self.get_logger().info(f'ghost_clear up: {GRID_IN} -> {GRID_OUT} '
                               f'({"erasing moved things" if self.on else "PASS-THROUGH (ROVER_GHOST_CLEAR=off)"})')

    def _odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _scan(self, m):
        if not self.on:
            return
        t0 = time.time()
        try:
            tr = self.buf.lookup_transform('odom', m.header.frame_id, Time()).transform
        except Exception:
            return
        r = np.asarray(m.ranges)
        a = m.angle_min + np.arange(r.size) * m.angle_increment
        ok = np.isfinite(r) & (r > 0.05) & (r < 12.0)
        th = yaw_of(tr.rotation)
        c, s = math.cos(th), math.sin(th)
        lx, ly = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        hits = np.stack([tr.translation.x + c * lx - s * ly, tr.translation.y + s * lx + c * ly], 1)
        self.g.on_scan((tr.translation.x, tr.translation.y), hits, time.time())
        self.ms += (time.time() - t0) * 1000.0
        self.n += 1

    def _grid(self, m):
        if not self.on:
            self.pub.publish(m)
            return
        now = time.time()
        info = m.info
        a = np.asarray(m.data, np.int8).reshape(info.height, info.width)
        jj, ii = np.nonzero(a >= OCC)
        res = info.resolution
        # global cell index of grid cell (ii, jj): nvblox's grid sits on whole cells
        gx = np.floor((info.origin.position.x + (ii + 0.5) * res) / res).astype(np.int64)
        gy = np.floor((info.origin.position.y + (jj + 0.5) * res) / res).astype(np.int64)
        self.g.on_grid(np.stack([gx, gy], 1), now)
        if self.pose is not None:
            dt = 0.0 if self.last_grid_t is None else min(now - self.last_grid_t, 1.0)
            self.g.on_pose(self.pose, now - self.passed_t <= LIVE_S, dt)
        self.last_grid_t = now
        erase = self.g.erase_mask(G.keys_of(gx, gy))
        b = a.copy()
        b[jj[erase], ii[erase]] = 0
        self.pub.publish(self._grid_msg(m, b))
        if now - self.erased_t >= ERASED_EVERY_S:
            self.erased_t = now
            e = np.full(a.shape, -1, np.int8)
            e[jj[erase], ii[erase]] = 100
            self.pub_erased.publish(self._grid_msg(m, e))

    @staticmethod
    def _grid_msg(like, cells):
        # array('b') from bytes, not .tolist(): a 400 x 450 grid as a Python
        # list five times a second was most of this node's CPU
        out = OccupancyGrid(header=like.header, info=like.info)
        out.data = array.array('b', cells.tobytes())
        return out

    def _report(self):
        if self.on:
            self.get_logger().info(
                f'marks {self.g.occ.size}, tall {len(self.g.tall)}, erased {len(self.g.cleared)}; '
                f'scan {self.ms / max(self.n, 1):.1f} ms avg over {self.n}')
        self.ms, self.n = 0.0, 0


def main():
    rclpy.init()
    n = GhostClear()
    # EventsExecutor, as the other rover nodes (gap_pass.Pass): the stock one
    # spent 2/3 of this node's time in wait_for_ready_callbacks (py-spy,
    # 2026-10-04). ROVER_EXECUTOR=single restores it.
    if os.environ.get('ROVER_EXECUTOR', 'events') == 'events':
        from rclpy.experimental import EventsExecutor
        ex = EventsExecutor()
    else:
        from rclpy.executors import SingleThreadedExecutor
        ex = SingleThreadedExecutor()
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
