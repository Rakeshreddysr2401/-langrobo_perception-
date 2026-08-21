#!/usr/bin/env python3
"""Draw the map in the terminal, with the rover on it.

RViz is the real view, but it lives on another machine and needs its own setup.
This renders the same occupancy grid as text, right where you are, so you can
hold it up against the room and see whether the walls are where the walls are.

    #   obstacle          .   floor the camera has seen
    (blank) unknown       R   the rover, > < ^ v showing which way it faces

North is up and the grid is drawn in the odom frame, so the map rotates as the
rover's start orientation dictates -- not as the room does.
"""
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                       HistoryPolicy, qos_profile_sensor_data)
from nav_msgs.msg import OccupancyGrid, Odometry

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)
VOLATILE = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST)


class View(Node):
    def __init__(self, topic):
        super().__init__('map_view')
        # nvblox publishes VOLATILE, nav2's costmaps publish TRANSIENT_LOCAL,
        # and picking the wrong one silently receives nothing while the topic
        # looks perfectly alive. Subscribe both ways and take whichever answers.
        # One of the two always logs an incompatible-QoS warning; that is this
        # working as intended, not a fault.
        import rclpy.logging
        rclpy.logging.set_logger_level('map_view', rclpy.logging.LoggingSeverity.ERROR)
        for q in (VOLATILE, LATCHED):
            self.create_subscription(OccupancyGrid, topic, self._m, q)
        self.create_subscription(Odometry, '/odom', self._o, qos_profile_sensor_data)
        self.grid = None
        self.pose = None

    def _m(self, m):
        self.grid = m

    def _o(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)


def render(g, pose, cols=110, rows=44):
    w, h, res = g.info.width, g.info.height, g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    d = g.data

    # Squeeze the grid into the terminal. Characters are about twice as tall as
    # they are wide, so x is sampled half as coarsely to keep the room's shape.
    sx = max(1, math.ceil(w / cols))
    sy = max(1, math.ceil(h / (rows * 2)))
    s = max(sx, sy)

    out = []
    for ry in range(h - 1, -1, -s * 2):
        line = []
        for rx in range(0, w, s):
            occ = free = unk = 0
            for yy in range(ry, max(-1, ry - s * 2), -1):
                for xx in range(rx, min(w, rx + s)):
                    v = d[yy * w + xx]
                    if v < 0:
                        unk += 1
                    elif v >= 50:
                        occ += 1
                    else:
                        free += 1
            line.append('#' if occ else ('.' if free else ' '))
        out.append(line)

    if pose is not None:
        px, py, yaw = pose
        cx = int((px - ox) / res / s)
        cy = int((h - 1 - (py - oy) / res) / (s * 2))
        if 0 <= cy < len(out) and 0 <= cx < len(out[0]):
            deg = (math.degrees(yaw) + 360) % 360
            out[cy][cx] = ('>' if deg < 45 or deg >= 315 else
                           '^' if deg < 135 else
                           '<' if deg < 225 else 'v')
    return '\n'.join(''.join(r) for r in out)


def main():
    topic = '/nvblox_node/static_occupancy_grid'
    once = False
    for a in sys.argv[1:]:
        if a == '--once':
            once = True
        else:
            topic = a

    rclpy.init()
    n = View(topic)
    # Wait for BOTH the map and the pose. Waiting only for the map raced: the
    # grid arrives first, the rover marker is silently omitted, and the map
    # looks like it has no robot on it.
    end = time.time() + 15
    while (n.grid is None or n.pose is None) and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.grid is None:
        print(f'  nothing on {topic} in 15 s')
        return
    if n.pose is None:
        print('  note: no /odom, so the rover is not marked on the map\n')

    try:
        while True:
            for _ in range(10):
                rclpy.spin_once(n, timeout_sec=0.05)
            g = n.grid
            body = render(g, n.pose)
            occ = sum(1 for v in g.data if v >= 50)
            free = sum(1 for v in g.data if 0 <= v < 50)
            r = g.info.resolution
            head = (f'  {topic}   {g.info.width}x{g.info.height} @ {r:.2f} m '
                    f'= {g.info.width * r:.1f} x {g.info.height * r:.1f} m   '
                    f'floor {free * r * r:.1f} m2   walls {occ * r * r:.1f} m2')
            if n.pose:
                head += f'   rover ({n.pose[0]:+.2f}, {n.pose[1]:+.2f})'
            if once:
                print(head + '\n')
                print(body)
                print('\n  # wall/obstacle   . floor seen   blank unknown   '
                      'R/^/v/</> the rover')
                break
            print('\033[H\033[J' + head + '\n')
            print(body)
            print('\n  # wall   . floor   blank unknown   Ctrl-C to stop')
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
