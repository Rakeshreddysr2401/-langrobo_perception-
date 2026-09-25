#!/usr/bin/env python3
"""How much of the room has nvblox actually mapped?

A map topic publishing at 5 Hz proves the node is alive, not that it has seen
anything. This counts the cells: free, occupied, and still unknown. Driving
should make free and occupied grow and unknown shrink.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid

# nvblox publishes VOLATILE, not transient_local -- a latched subscription gets
# nothing and looks exactly like a node that is not running. And note the topic:
# /nvblox_node/static_map_slice is an nvblox_msgs/DistanceMapSlice, while the
# nav_msgs/OccupancyGrid that RViz and nav2 want is static_occupancy_grid.
QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST)


class Stats(Node):
    def __init__(self, topic):
        super().__init__('map_stats')
        self.create_subscription(OccupancyGrid, topic, self._m, QOS)
        self.m = None

    def _m(self, m):
        self.m = m


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/nvblox_node/static_occupancy_grid'
    rclpy.init()
    n = Stats(topic)
    end = time.time() + 15
    while n.m is None and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.m is None:
        print(f'  nothing on {topic} in 15 s')
        return

    m = n.m
    d = list(m.data)
    unknown = sum(1 for v in d if v < 0)
    free = sum(1 for v in d if 0 <= v < 50)
    occ = sum(1 for v in d if v >= 50)
    total = len(d)
    res = m.info.resolution
    print(f'\n  {topic}')
    print(f'    grid        {m.info.width} x {m.info.height} cells @ {res:.3f} m'
          f'   = {m.info.width * res:.1f} x {m.info.height * res:.1f} m')
    print(f'    origin      x {m.info.origin.position.x:+.2f}  '
          f'y {m.info.origin.position.y:+.2f}')
    print(f'    frame       {m.header.frame_id}')
    print()
    print(f'    unknown     {unknown:8d}  {100 * unknown / total:5.1f}%')
    print(f'    free        {free:8d}  {100 * free / total:5.1f}%   '
          f'= {free * res * res:.2f} m2 of floor seen')
    print(f'    occupied    {occ:8d}  {100 * occ / total:5.1f}%   '
          f'= {occ * res * res:.2f} m2 of obstacle')
    print()
    if free + occ == 0:
        print('    NOTHING MAPPED YET. The node is running but has integrated no')
        print('    geometry. Check that depth is arriving and that TF from the')
        print('    global frame to the depth optical frame resolves.')
    else:
        print('    Drive the rover and re-run: free and occupied should grow.')
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
