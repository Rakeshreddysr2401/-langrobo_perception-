#!/usr/bin/env python3
"""reach.py — send a goal to reach_node and follow it (./rover reach).

    ./rover reach X Y [DEG]         relative to the rover now
    add --odom / --map for an absolute goal. Ctrl-C cancels.

reach_node keeps at it -- nav2, goal_exec's exact finish, and on failure a
look around and a precise pass or a wait -- until it is there
(phase3/nodes/reach_node.py). RViz's 2D Goal Pose goes the same way.
"""
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Empty, String


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    a = [v for v in sys.argv[1:] if not v.startswith('--')]
    if len(a) < 2:
        sys.exit(__doc__)
    x, y = float(a[0]), float(a[1])
    th = math.radians(float(a[2])) if len(a) > 2 else None
    frame = 'map' if '--map' in sys.argv else ('odom' if '--odom' in sys.argv else 'rel')
    rclpy.init()
    n = Node('reach_cli')
    st, got = {}, []
    n.create_subscription(Odometry, '/odom', lambda m: st.update(o=m), 10)
    n.create_subscription(String, '/reach/status', lambda m: got.append(json.loads(m.data)), 10)
    pub = n.create_publisher(PoseStamped, '/reach/goal', 10)
    cancel = n.create_publisher(Empty, '/reach/cancel', 10)
    end = time.time() + 10
    while time.time() < end and ('o' not in st or pub.get_subscription_count() == 0):
        rclpy.spin_once(n, timeout_sec=0.1)
    if 'o' not in st:
        sys.exit('  no /odom: ./rover fused')
    if pub.get_subscription_count() == 0:
        sys.exit('  reach_node is not running: ./rover nav starts it')
    p = PoseStamped()
    if frame == 'rel':
        o = st['o'].pose.pose
        c = yaw_of(o.orientation)
        gx = o.position.x + math.cos(c) * x - math.sin(c) * y
        gy = o.position.y + math.sin(c) * x + math.cos(c) * y
        gth = c + (th if th is not None else math.atan2(y, x))
        p.header.frame_id = 'odom'
    else:
        gx, gy, gth = x, y, th if th is not None else 0.0
        p.header.frame_id = frame
    p.pose.position.x, p.pose.position.y = gx, gy
    p.pose.orientation.z, p.pose.orientation.w = math.sin(gth / 2), math.cos(gth / 2)
    print(f'  goal {p.header.frame_id}: ({gx:+.3f}, {gy:+.3f}, {math.degrees(gth):+.1f} deg). THIS DRIVES THE ROVER. Ctrl-C cancels.')
    pub.publish(p)
    t0 = time.time()
    seen = 0
    try:
        while True:
            rclpy.spin_once(n, timeout_sec=0.1)
            for s in got[seen:]:
                print(f'  {time.time() - t0:5.1f}s  ' + ', '.join(f'{k} {v}' for k, v in s.items() if k != 't'))
                if 'result' in s:
                    return 0 if s['result'] == 'reached' else 1
            seen = len(got)
    except KeyboardInterrupt:
        cancel.publish(Empty())
        rclpy.spin_once(n, timeout_sec=0.3)
        print('  => cancelled')
        return 1


if __name__ == '__main__':
    sys.exit(main())
