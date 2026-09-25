#!/usr/bin/env python3
"""goto.py — send one exact goal to goal_exec and watch it (./rover goto).

    ./rover goto X Y [DEG]          relative to where the rover is now (default)
    ./rover goto X Y DEG --odom     in the odom frame
    ./rover goto X Y DEG --map      in the map frame (SLAM)

Relative: X forward, Y left, DEG turn left, all from the current pose.
Prints every state change and the result; Ctrl-C cancels the goal (the node
stops the wheels).
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


def main():
    a = [x for x in sys.argv[1:] if not x.startswith('--')]
    if len(a) < 2:
        sys.exit(__doc__)
    x, y = float(a[0]), float(a[1])
    th = math.radians(float(a[2])) if len(a) > 2 else 0.0
    frame = 'map' if '--map' in sys.argv else ('odom' if '--odom' in sys.argv else 'rel')

    rclpy.init()
    n = Node('goto_client')
    pose, got = {}, []
    n.create_subscription(Odometry, '/odom', lambda m: pose.update(m=m), 10)
    n.create_subscription(String, '/goal_exec/status', lambda m: got.append(json.loads(m.data)), 10)
    pub = n.create_publisher(PoseStamped, '/goal_exec/goal', 10)
    cancel = n.create_publisher(Empty, '/goal_exec/cancel', 10)
    end = time.time() + 8
    while time.time() < end and ('m' not in pose or pub.get_subscription_count() == 0):
        rclpy.spin_once(n, timeout_sec=0.1)
    if 'm' not in pose:
        sys.exit('  no /odom: ./rover fused first')
    if pub.get_subscription_count() == 0:
        sys.exit('  goal_exec is not running (./rover goto starts it)')

    g = PoseStamped()
    if frame == 'rel':
        p = pose['m'].pose.pose
        cth = math.atan2(2 * (p.orientation.w * p.orientation.z), 1 - 2 * p.orientation.z ** 2)
        gx = p.position.x + math.cos(cth) * x - math.sin(cth) * y
        gy = p.position.y + math.sin(cth) * x + math.cos(cth) * y
        gth = cth + th
        g.header.frame_id = 'odom'
    else:
        gx, gy, gth = x, y, th
        g.header.frame_id = frame
    g.pose.position.x, g.pose.position.y = gx, gy
    g.pose.orientation.z, g.pose.orientation.w = math.sin(gth / 2), math.cos(gth / 2)
    print(f'  goal {g.header.frame_id}: ({gx:+.3f}, {gy:+.3f}, {math.degrees(gth):+.1f} deg)'
          + (f'   [relative {x:+.2f} m, {y:+.2f} m, {math.degrees(th):+.0f} deg]' if frame == 'rel' else ''))
    print('  THIS DRIVES THE ROVER. Ctrl-C cancels.')
    pub.publish(g)
    t0, seen = time.time(), 0
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.1)
            for s in got[seen:]:
                extra = f"  err {s.get('err_cm')} cm {s.get('err_deg')} deg" if 'err_cm' in s else ''
                paused = f"  PAUSED: {s['paused']}" if s.get('paused') else ''
                print(f"  {time.time() - t0:5.1f}s  {s['state']:6s} try {s.get('tries')}{extra}{paused}")
                if s['state'] == 'done':
                    print(f"  => {s['result']}: {s['why']}")
                    return 0 if s['result'] == 'reached' else 1
            seen = len(got)
            if time.time() - t0 > 180:
                cancel.publish(Empty())
                print('  => gave up after 180 s; cancelled')
                return 1
    except KeyboardInterrupt:
        cancel.publish(Empty())
        for _ in range(5):
            rclpy.spin_once(n, timeout_sec=0.05)
        print('  => cancelled')
        return 1


if __name__ == '__main__':
    sys.exit(main())
