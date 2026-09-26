#!/usr/bin/env python3
"""navto.py — nav2 for the route, goal_exec for the exact finish (./rover navto).

    ./rover navto X Y [DEG]            relative to the rover now; nav2 only (±10 cm, ±14 deg)
    ./rover navto X Y DEG --exact      nav2 to the PRE-GOAL, then goal_exec to (x, y, θ) exactly
    add --odom / --map for an absolute goal

nav2 plans a route around what the costmaps know (nvblox + LiDAR, 5 cm margin)
and drives it with RPP; its goal checker is coarse by design. With --exact,
nav2's goal is the point 35 cm BEHIND the target on its heading line -- the
pre-goal goal_exec itself would use -- so nav2 ends roughly lined up, and
goal_exec does the straight final approach to ~1 cm (LOCALIZATION.md §12).
This is the split the VLM brain will use: route by nav2, finish by goal_exec.

Ctrl-C cancels whichever is running; both stop the wheels.
"""
import json
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty, String

PRE = 0.35        # m, goal_exec's pre-goal runway (GoalExec.L)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose_msg(frame, x, y, th):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
    return p


def main():
    a = [v for v in sys.argv[1:] if not v.startswith('--')]
    if len(a) < 2:
        sys.exit(__doc__)
    x, y = float(a[0]), float(a[1])
    th = math.radians(float(a[2])) if len(a) > 2 else None
    exact = '--exact' in sys.argv
    frame = 'map' if '--map' in sys.argv else ('odom' if '--odom' in sys.argv else 'rel')
    if exact and th is None:
        sys.exit('  --exact needs a heading: navto X Y DEG --exact')

    rclpy.init()
    n = Node('navto')
    st = {}
    n.create_subscription(Odometry, '/odom', lambda m: st.update(odom=m), 10)
    got = []
    n.create_subscription(String, '/goal_exec/status', lambda m: got.append(json.loads(m.data)), 10)
    ge_pub = n.create_publisher(PoseStamped, '/goal_exec/goal', 10)
    ge_cancel = n.create_publisher(Empty, '/goal_exec/cancel', 10)
    nav = ActionClient(n, NavigateToPose, 'navigate_to_pose')
    end = time.time() + 10
    while time.time() < end and 'odom' not in st:
        rclpy.spin_once(n, timeout_sec=0.1)
    if 'odom' not in st:
        sys.exit('  no /odom: ./rover fused')
    if not nav.wait_for_server(timeout_sec=10):
        sys.exit('  nav2 is not up: ./rover nav')

    # the goal, in odom (or map)
    if frame == 'rel':
        p = st['odom'].pose.pose
        cth = yaw_of(p.orientation)
        gx = p.position.x + math.cos(cth) * x - math.sin(cth) * y
        gy = p.position.y + math.sin(cth) * x + math.cos(cth) * y
        gth = cth + (th if th is not None else math.atan2(y, x))
        gframe = 'odom'
    else:
        gx, gy, gth, gframe = x, y, (th if th is not None else 0.0), frame
    # nav2's goal: the target itself, or with --exact the pre-goal on its line
    nx, ny = (gx - PRE * math.cos(gth), gy - PRE * math.sin(gth)) if exact else (gx, gy)
    print(f'  target {gframe}: ({gx:+.3f}, {gy:+.3f}, {math.degrees(gth):+.1f} deg)'
          + (f'   nav2 -> pre-goal ({nx:+.3f}, {ny:+.3f})' if exact else ''))
    print('  THIS DRIVES THE ROVER. Ctrl-C cancels.')

    goal = NavigateToPose.Goal()
    goal.pose = pose_msg(gframe, nx, ny, gth)
    goal.pose.header.stamp = n.get_clock().now().to_msg()
    fb = {}
    t0 = time.time()
    fut = nav.send_goal_async(goal, feedback_callback=lambda f: fb.update(d=f.feedback.distance_remaining))
    rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
    gh = fut.result()
    if gh is None or not gh.accepted:
        sys.exit('  nav2 rejected the goal')
    res = gh.get_result_async()
    last = 0
    try:
        while not res.done():
            rclpy.spin_once(n, timeout_sec=0.1)
            if time.time() - last > 2 and 'd' in fb:
                last = time.time()
                print(f'  {time.time() - t0:5.1f}s  nav2  {fb["d"]:.2f} m to go')
            if time.time() - t0 > 180:
                gh.cancel_goal_async()
                sys.exit('  nav2 took over 180 s; cancelled')
    except KeyboardInterrupt:
        gh.cancel_goal_async()
        rclpy.spin_once(n, timeout_sec=0.5)
        sys.exit('  => cancelled')
    status = res.result().status
    p = st['odom'].pose.pose
    err = math.hypot(nx - p.position.x, ny - p.position.y) if gframe == 'odom' else float('nan')
    print(f'  {time.time() - t0:5.1f}s  nav2  {"SUCCEEDED" if status == GoalStatus.STATUS_SUCCEEDED else f"status {status}"}'
          + (f'   {err * 100:.1f} cm from its goal' if err == err else ''))
    if status != GoalStatus.STATUS_SUCCEEDED:
        return 1
    if not exact:
        return 0

    # the exact finish
    end = time.time() + 5
    while time.time() < end and ge_pub.get_subscription_count() == 0:
        rclpy.spin_once(n, timeout_sec=0.1)
    if ge_pub.get_subscription_count() == 0:
        sys.exit('  goal_exec is not running (./rover navto starts it)')
    ge_pub.publish(pose_msg(gframe, gx, gy, gth))
    seen = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.1)
            for s in got[seen:]:
                if s['state'] == 'done':
                    print(f"  {time.time() - t0:5.1f}s  goal_exec {s['result']}: {s['why']}")
                    return 0 if s['result'] == 'reached' else 1
            seen = len(got)
            if time.time() - t0 > 300:
                ge_cancel.publish(Empty())
                sys.exit('  gave up; cancelled')
    except KeyboardInterrupt:
        ge_cancel.publish(Empty())
        rclpy.spin_once(n, timeout_sec=0.5)
        sys.exit('  => cancelled')


if __name__ == '__main__':
    sys.exit(main())
