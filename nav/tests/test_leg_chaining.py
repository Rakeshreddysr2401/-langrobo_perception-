#!/usr/bin/env python3
"""test_leg_chaining.py — legs chain without a stop, and never two drivers at once (2026-11-13).

    python3 nav/tests/test_leg_chaining.py   (in the rover container: imports rclpy; no robot)

run_nav2 now returns at a leg's waypoint with nav2 still driving (the next goal pre-empts it in
motion). Every other way reach moves the rover -- /cmd_vel, goal_exec's goal / turn / pass -- and
every stop() must first end that nav2 goal.
"""
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
import reach_node as RN  # noqa: E402


class Handle:
    def __init__(self):
        self.cancels = 0

    def cancel_goal_async(self):
        self.cancels += 1


def make():
    rclpy.init()
    node = rclpy.create_node('test_leg_chaining')
    r = object.__new__(RN.Reach)          # no subscriptions, no nav2: only what is tested
    r.spin_for = lambda s: None
    sent = []
    r.cmd = node.create_publisher(Twist, '/test/cmd_vel', 1)
    r.ge_goal = node.create_publisher(PoseStamped, '/test/ge_goal', 1)
    r.ge_turn = node.create_publisher(PoseStamped, '/test/ge_turn', 1)
    r.pub = node.create_publisher(String, '/test/ge_pass', 1)
    for name in ('cmd', 'ge_goal', 'ge_turn', 'pub'):
        pub = getattr(r, name)
        orig = pub.publish
        pub.publish = (lambda o, n: (lambda m: (sent.append((n, r.nav_gh)), o(m))))(orig, name)
        pub.publish = r._settled(pub.publish)        # as Reach.__init__ wraps them
    return r, node, sent


def test_every_other_driver_ends_a_running_nav2_goal_first():
    r, node, sent = make()
    try:
        for name, msg in (('cmd', Twist()), ('ge_goal', PoseStamped()), ('ge_turn', PoseStamped()), ('pub', String())):
            h = Handle()
            r.nav_gh = h                                 # a leg's nav2 goal still driving
            getattr(r, name).publish(msg)
            assert h.cancels == 1, f'{name}: nav2 cancelled once before it ({h.cancels})'
            assert sent[-1] == (name, None), f'{name}: published only after nav2 was ended'
        h = Handle()
        r.nav_gh = h
        r.stop()
        assert h.cancels == 1 and r.nav_gh is None, 'stop() ends nav2 too'
        r.cmd.publish(Twist())                           # nothing running: nothing cancelled
        assert h.cancels == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    test_every_other_driver_ends_a_running_nav2_goal_first()
    print('ok   test_every_other_driver_ends_a_running_nav2_goal_first\nALL PASS')
