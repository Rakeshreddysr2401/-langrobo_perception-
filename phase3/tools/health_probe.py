#!/usr/bin/env python3
"""Every layer's topic rate, in ONE pass (~WINDOW s), as one JSON line.

    python3 health_probe.py [window_s]

`./rover status` runs `ros2 topic hz` once per topic, one after another: 14
topics x 12 s is over two minutes, too slow for anything that must keep
watching. This subscribes to all of them at once, raw (no message decoding, so
images cost nothing), and counts arrivals. Also reports whether /cmd_vel is
non-zero, so a caller can avoid restarting things while the rover drives.

Output: {"window": 6.0, "hz": {"/scan": 10.0, ...}, "moving": false}
A topic nobody publishes reports 0.0 (its type is unknown: it cannot be
subscribed to until a publisher appears; it is retried while we wait).
"""
import json
import sys
import threading
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

CAM = '/camera/camera0'
TOPICS = [
    f'{CAM}/infra1/image_rect_raw', f'{CAM}/depth/image_rect_raw',
    '/vo/odom', '/gyro/base', '/scan', '/wheel_state', '/odom', '/map',
    '/ghost_clear/grid', '/global_costmap/costmap', '/local_costmap/costmap',
    '/depth_gate/passed', '/goal_exec/obstacles',
    '/camera/color/image_raw/compressed', '/vision/people',
]


def main():
    window = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
    rclpy.init()
    node = rclpy.create_node('health_probe')
    count = {t: 0 for t in TOPICS}
    subs = {}
    t_sub = {}
    moving = {'v': False, 'seen': False}

    def hit(t):
        def cb(_):
            count[t] += 1
        return cb

    def on_cmd(msg):
        from_msg = abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z)
        moving['v'] = moving['v'] or from_msg > 0.01
        moving['seen'] = True

    from geometry_msgs.msg import Twist
    node.create_subscription(Twist, '/cmd_vel', on_cmd, 10)

    def attach():
        known = dict(node.get_topic_names_and_types())
        for t in TOPICS:
            if t in subs or t not in known:
                continue
            try:
                cls = get_message(known[t][0])
                subs[t] = node.create_subscription(cls, t, hit(t), qos_profile_sensor_data, raw=True)
                t_sub[t] = time.time()
            except Exception:
                pass

    t0 = time.time()
    # spin_once handles ONE callback per call and under-counts the 100+ Hz
    # topics; spin in a thread and only look for new topics here.
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    while time.time() - t0 < window + 2.0:
        attach()
        time.sleep(0.2)
    # Each topic's rate is counted from the moment IT was subscribed. The first
    # message of a topic is dropped from the count only by discovery timing, so
    # the slowest topics (a 0.5 Hz map) read a little low in a short window.
    now = time.time()
    out = {'window': round(window, 1),
           'hz': {t: round(count[t] / max(now - t_sub[t], 1.0), 2) if t in t_sub else 0.0 for t in TOPICS},
           'moving': moving['v']}
    print(json.dumps(out))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
