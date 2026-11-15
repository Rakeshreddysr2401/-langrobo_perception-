#!/usr/bin/env python3
"""Every layer's topic rate, in ONE pass (~WINDOW s), as one JSON line.

    python3 health_probe.py [window_s]
    python3 health_probe.py --watch [period_s] [life_s]   # the watchdog's: stays up, one line per period

`./rover status` used to run `ros2 topic hz` once per topic, one after another:
14 topics x 12 s, over two minutes, too slow for anything that must keep
watching (it uses this since 2026-10-18). This subscribes to all of them at once, raw (no message decoding, so
images cost nothing), and counts arrivals. Also reports whether /cmd_vel is
non-zero, so a caller can avoid restarting things while the rover drives.

Output: {"window": 6.0, "hz": {"/scan": 10.0, ...}, "moving": false}
A topic nobody publishes reports 0.0 (its type is unknown: it cannot be
subscribed to until a publisher appears; it is retried while we wait).

--watch is for ops/rover_watchdog.py. It used to run the one-shot form every
30 s: a new process and a new DDS participant each time (every node on the
Jetson and the Pi 5 answers a newcomer with its whole endpoint list), and 6 s
of two raw camera streams over UDP -- 30-40 % of a core on a Jetson that had
none to spare (2026-11-15). The watch form is ONE participant for its whole
life and counts the camera by its camera_info (WATCH_BY: published with every
frame, a few hundred bytes), reported under the image topic's name. It ends by
itself after life_s, so a copy nobody reads any more cannot stay for ever.
"""
import json
import sys
import threading
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor
from rosidl_runtime_py.utilities import get_message

CAM = '/camera/camera0'
TOPICS = [
    f'{CAM}/infra1/image_rect_raw', f'{CAM}/depth/image_rect_raw',
    '/vo/odom', '/gyro/base', '/scan', '/wheel_state', '/odom', '/map',
    '/ghost_clear/grid', '/global_costmap/costmap', '/local_costmap/costmap',
    '/depth_gate/passed', '/goal_exec/obstacles',
    '/camera/color/image_raw/compressed',
    # NOT /vision/people: subscribing to it WAKES detections_3d (YOLO sleeps without demand),
    # and the watchdog probes 6 s of every 30 -> YOLO awake ~16 s of every 30 for nobody
    # (33 % of a core on the real robot, 2026-10-25). The detect layer is checked by publisher.
]

# Topics whose type is fixed here, not read from the graph: a subscriber of the WRONG
# type adds a second type to the list (2026-11-01: an RViz without the compressed-image
# plugin subscribed to the camera JPEG as sensor_msgs/Image; this probe could copy it,
# and then count nothing).
TYPES = {'/camera/color/image_raw/compressed': 'sensor_msgs/msg/CompressedImage'}

# --watch only: count this small topic in place of the big one (same publisher, same rate).
WATCH_BY = {f'{CAM}/infra1/image_rect_raw': f'{CAM}/infra1/camera_info',
            f'{CAM}/depth/image_rect_raw': f'{CAM}/depth/camera_info'}


def main():
    watch = len(sys.argv) > 1 and sys.argv[1] == '--watch'
    args = [float(a) for a in sys.argv[(2 if watch else 1):]]
    window = args[0] if args else (30.0 if watch else 6.0)
    life = args[1] if watch and len(args) > 1 else 3600.0
    source = {t: (WATCH_BY.get(t, t) if watch else t) for t in TOPICS}     # reported name -> subscribed topic
    rclpy.init()
    node = rclpy.create_node('health_watch' if watch else 'health_probe')
    count = {t: 0 for t in TOPICS}
    subs = {}
    t_sub = {}
    moving = {'v': False}

    def hit(t):
        def cb(_):
            count[t] += 1
        return cb

    def on_cmd(msg):
        if abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z) > 0.01:
            moving['v'] = True

    from geometry_msgs.msg import Twist
    node.create_subscription(Twist, '/cmd_vel', on_cmd, 10)

    def attach():
        known = dict(node.get_topic_names_and_types())
        for t in TOPICS:
            src = source[t]
            if t in subs or src not in known:
                continue
            try:
                cls = get_message(TYPES.get(src) or known[src][0])
                subs[t] = node.create_subscription(cls, src, hit(t), qos_profile_sensor_data, raw=True)
                t_sub[t] = time.time()
            except Exception:
                pass

    def report(since):
        """Rates since `since` (or since a topic was subscribed, if later); the counts start again."""
        now = time.time()
        hz = {}
        for t in TOPICS:
            hz[t] = round(count[t] / max(now - max(t_sub[t], since), 1.0), 2) if t in t_sub else 0.0
            count[t] = 0
            if t in t_sub:
                t_sub[t] = min(t_sub[t], since)      # from now on: the whole period
        out = {'window': round(now - since, 1), 'hz': hz, 'moving': moving['v']}
        moving['v'] = False
        return out

    t0 = time.time()
    # spin_once handles ONE callback per call and under-counts the 100+ Hz
    # topics; spin in a thread and only look for new topics here.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    try:
        if watch:
            while time.time() - t0 < life:
                t1 = time.time()
                while time.time() - t1 < window:
                    attach()
                    time.sleep(1.0)
                print(json.dumps(report(t1)), flush=True)    # a closed pipe (reader gone) ends us here
        else:
            while time.time() - t0 < window + 2.0:
                attach()
                time.sleep(0.2)
            # Each topic's rate is counted from the moment IT was subscribed. The first
            # message of a topic is dropped from the count only by discovery timing, so
            # the slowest topics (a 0.5 Hz map) read a little low in a short window.
            out = report(t0)
            out['window'] = round(window, 1)
            print(json.dumps(out), flush=True)
    except BrokenPipeError:
        pass
    finally:
        # Stop callbacks before destroying their subscriptions/native handles.
        executor.shutdown()
        spin_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
