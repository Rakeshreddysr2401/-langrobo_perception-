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
frame, a few hundred bytes), reported under the image topic's name. The fast
topics (WATCH_FAST, ~360 messages a second together) are listened to for
FAST_S of every period only: subscribed all the time they cost this Python
process 36 % of a core under the full stack (measured the same day); the slow
ones stay subscribed and are counted over the whole period. It ends by itself
after life_s, so a copy nobody reads any more cannot stay for ever.
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
# --watch only: 10 Hz and faster -> sampled FAST_S per period, not subscribed all the time.
WATCH_FAST = {f'{CAM}/infra1/image_rect_raw', f'{CAM}/depth/image_rect_raw', '/vo/odom', '/gyro/base',
              '/scan', '/wheel_state', '/odom', '/depth_gate/passed'}
FAST_S = 4.0


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

    def attach(only=None):
        known = dict(node.get_topic_names_and_types())
        for t in TOPICS:
            src = source[t]
            if t in subs or src not in known or (only is not None and t not in only):
                continue
            try:
                cls = get_message(TYPES.get(src) or known[src][0])
                subs[t] = node.create_subscription(cls, src, hit(t), qos_profile_sensor_data, raw=True)
                t_sub[t] = time.time()
            except Exception:
                pass

    def rates(topics, since):
        """Rates since `since` (or since a topic was subscribed, if later); those counts start again."""
        now = time.time()
        hz = {}
        for t in topics:
            hz[t] = round(count[t] / max(now - max(t_sub[t], since), 1.0), 2) if t in t_sub else 0.0
            count[t] = 0
            if t in t_sub:
                t_sub[t] = min(t_sub[t], since)      # from now on: the whole period
        return hz

    def sample_fast():
        """Listen to the fast topics for FAST_S, then let them go again."""
        fast = [t for t in TOPICS if t in WATCH_FAST]
        t1 = time.time()
        while time.time() - t1 < FAST_S:
            attach(only=WATCH_FAST)
            time.sleep(0.5)
        hz = rates(fast, t1)
        for t in fast:
            if t in subs:
                node.destroy_subscription(subs.pop(t))
                del t_sub[t]
        return hz

    t0 = time.time()
    # spin_once handles ONE callback per call and under-counts the 100+ Hz
    # topics; spin in a thread and only look for new topics here.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    try:
        if watch:
            slow = [t for t in TOPICS if t not in WATCH_FAST]
            while time.time() - t0 < life:
                t1 = time.time()
                while time.time() - t1 < window - FAST_S:
                    attach(only=slow)
                    time.sleep(1.0)
                hz = sample_fast()
                hz.update(rates(slow, t1))
                # 'silent': a publisher exists and nothing arrived. Either it is stuck -- or THIS
                # participant never matched it (seen 2026-11-15 on a probe born while nav2 was
                # restarting: camera, lidar and maps read 0 for minutes, a fresh probe read them
                # all). The watchdog starts a fresh probe once to tell the two apart.
                silent = [t for t in TOPICS if hz[t] == 0.0 and node.count_publishers(source[t]) > 0]
                out = {'window': round(time.time() - t1, 1), 'hz': {t: hz[t] for t in TOPICS},
                       'moving': moving['v'], 'silent': silent}
                moving['v'] = False
                print(json.dumps(out), flush=True)           # a closed pipe (reader gone) ends us here
        else:
            while time.time() - t0 < window + 2.0:
                attach()
                time.sleep(0.2)
            # Each topic's rate is counted from the moment IT was subscribed. The first
            # message of a topic is dropped from the count only by discovery timing, so
            # the slowest topics (a 0.5 Hz map) read a little low in a short window.
            print(json.dumps({'window': round(window, 1), 'hz': rates(TOPICS, t0), 'moving': moving['v']}), flush=True)
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
