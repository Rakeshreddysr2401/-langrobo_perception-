#!/usr/bin/env python3
"""virtual_person.py — a person who is not there, for floor-testing follow_node.

    python3 phase3/tools/virtual_person.py --ahead 1.6 --left 0.4 [--seconds 30]
        [--walk 0.2] [--topic /vision/people_virtual] [--force]

Fixes a point in odom from the rover's pose at start (ahead / left of it, m)
and publishes it on --topic at 5 Hz in the /vision/people format
(detections_3d.py), with dist and bearing from the LIVE pose -- exactly what
the follower would get from YOLO for someone standing there. --walk moves the
point away along the rover's starting heading (m/s). Outside the camera's
view (--fov, the D555's 87 deg) the person is not in the message, so the
follower's lost / re-acquire logic is exercised too.

Run follow_node on the same topic (follow_node ... -p people_topic:=<topic>)
so real YOLO output never mixes in.

WHY (2026-10-10). The follower's wheels, obstacle checks and pose-trust rules
need floor tests, the owner was asleep, and a test that needs a person
walking is neither repeatable nor possible alone. The REAL obstacle checks
still run: the follower ignores obstacle points within PERSON_R (0.4 m) of
its target -- that is the person's own body -- so this refuses a spot with
LiDAR points within --clear (0.6 m) of it, unless --force. A virtual person
standing inside a real chair would make the follower drive into the chair.
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

HALF_FOV = math.radians(87.0 / 2)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class VirtualPerson(Node):
    def __init__(self, a):
        super().__init__('virtual_person')
        self.a = a
        self.pose = None
        self.scan = None
        self.start = None
        self.t0 = None
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.pub = self.create_publisher(String, a.topic, 10)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m),
                                 qos_profile_sensor_data)

    def _odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def place(self):
        x, y, th = self.pose
        c, s = math.cos(th), math.sin(th)
        self.start = (x + c * self.a.ahead - s * self.a.left,
                      y + s * self.a.ahead + c * self.a.left, th)
        return self.start

    def nearest_scan_point(self, px, py):
        """Distance from (px, py) in odom to the nearest LiDAR return, via TF."""
        m = self.scan
        if m is None or not self.buf.can_transform('odom', m.header.frame_id, Time()):
            return None
        tr = self.buf.lookup_transform('odom', m.header.frame_id, Time()).transform
        th = yaw_of(tr.rotation)
        r = np.asarray(m.ranges)
        ang = m.angle_min + np.arange(r.size) * m.angle_increment
        ok = np.isfinite(r) & (r > 0.05) & (r < 8.0)
        xs = tr.translation.x + r[ok] * np.cos(ang[ok] + th)
        ys = tr.translation.y + r[ok] * np.sin(ang[ok] + th)
        if not xs.size:
            return None
        return float(np.min(np.hypot(xs - px, ys - py)))

    def tick(self):
        t = time.time()
        px, py, th0 = self.start
        walked = self.a.walk * (t - self.t0)
        px += math.cos(th0) * walked
        py += math.sin(th0) * walked
        x, y, yaw = self.pose
        dx, dy = px - x, py - y
        bearing = math.atan2(math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw))
        people = []
        if abs(bearing) <= HALF_FOV:
            people.append({"id": 1, "x": round(px, 3), "y": round(py, 3),
                           "vx": round(math.cos(th0) * self.a.walk, 2),
                           "vy": round(math.sin(th0) * self.a.walk, 2),
                           "dist": round(math.hypot(dx, dy), 3), "bearing": round(bearing, 3),
                           "conf": 0.9, "lost_s": 0.0, "box": [0, 0, 0, 0], "virtual": True})
        self.pub.publish(String(data=json.dumps({"frame": "odom", "stamp": t, "people": people})))
        return math.hypot(dx, dy), bearing, bool(people)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--ahead', type=float, required=True, help='m ahead of the rover at start')
    ap.add_argument('--left', type=float, default=0.0, help='m to its left at start')
    ap.add_argument('--walk', type=float, default=0.0, help='m/s away along the start heading')
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--topic', default='/vision/people_virtual')
    ap.add_argument('--clear', type=float, default=0.6, help='m of LiDAR-clear floor needed around the spot')
    ap.add_argument('--force', action='store_true')
    a = ap.parse_args()

    rclpy.init()
    n = VirtualPerson(a)
    deadline = time.time() + 5.0
    while (n.pose is None or n.scan is None) and time.time() < deadline:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.pose is None:
        sys.exit('no /odom -- is ./rover fused up?')
    px, py, _ = n.place()
    for _ in range(20):                       # a moment for TF
        rclpy.spin_once(n, timeout_sec=0.05)
    near = n.nearest_scan_point(px, py)
    where = f'({px:.2f}, {py:.2f}) odom, {a.ahead:.2f} m ahead / {a.left:+.2f} m left'
    if near is None:
        msg = f'virtual person at {where}: no LiDAR view of the spot'
        if not a.force:
            sys.exit(msg + ' -- refusing (use --force only if you have looked)')
        print(msg + ' (--force)')
    elif near < a.clear:
        msg = f'virtual person at {where}: a real LiDAR return is {near:.2f} m from it'
        if not a.force:
            sys.exit(msg + f' (< {a.clear} m) -- the follower would treat it as the person; refusing')
        print(msg + ' (--force)')
    else:
        print(f'virtual person at {where}; nearest LiDAR return {near:.2f} m from it')
    print(f'publishing on {a.topic} at 5 Hz for {a.seconds:.0f} s'
          + (f', walking {a.walk:.2f} m/s' if a.walk else ''))
    n.t0 = time.time()
    last_print = 0.0
    try:
        while time.time() - n.t0 < a.seconds:
            t_next = time.time() + 0.2
            d, b, seen = n.tick()
            if time.time() - last_print > 1.0:
                last_print = time.time()
                print(f'  t={time.time() - n.t0:5.1f}s  dist {d:4.2f} m  bearing {math.degrees(b):+5.0f} deg'
                      + ('' if seen else '  (out of view)'), flush=True)
            while time.time() < t_next:
                rclpy.spin_once(n, timeout_sec=max(0.0, t_next - time.time()))
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
