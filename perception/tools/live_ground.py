#!/usr/bin/env python3
"""Live position test, run INSIDE the rover container (it only reads; it never drives).
Used 2026-11-20/21 to check the frame memory's positions (Pi docs/plans/FRAME_MEMORY.md):
  docker exec rover bash -lc "source /opt/ros/lyrical/setup.bash; cd /logs && python3 /opt/rover/perception/tools/live_ground.py grab"

  live_ground.py grab               save the current colour photo (/logs/live/frame.jpg + frame.json with
                                    its camera stamp) and ask pixel_to_goal to freeze that moment's depth
  live_ground.py ground boxes.json  boxes.json = {"stamp_ns": N, "boxes": {"helmet": [x0,y0,x1,y1], ...}}
                                    -> pixel_to_goal places each box on THAT photo's depth + pose;
                                    prints ahead/left from the LiDAR centre (and from base_link)
"""
import json
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import String

LIDAR_X = 0.1342          # description/params.yaml: the C1 is 13.4 cm ahead of base_link, on the centre line
OUT = '/logs/live'


class N(Node):
    def __init__(self):
        super().__init__('live_ground')
        self.img = self.info = None
        self.results = {}
        self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self._img, 2)
        self.create_subscription(CameraInfo, '/camera/camera0/color/camera_info', self._info, 2)
        self.create_subscription(String, '/vision/pixel_result', self._res, 10)
        self.snap = self.create_publisher(PointStamped, '/vision/pixel_snapshot', 5)
        self.query = self.create_publisher(PointStamped, '/vision/pixel_query', 5)

    def _img(self, m):
        self.img = m

    def _info(self, m):
        self.info = m

    def _res(self, m):
        j = json.loads(m.data)
        self.results[j.get('id')] = j


def spin_until(node, cond, t):
    end = time.time() + t
    while time.time() < end and not cond():
        rclpy.spin_once(node, timeout_sec=0.1)


rclpy.init()
n = N()
os.makedirs(OUT, exist_ok=True)
if sys.argv[1] == 'grab':
    spin_until(n, lambda: n.img is not None and n.info is not None, 10)
    m = n.img
    p = PointStamped()
    p.header.stamp = m.header.stamp
    for _ in range(3):
        n.snap.publish(p)
        spin_until(n, lambda: False, 0.2)
    open(f'{OUT}/frame.jpg', 'wb').write(bytes(m.data))
    st = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
    json.dump({'stamp_ns': st, 'info_w': n.info.width, 'info_h': n.info.height}, open(f'{OUT}/frame.json', 'w'))
    print(f'grabbed {OUT}/frame.jpg stamp {st}, camera_info {n.info.width}x{n.info.height}')
else:
    req = json.load(open(sys.argv[2]))
    spin_until(n, lambda: n.query.get_subscription_count() > 0, 10)   # first message is lost before discovery
    spin_until(n, lambda: False, 0.5)
    st = req['stamp_ns']
    for name, b in req['boxes'].items():
        q = PointStamped()
        q.header.stamp.sec, q.header.stamp.nanosec = st // 10**9, st % 10**9
        q.header.frame_id = f'{name};box={",".join(str(round(v)) for v in b)}'
        q.point.x, q.point.y = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
        n.query.publish(q)
        spin_until(n, lambda: name in n.results, 6)
        r = n.results.get(name)
        if not r or not r.get('ok'):
            print(f'{name:10s} FAILED: {r and r.get("reason")}')
            continue
        rel = r['relative']
        ahead_l = rel['forward_m'] - LIDAR_X
        side = rel['left_m']
        print(f"{name:10s} from LiDAR: ahead {ahead_l * 100:5.0f} cm  {'left ' if side >= 0 else 'right'} "
              f"{abs(side) * 100:4.0f} cm   (depth {r['depth_m']:.2f} m, at_capture={r.get('at_capture')})")
    json.dump(n.results, open(f'{OUT}/results.json', 'w'), indent=1)
rclpy.shutdown()
