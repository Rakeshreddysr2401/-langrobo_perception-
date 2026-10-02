#!/usr/bin/env python3
"""depth_gate.py — depth frames to nvblox only while the rover is not turning.

    /camera/camera0/depth/image_rect_raw  ->  /camera/camera0/depth/image_still

turn_gate.py says why (frames painted mid-turn smeared the map 3-10 cm).
Frames pass as raw bytes, never decoded: only the header stamp is read, out
of the CDR (4-byte encapsulation, then int32 sec, uint32 nanosec). Started
by `./rover map` before nvblox, which is remapped to image_still.
"""
import struct
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
from turn_gate import TurnGate  # noqa: E402

SRC = '/camera/camera0/depth/image_rect_raw'
DST = '/camera/camera0/depth/image_still'
REPORT_S = 30.0


class DepthGate(Node):
    def __init__(self):
        super().__init__('depth_gate')
        self.gate = TurnGate(self)
        # depth 5: a frame is ~800 KB in fragments; history 1 lost every frame
        # mid-reassembly (depth_obstacles.py, 2026-09-26)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Image, DST, qos)
        self.create_subscription(Image, SRC, self._raw, qos_profile_sensor_data, raw=True)
        self.t_report = time.time()
        self.get_logger().info(f'depth gate up: {SRC} -> {DST} while |wz| <= {self.gate.wz_max} rad/s')

    def _raw(self, data):
        sec, nsec = struct.unpack_from('<iI', data, 4)
        if self.gate.still(sec + nsec * 1e-9):
            self.pub.publish(data)
        if time.time() - self.t_report > REPORT_S:
            self.t_report = time.time()
            self.get_logger().info(f'depth gate: {self.gate.counts()}')


def main():
    rclpy.init()
    n = DepthGate()
    try:
        from rclpy.experimental import EventsExecutor
        ex = EventsExecutor()
    except ImportError:
        from rclpy.executors import SingleThreadedExecutor
        ex = SingleThreadedExecutor()
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
