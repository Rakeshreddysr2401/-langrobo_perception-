#!/usr/bin/env python3
"""depth_gate.py — depth frames to nvblox only while the rover is not turning.

    /camera/camera0/depth/image_rect_raw  ->  /camera/camera0/depth/image_still

turn_gate.py says why (frames painted mid-turn smeared the map 3-10 cm).
Frames are handled as raw bytes: the header stamp is read out of the CDR
(4-byte encapsulation, then int32 sec, uint32 nanosec), and the pixels of a
frame that passes are cleaned of stereo speckle in place (depth_speckle.py
says why: blobs at random ranges on plain surfaces painted false obstacles;
DEPTH_SPECKLE_PX=0 turns that off). Started by `./rover map` before nvblox,
which is remapped to image_still.
"""
import os
import struct
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parent))
import depth_speckle  # noqa: E402
from turn_gate import TurnGate  # noqa: E402

SRC = '/camera/camera0/depth/image_rect_raw'
DST = '/camera/camera0/depth/image_still'
INFO = '/camera/camera0/depth/camera_info'
SPECKLE_PX = int(os.environ.get('DEPTH_SPECKLE_PX', depth_speckle.MIN_PATCH_PX))
PASSED = '/depth_gate/passed'     # one small Header per frame passed: "nvblox got a view" (fused_obstacles.py)
# frames handed to nvblox per second. 30 -> 10 on 2026-11-14: the speckle filter costs ~9 ms a frame and at
# 30 Hz took ~46 % of a core on the 15 W Jetson; 10 Hz is 3.5 cm of travel per frame at nav2's 35 cm/s top
# speed, and every reader of PASSED wants >= 3-5 Hz. Spaced by the frames' own stamps. 0 = every frame.
MAP_HZ = float(os.environ.get('DEPTH_MAP_HZ', '10'))
REPORT_S = 30.0


class DepthGate(Node):
    def __init__(self):
        super().__init__('depth_gate')
        self.gate = TurnGate(self)
        # depth 5: a frame is ~800 KB in fragments; history 1 lost every frame
        # mid-reassembly (depth_obstacles.py, 2026-09-26)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Image, DST, qos)
        self.pub_passed = self.create_publisher(Header, PASSED, 10)
        self.create_subscription(Image, SRC, self._raw, qos_profile_sensor_data, raw=True)
        self.fx = None                      # from the camera's own camera_info (the speckle filter works in disparity)
        self.info_sub = self.create_subscription(CameraInfo, INFO, self._info, qos_profile_sensor_data)
        self.cleaned = self.removed = self.odd = self.skipped = 0
        self.t_last = -1e9                  # stamp of the last frame passed on
        self.t_report = time.time()
        self.get_logger().info(f'depth gate up: {SRC} -> {DST} while |wz| <= {self.gate.wz_max} rad/s; '
                               f'speckle patches under {SPECKLE_PX} px removed' + (' (OFF)' if SPECKLE_PX <= 0 else '') + f'; at most {MAP_HZ:g} Hz')

    def _info(self, m):
        if m.k[0] > 0:
            self.fx = float(m.k[0])
            self.destroy_subscription(self.info_sub)

    def _raw(self, data):
        sec, nsec = struct.unpack_from('<iI', data, 4)
        t = sec + nsec * 1e-9
        if t < self.t_last - 1.0:          # the camera restarted with an earlier clock: start over
            self.t_last = -1e9
        if MAP_HZ > 0 and t - self.t_last < 1.0 / MAP_HZ - 0.005:
            self.skipped += 1               # (before the gate and the filter: a skipped frame costs nothing)
        elif self.gate.still(t):
            self.t_last = t
            if self.fx is not None:         # (the first frames before camera_info pass as they are)
                data, n = depth_speckle.clean_cdr(data, self.fx, SPECKLE_PX)
                self.cleaned += 1; self.removed += max(n, 0); self.odd += n < 0
            self.pub.publish(data)
            h = Header()
            h.stamp.sec, h.stamp.nanosec = sec, nsec
            self.pub_passed.publish(h)
        if time.time() - self.t_report > REPORT_S:
            self.t_report = time.time()
            self.get_logger().info(f'depth gate: {self.gate.counts()}, {self.skipped} skipped for {MAP_HZ:g} Hz; speckle: {self.cleaned} frames cleaned, '
                                   f'{self.removed // max(self.cleaned, 1)} px removed per frame'
                                   + (f', {self.odd} of unexpected layout passed untouched' if self.odd else '')
                                   + ('' if self.fx else ' -- NO camera_info yet: frames pass unfiltered'))


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
