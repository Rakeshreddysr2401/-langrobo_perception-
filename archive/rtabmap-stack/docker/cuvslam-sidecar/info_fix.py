#!/usr/bin/env python3
"""Relay infra2 camera_info with the P-matrix Tx sign corrected.
The D555 DDS driver exports Tx=+fx*B; ROS stereo convention (and cuVSLAM)
needs Tx=-fx*B for the right camera."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class Fix(Node):
    def __init__(self):
        super().__init__('infra2_info_fix')
        self.pub = self.create_publisher(CameraInfo, '/camera/camera0/infra2/camera_info_fixed', 10)
        self.create_subscription(CameraInfo, '/camera/camera0/infra2/camera_info', self.cb, 10)
    def cb(self, m):
        p = list(m.p)
        if p[3] > 0:
            p[3] = -p[3]
        m.p = p
        self.pub.publish(m)

rclpy.init()
rclpy.spin(Fix())
