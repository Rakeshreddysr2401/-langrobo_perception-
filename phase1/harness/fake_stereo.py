#!/usr/bin/env python3
"""fake_stereo.py -- a synthetic D555 IR pair, to test visual odometry with no camera.

A random-textured wall Z metres ahead; the camera slides sideways at V m/s (+x of
the camera = to the right), so the true motion is known exactly. Publishes what
the D555 driver publishes and vo_isaac.launch.py reads:

    /camera/camera0/infra{1,2}/image_rect_raw   mono8 896x504
    /camera/camera0/infra{1,2}/camera_info      (infra2 carries the baseline in P[3])

    python3 fake_stereo.py [--z 2.0] [--v 0.10] [--hz 30] [--seconds 20]

Check: /vo/odom should move V m/s along the camera's x (base frame: -y).
Use a private ROS domain (not 0): this is not the robot.
First used 2026-10-18 for the Isaac ROS 4.6 check (a scratch copy); kept here from 2.0 on.
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

W, H = 896, 504
FX = FY = 430.0          # close to the D555 IR intrinsics at 896x504
CX, CY = W / 2, H / 2
BASELINE = 0.05          # m (the D555's is ~5 cm)
NS = '/camera/camera0'


def texture(seed=1, size=(1400, 4000)):
    """Blurred noise + blobs: plenty of corners at every scale, no repetition."""
    rng = np.random.default_rng(seed)
    t = rng.random(size, dtype=np.float32)
    k = np.ones(5, np.float32) / 5
    t = np.apply_along_axis(lambda r: np.convolve(r, k, 'same'), 1, t)
    t = np.apply_along_axis(lambda c: np.convolve(c, k, 'same'), 0, t)
    t = (t - t.min()) / (t.max() - t.min())
    return (t * 255).astype(np.uint8)


class FakeStereo(Node):
    def __init__(self, a):
        super().__init__('fake_stereo')
        self.a = a
        self.tex = texture()
        # wall pixel scale: one texture pixel = 1 image pixel at depth Z
        self.m_per_px = a.z / FX
        self.pub = {}
        for cam in ('infra1', 'infra2'):
            self.pub[cam] = (
                self.create_publisher(Image, f'{NS}/{cam}/image_rect_raw', qos_profile_sensor_data),
                self.create_publisher(CameraInfo, f'{NS}/{cam}/camera_info', qos_profile_sensor_data))
        self.t0 = time.monotonic()
        self.n = 0
        self.create_timer(1.0 / a.hz, self.tick)

    def info(self, cam, stamp):
        m = CameraInfo()
        m.header.stamp = stamp
        m.header.frame_id = f'camera0_{cam}_optical_frame'
        m.width, m.height = W, H
        m.distortion_model = 'plumb_bob'
        m.d = [0.0] * 5
        m.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        tx = -FX * BASELINE if cam == 'infra2' else 0.0
        m.p = [FX, 0.0, CX, tx, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
        return m

    def view(self, x_cam):
        """The wall seen from camera x position x_cam (metres)."""
        off = int(round(x_cam / self.m_per_px)) + 200
        y0 = (self.tex.shape[0] - H) // 2
        return np.ascontiguousarray(self.tex[y0:y0 + H, off:off + W])

    def tick(self):
        t = time.monotonic() - self.t0
        if t > self.a.seconds:
            raise SystemExit
        x = self.a.v * t
        stamp = self.get_clock().now().to_msg()
        for cam, xc in (('infra1', x), ('infra2', x + BASELINE)):
            img = Image()
            img.header.stamp = stamp
            img.header.frame_id = f'camera0_{cam}_optical_frame'
            img.height, img.width, img.encoding, img.step = H, W, 'mono8', W
            img.data = self.view(xc).tobytes()
            pi, pc = self.pub[cam]
            pi.publish(img)
            pc.publish(self.info(cam, stamp))
        self.n += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--z', type=float, default=2.0)
    ap.add_argument('--v', type=float, default=0.10)
    ap.add_argument('--hz', type=float, default=30.0)
    ap.add_argument('--seconds', type=float, default=20.0)
    a = ap.parse_args()
    rclpy.init()
    n = FakeStereo(a)
    try:
        rclpy.spin(n)
    except SystemExit:
        pass
    print(f'published {n.n} stereo frames in {a.seconds:.0f} s; true motion {a.v * a.seconds:.3f} m along camera +x')
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
