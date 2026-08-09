#!/usr/bin/env python3
"""Read the D555 reported depth at image centre — one shot, prints metres.

Used to check camera METRIC SCALE against a tape-measured wall distance:
    depth_probe.py [depth_topic]
Handles 16UC1 (millimetres) and 32FC1 (metres). Reports a robust median over a
central ROI so a single bad pixel doesn't dominate.
"""
import sys
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

TOPIC = sys.argv[1] if len(sys.argv) > 1 else "/camera/camera0/depth/image_rect_raw"


def main():
    rclpy.init()
    n = rclpy.create_node("depth_probe")
    br = CvBridge()
    got = {}

    def cb(m):
        if "img" in got:
            return
        img = br.imgmsg_to_cv2(m, desired_encoding="passthrough")
        got["img"] = img
        got["enc"] = m.encoding
        got["wh"] = (m.width, m.height)

    n.create_subscription(Image, TOPIC, cb, qos_profile_sensor_data)
    import time
    t0 = time.time()
    while time.time() - t0 < 6 and "img" not in got:
        rclpy.spin_once(n, timeout_sec=0.1)

    if "img" not in got:
        print(f"NO depth on {TOPIC}")
    else:
        img = got["img"].astype(np.float32)
        enc = got["enc"]
        w, h = got["wh"]
        # to metres
        scale = 0.001 if ("16" in enc or img.max() > 100) else 1.0
        z = img * scale
        cy, cx = h // 2, w // 2
        roi = z[cy - 15:cy + 15, cx - 15:cx + 15]
        valid = roi[(roi > 0.05) & (roi < 20.0)]
        med = float(np.median(valid)) if valid.size else float("nan")
        print(f"topic={TOPIC} enc={enc} {w}x{h} scale={scale}")
        print(f"CENTRE-ROI median depth = {med:.3f} m  (valid px {valid.size}/{roi.size})")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
