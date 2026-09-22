#!/usr/bin/env python3
"""Forward clearance from the DEPTH camera, whose frame is known-good.

The lidar cannot be trusted for this yet -- its yaw is the thing being
calibrated, so "which beams point forward" is exactly the open question. The
D555 sits on the front face pointing +x along base_link and that has been
stable since Phase 1, so it is the honest source for "is it safe to drive".
"""
import numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

TOPIC = "/camera/camera0/depth/image_rect_raw"

def main():
    rclpy.init(); n = Node("clearance"); got = []
    n.create_subscription(Image, TOPIC, got.append, qos_profile_sensor_data)
    end = n.get_clock().now().nanoseconds + int(10e9)
    while len(got) < 5 and n.get_clock().now().nanoseconds < end:
        rclpy.spin_once(n, timeout_sec=0.3)
    if not got:
        print("      no depth image"); return 1
    m = got[-1]
    d = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width).astype(np.float32) / 1000.0
    d[d <= 0.05] = np.nan
    h, w = d.shape
    # central band: the strip the rover would drive into
    band = d[int(h*0.35):int(h*0.75), int(w*0.30):int(w*0.70)]
    valid = band[np.isfinite(band)]
    print(f"      depth {w}x{h}, centre band {band.size} px, {valid.size} valid")
    if valid.size < 200:
        print("      too few valid depth pixels to call it safe"); return 1
    for q in (1, 5, 25, 50):
        print(f"        {q:2d}th pct : {np.percentile(valid, q):.2f} m")
    print(f"      NEAREST 1% of the forward band: {np.percentile(valid,1):.2f} m")
    n.destroy_node(); rclpy.shutdown(); return 0

raise SystemExit(main())
