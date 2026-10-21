#!/usr/bin/env python3
"""sim_target_track.py — the real target_track_node, fed a rendered world over ROS.

    # in the rover container, a spare domain, the node running:
    python3 /opt/rover4/nodes/target_track_node.py &
    python3 /opt/rover4/tools/sim_target_track.py

Publishes what the robot would (test_target_lock.py's ray-cast world): raw
colour, aligned depth (16UC1 mm, 30 % holes), colour camera_info, image_bridge's
JPEG, and odom -> camera TF stamped like the frames. Then the Pi 5's part:
/vision/pixel_snapshot for a photo, and 3 s later (the VLM "thinking") a
/target/lock with that photo's stamp, the box, a first guess 25 cm off and the
photo's camera pose (as pixel_to_goal's reply carries it). The camera then
drives 1.4 m toward the object while weaving. PASS when /target/state says
"locked", then "tracking" on >= 90 % of the frames, and the point ends within
3 cm of the truth; and a cancel returns "idle".
"""
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

sys.path.insert(0, os.path.dirname(__file__))
import test_target_lock as W  # noqa: E402  (the renderer and the world)

FRAME = "camera_color_optical_frame"


def R_to_quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


class Sim(Node):
    def __init__(self):
        super().__init__("sim_target_track")
        self.pc = self.create_publisher(Image, "/camera/camera0/color/image_raw", 5)
        self.pd = self.create_publisher(Image, "/camera/camera0/aligned_depth_to_color/image_raw", 5)
        self.pi = self.create_publisher(CameraInfo, "/camera/camera0/color/camera_info", 5)
        self.pj = self.create_publisher(CompressedImage, "/camera/color/image_raw/compressed", 5)
        self.ps = self.create_publisher(PointStamped, "/vision/pixel_snapshot", 5)
        self.pl = self.create_publisher(String, "/target/lock", 5)
        self.px = self.create_publisher(String, "/target/cancel", 5)
        self.tfb = TransformBroadcaster(self)
        self.states = []
        self.create_subscription(String, "/target/state", lambda m: self.states.append(json.loads(m.data)), 50)
        self.rng = np.random.default_rng(1)

    def publish_frame(self, world, pose, jpeg=True):
        R, t = W.cam_pose(*pose)
        img, depth = W.render(world, (R, t))
        depth = W.noisy_depth(depth, self.rng)
        stamp = self.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, "odom", FRAME
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, t)
        q = R_to_quat(R)
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
        self.tfb.sendTransform(tf)
        ci = CameraInfo()
        ci.header.stamp, ci.header.frame_id = stamp, FRAME
        ci.width, ci.height = W.W, W.H
        ci.k = [float(v) for v in W.K.flatten()]
        m = Image()
        m.header = ci.header
        m.height, m.width, m.encoding, m.step = W.H, W.W, "bgr8", W.W * 3
        m.data = img.tobytes()
        d = Image()
        d.header = ci.header
        d.height, d.width, d.encoding, d.step = W.H, W.W, "16UC1", W.W * 2
        d.data = (depth * 1000).astype(np.uint16).tobytes()
        self.pi.publish(ci)
        # colour and depth share a stamp but arrive in EITHER order on the robot:
        # alternate, so a node pairing "colour + latest depth" fails here too
        self.k = getattr(self, "k", 0) + 1
        if self.k % 2:
            self.pd.publish(d); self.pc.publish(m)
        else:
            self.pc.publish(m); self.spin_for(0.01); self.pd.publish(d)
        if jpeg:
            j = CompressedImage()
            j.header, j.format = ci.header, "jpeg"
            j.data = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
            self.pj.publish(j)
        return stamp, (R, t)

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)


def main():
    rclpy.init()
    n = Sim()
    world = W.scene()
    obj = W.obj_face(2.5, 0.2)
    truth = W.centre(obj)
    fails = []

    def check(ok, what):
        print(("  PASS " if ok else "  FAIL ") + what)
        if not ok:
            fails.append(what)

    print("warming up (discovery, TF)...")
    for _ in range(30):
        n.publish_frame(world, (0, 0, 0))
        n.spin_for(1 / 15)
    # the Pi 5 takes a photo and says so
    stamp, (R, t) = n.publish_frame(world, (0, 0, 0))
    n.spin_for(0.05)
    snap = PointStamped()
    snap.header.stamp = stamp
    n.ps.publish(snap)
    box = W.box_of(obj, (R, t))
    # ... the VLM thinks for 3 s while the stream goes on
    for _ in range(45):
        n.publish_frame(world, (0, 0, 0))
        n.spin_for(1 / 15)
    guess = truth + np.array([0.20, -0.15, 0.0])
    lock = {"id": "t1", "stamp": stamp.sec + stamp.nanosec * 1e-9, "box": box, "label": "box",
            "object": {"x": guess[0], "y": guess[1], "z": guess[2]},
            "camera": {"t": t.tolist(), "q": list(R_to_quat(R))}}
    n.pl.publish(String(data=json.dumps(lock)))
    n.spin_for(0.5)
    check(any(s.get("state") == "locked" for s in n.states),
          f"locked on the 3 s old photo ({[s.get('state') for s in n.states][:3]})")
    n.states.clear()
    t0 = time.time()
    for k in range(60):                      # 4 s, 1.4 m, weaving
        n.publish_frame(world, (0.0233 * k, 0.0, math.radians(8) * math.sin(k / 6)), jpeg=(k % 3 == 0))
        n.spin_for(1 / 15)
    n.spin_for(0.3)
    tracked = [s for s in n.states if s.get("state") == "tracking"]
    check(len(n.states) >= 50, f"a state for (nearly) every frame: {len(n.states)} of 60")
    check(len(tracked) >= 0.9 * len(n.states), f"tracking {len(tracked)}/{len(n.states)}")
    wd = sum(1 for s in n.states if s.get("with_depth"))
    check(wd >= 0.9 * len(n.states), f"depth paired on {wd}/{len(n.states)} frames (colour first or depth first)")
    if n.states:
        e = float(np.linalg.norm(np.array(n.states[-1]["object"]) - truth))
        check(e < 0.03, f"25 cm first guess -> {e * 100:.1f} cm from the truth "
                        f"(from_lock_m {n.states[-1]['from_lock_m']}, sigma {n.states[-1]['sigma_m']})")
    n.px.publish(String(data="t1"))
    n.spin_for(0.5)
    check(any(s.get("state") == "idle" for s in n.states[-3:]), "cancel -> idle")
    print(f"\n({time.time() - t0:.1f} s of driving)")
    print("ALL PASS" if not fails else f"{len(fails)} FAILED")
    n.destroy_node()
    rclpy.shutdown()
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
