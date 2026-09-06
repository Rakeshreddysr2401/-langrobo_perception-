#!/usr/bin/env python3
"""image_bridge — republish the D555 colour stream as JPEG for the Pi 5 brain.

WHY THIS EXISTS
    The Pi 5's langrobo_core agent (agent_node.py) has a fixed subscription:

        /camera/color/image_raw/compressed   (sensor_msgs/CompressedImage)

    It feeds both look() (the VLM sees the current view on request) and
    approach_described_object's _vlm_locate (the VLM points at a described
    object in the frame). Nothing in this rover published that topic until
    now — `ros2 topic info` showed the Pi 5's subscription with
    "Publisher count: 0". The brain was never actually blind on a bad day;
    it was blind full stop, since before color was even enabled (see
    README.md 2026-09-06).

    Two mismatches with our real camera, both handled here rather than by
    renaming this rover's topics (which everything else - rover, TODO,
    README, ARCHITECTURE - depends on):
      1. namespace — ours is /camera/camera0/color/image_raw, not
         /camera/color/image_raw. Topic name below is DELIBERATELY the Pi5's
         name, not ours: do not "fix" it to match our convention without
         also changing agent_node.py's subscription.
      2. transport — the frozen orin-nav:1.1 image has no
         compressed_image_transport plugin (`ros2 run image_transport
         list_transports` shows only "raw"), so `image_transport republish`
         can't produce the /compressed topic. cv_bridge + cv2.imencode does
         the same job in ~15 lines without that plugin.

RESOLUTION IS WHATEVER THE DRIVER ACTUALLY PUBLISHES, READ LIVE — never
assumed. rgb_camera.color_profile:=424x240x15 in ./rover camera is silently
NOT honoured by this D555: the color stream comes out at 896x504, matching
the depth profile instead (confirmed via camera_info, 2026-09-06 — the same
"a launch argument is not proof that it happened" lesson as the IR emitter,
see rover's camera) case). Re-encoding whatever arrives sidesteps that bug
rather than depending on it being fixed.

Run:  python3 image_bridge.py
"""
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

# The VLM round-trip takes seconds; nothing needs the color stream's native
# rate here. Capping the encode+publish rate keeps this node cheap on a
# Jetson that is already tight on GPU/CPU (see JETSON_LOAD.md).
_MAX_HZ = 5.0
_JPEG_QUALITY = 80


class ImageBridge(Node):

    def __init__(self):
        super().__init__('image_bridge')
        self._bridge = CvBridge()
        self._min_period = 1.0 / _MAX_HZ
        self._last_pub = 0.0
        self._pub = self.create_publisher(
            CompressedImage, '/camera/color/image_raw/compressed', 1)
        self.create_subscription(
            Image, '/camera/camera0/color/image_raw', self._on_image, 1)
        self.get_logger().info(
            'image_bridge up: camera0/color/image_raw -> '
            '/camera/color/image_raw/compressed (for the Pi 5 brain)')

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self._last_pub < self._min_period:
            return
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warning(f'imgmsg_to_cv2 failed: {e}', throttle_duration_sec=5.0)
            return
        ok, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, _JPEG_QUALITY])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self._pub.publish(out)
        self._last_pub = now


def main():
    rclpy.init()
    node = ImageBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
