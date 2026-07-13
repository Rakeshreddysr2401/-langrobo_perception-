"""Republish the sim camera onto the real robot's camera topics.

The real robot's camera_node (Brio webcam, Jetson ai_stack) publishes
  /camera/color/image_raw             sensor_msgs/Image, bgr8  → target_node (YOLO)
  /camera/color/image_raw/compressed  CompressedImage, jpeg    → brain snapshots
In sim the equivalents are rover_sim's /cam_1 streams; this relay makes the
sim indistinguishable to both consumers. Gazebo publishes rgb8, target_node
decodes bgr8 blindly, so the raw path swaps channels. Not used in the real
profile, where camera_node owns these topics.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class SimCameraRelay(Node):

    def __init__(self):
        super().__init__('sim_camera_relay')
        self.declare_parameter('input_compressed', '/cam_1/image/compressed')
        self.declare_parameter('input_raw', '/cam_1/color/image_raw')
        self.declare_parameter('output_compressed', '/camera/color/image_raw/compressed')
        self.declare_parameter('output_raw', '/camera/color/image_raw')
        self._pub_c = self.create_publisher(
            CompressedImage, self.get_parameter('output_compressed').value, 5)
        self._pub_r = self.create_publisher(
            Image, self.get_parameter('output_raw').value, 5)
        self.create_subscription(
            CompressedImage, self.get_parameter('input_compressed').value,
            self._pub_c.publish, 5)
        self.create_subscription(
            Image, self.get_parameter('input_raw').value, self._relay_raw, 5)

    def _relay_raw(self, msg: Image):
        if msg.encoding == 'rgb8':
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, 3)
            msg.data = rgb[:, ::-1].tobytes()
            msg.encoding = 'bgr8'
        self._pub_r.publish(msg)


def main():
    rclpy.init()
    node = SimCameraRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
