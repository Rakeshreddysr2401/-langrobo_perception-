#!/usr/bin/env python3
"""cuvgl_relay.py — the D555 IR pair for NVIDIA's cuVGL (ISAAC_ROS_5_UPGRADE.md B3), re-framed.

realsense2_camera stamps infra2's camera_info with INFRA1's frame id (the twin copies it), and
its camera0_infra2 frame has the opposite baseline sign to REP-104 (phase1/launch/vo_isaac.launch.py).
cuVSLAM is given vo_isaac.py's vslam_left_optical / vslam_right_optical frames for that reason;
cuVGL matches images to camera infos BY FRAME ID ("Camera info for camera0_infra2_optical_frame
is not loaded yet", 2026-10-25) and its map was built in the vslam_* frames. So this republishes

    /camera/camera0/infra{1,2}/image_rect_raw  -> /cuvgl/{left,right}/image          frame vslam_{left,right}_optical
    /camera/camera0/infra{1,2}/camera_info     -> /cuvgl/{left,right}/camera_info    same frames

unchanged otherwise (same stamps, same bytes). Cost: two 448x252 mono copies at 30 Hz.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

CAM = '/camera/camera0'
SIDES = (('infra1', 'left'), ('infra2', 'right'))


class Relay(Node):
    def __init__(self):
        super().__init__('cuvgl_relay')
        for src, side in SIDES:
            frame = f'vslam_{side}_optical'
            for kind, typ in (('image_rect_raw', Image), ('camera_info', CameraInfo)):
                out = 'image' if kind == 'image_rect_raw' else 'camera_info'
                pub = self.create_publisher(typ, f'/cuvgl/{side}/{out}', qos_profile_sensor_data)

                def cb(m, pub=pub, frame=frame):
                    m.header.frame_id = frame
                    pub.publish(m)
                self.create_subscription(typ, f'{CAM}/{src}/{kind}', cb, qos_profile_sensor_data)
        self.get_logger().info('cuvgl_relay: IR pair -> /cuvgl/{left,right}/{image,camera_info} in vslam_* frames')


def main():
    rclpy.init()
    rclpy.spin(Relay())


if __name__ == '__main__':
    main()
