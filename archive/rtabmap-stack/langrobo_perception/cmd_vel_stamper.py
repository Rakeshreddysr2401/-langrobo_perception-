"""Relay Nav2's unstamped /cmd_vel to the rover's TwistStamped command topic.

The mecanum drive controller (sim and real rover alike) consumes
geometry_msgs/TwistStamped and brakes on commands older than its
cmd_vel_timeout, so the stamp must come from the active clock
(sim time when use_sim_time is set).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelStamper(Node):

    def __init__(self):
        super().__init__('cmd_vel_stamper')
        self.declare_parameter('output_topic', '/mecanum_drive_controller/cmd_vel')
        self.declare_parameter('frame_id', 'base_link')
        output_topic = self.get_parameter('output_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        self._pub = self.create_publisher(TwistStamped, output_topic, 10)
        self._sub = self.create_subscription(Twist, 'cmd_vel', self._relay, 10)

    def _relay(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame_id
        out.twist = msg
        self._pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
