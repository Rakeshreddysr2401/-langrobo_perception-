#!/usr/bin/env python3
"""Nav2 1.5 finite-Range compatibility; raw /sonar/* remains unchanged.

The installed collision monitor rejects REP117 positive infinity, retaining its
previous echo. Only its private inputs replace valid +inf by the sensor's maximum
range. This conservatively inserts a distant arc, never invents free near space
from a finite echo, and preserves the measurement timestamp and all metadata.
Remove the bridge when the installed monitor supports no-return observations.
"""
import copy
import math


def finite_no_echo(msg):
    """Leave finite/invalid observations untouched; normalize valid no-return."""
    if not (math.isinf(msg.range) and msg.range > 0):
        return msg
    if not (all(math.isfinite(v) for v in
                (msg.field_of_view, msg.min_range, msg.max_range))
            and 0 < msg.field_of_view <= math.pi
            and 0 <= msg.min_range < msg.max_range):
        return msg  # Nav2 must still reject malformed metadata.
    result = copy.deepcopy(msg)
    result.range = msg.max_range
    return result


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Range

    class Bridge(Node):
        def __init__(self):
            super().__init__('sonar_nav2_compat')
            self.pubs = {}
            for side in ('left', 'right'):
                self.pubs[side] = self.create_publisher(
                    Range, '/nav2_sonar/' + side, qos_profile_sensor_data)
                self.create_subscription(
                    Range, '/sonar/' + side,
                    lambda msg, side=side: self.pubs[side].publish(finite_no_echo(msg)),
                    qos_profile_sensor_data)
            self.get_logger().info(
                'Nav2 sonar compatibility: valid +inf -> max_range on /nav2_sonar/*; '
                'raw /sonar/* and capture timestamps unchanged')

    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
