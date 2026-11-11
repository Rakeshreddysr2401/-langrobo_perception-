#!/usr/bin/env python3
"""replay_nvblox_helper.py — the two small jobs replay_nvblox.sh needs inside its container.

    gyro   /odom's turn rate -> /gyro/base (sensor_msgs/Imu), stamped as the odom message: a bag
           has no gyro topic, and without one the turn gate lets every frame through
    snap   at TIMES seconds after the first /odom of the replay, save the newest nvblox grid and
           ghost-cleared grid to /out/grid_<seconds>.npz (cells, resolution, origin, pose)
"""
import math
import os
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu


def gyro(n):
    pub = n.create_publisher(Imu, '/gyro/base', qos_profile_sensor_data)

    def cb(m):
        g = Imu()
        g.header.stamp = m.header.stamp
        g.header.frame_id = 'base_link'
        g.angular_velocity.z = m.twist.twist.angular.z
        pub.publish(g)
    n.create_subscription(Odometry, '/odom', cb, 50)


def snap(n):
    times = sorted(float(t) for t in os.environ.get('TIMES', '').split())
    st = {'t0': None, 'pose': None, 'grids': {}}

    def odom(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        st['t0'] = st['t0'] or t
        q = m.pose.pose.orientation
        st['pose'] = (m.pose.pose.position.x, m.pose.pose.position.y,
                      math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        while times and t - st['t0'] >= times[0]:
            tw = times.pop(0)
            out = {'pose': np.array(st['pose']), 't': t, 't_rel': tw}
            for name, g in st['grids'].items():
                out[name] = np.frombuffer(bytes(g.data), np.int8).reshape(g.info.height, g.info.width)
                out[name + '_res'] = g.info.resolution
                out[name + '_origin'] = np.array([g.info.origin.position.x, g.info.origin.position.y])
            np.savez_compressed(f'/out/grid_{int(tw):04d}.npz', **out)
            print(f'saved grid_{int(tw):04d}.npz  grids {sorted(st["grids"])}  '
                  + '  '.join(f'{k}: {int((np.frombuffer(bytes(g.data), np.int8) >= 50).sum())} occupied' for k, g in st['grids'].items()), flush=True)
    rel = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    tl = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    n.create_subscription(Odometry, '/odom', odom, 50)
    n.create_subscription(OccupancyGrid, '/nvblox_node/static_occupancy_grid', lambda m: st['grids'].__setitem__('nvblox', m), tl)
    n.create_subscription(OccupancyGrid, '/ghost_clear/grid', lambda m: st['grids'].__setitem__('ghost', m), rel)


def main():
    rclpy.init()
    n = rclpy.create_node('replay_' + sys.argv[1])
    {'gyro': gyro, 'snap': snap}[sys.argv[1]](n)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
