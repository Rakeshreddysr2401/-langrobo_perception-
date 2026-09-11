#!/usr/bin/env python3
"""nav2 — click a goal, the rover plans a route and drives there.

WHAT THIS NEEDS THAT ALREADY EXISTS
    /odom + TF odom->base_link      fusion_node (Phase 1b)
    /nvblox_node/static_map_slice   nvblox (Phase 2b)
    /cmd_vel -> wheels              the ESP32, proven end to end

NO map SERVER, NO AMCL
    Everything runs in the odom frame. There is no saved map to localize
    against yet -- that is Phase 2c/2d -- so this navigates within one session,
    relative to where the rover started. Enough to click a goal and drive to it;
    not enough to recognise a room tomorrow.

THE ROVER CAN TURN IN PLACE -- THE DOCSTRING THAT SAID OTHERWISE WAS STALE
    It could not, when this file was written (TODO 14), and everything that
    rotates was disabled on those grounds. TODO 14 was fixed and verified on the
    floor on 2026-08-22 at 65 deg/s. The controller was relaxed then; the
    RECOVERIES were not, and TODO 40 is the bill for that -- Spin is back, and
    nav2.yaml carries the rotational limits it needs to clear the scrub
    breakaway. Read nav2.yaml before changing any rotation value here.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

CONFIG = '/opt/rover3/config/nav2.yaml'

# Order matters: the costmaps must be up before the servers that read them.
LIFECYCLE = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
]


def generate_launch_description():
    common = {'use_sim_time': False}
    nodes = [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[CONFIG],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[CONFIG]),
        # behavior_server ALSO goes through the smoother. It publishes to
        # `cmd_vel` by default, which on this rover is the wheels -- so every
        # recovery was a step change straight into the PID, bypassing the very
        # node added to prevent that. Harmless while the only recovery was a
        # 0.10 m/s reverse; not harmless now that Spin is back and commands
        # 1.5 rad/s from a standstill, which is exactly the slip that corrupts
        # the odometry nav2 is steering by. Remapped 2026-09-11, TODO 40.
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[CONFIG],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[CONFIG]),
        # The smoother sits between the controller and the wheels, so the
        # controller publishes cmd_vel_nav and only smoothed output reaches
        # /cmd_vel. Without it a step change in commanded velocity goes straight
        # to the PID, and on a skid-steer that is how you get wheel slip -- which
        # corrupts the very odometry nav2 is steering by.
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=[CONFIG],
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': False,
                          'autostart': True,
                          'node_names': LIFECYCLE}]),
    ]
    return LaunchDescription(nodes)
