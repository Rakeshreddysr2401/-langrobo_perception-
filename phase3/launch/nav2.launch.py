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

THE ROVER CANNOT TURN IN PLACE (TODO 14)
    Every rotate-in-place behaviour is disabled in nav2.yaml. Most importantly
    RPP runs with use_rotate_to_heading:false, and the Spin recovery is removed.
    Without that nav2 would command a rotation the rover cannot execute, sit
    still, and time out -- which looks like a planner bug and is not.
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
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[CONFIG]),
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
