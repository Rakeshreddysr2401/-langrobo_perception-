#!/usr/bin/env python3
"""nav2 — click a goal, the rover plans a route and drives there.

WHAT THIS NEEDS THAT ALREADY EXISTS
    /odom + TF odom->base_link      fusion2 (M3, since 2026-09-26)
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

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node

CONFIG = '/opt/rover3/config/nav2.yaml'

# Order matters: the costmaps must be up before the servers that read them.
LIFECYCLE = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
    'collision_monitor',
]


# ONE DIES, ALL STOP, since 2026-09-26: planner_server segfaulted (exit -11)
# after live footprint_padding changes and nothing noticed -- the global
# costmap lives in that process, so RViz showed a frozen map while the rest of
# nav2 ran. nav2's own respawn (respawn=True + attempt_respawn_reconnection)
# was tried and left the lifecycle manager stuck mid-reset: planner
# unconfigured, controller active, navigator inactive. So any node exiting
# now ends this launch, and phase3/launch/nav2_supervise.sh (./rover nav)
# starts nav2 again from scratch, which always comes up clean.
# Do not change the footprint live: restart nav2.
STOP_ALL = dict(on_exit=[Shutdown(reason='a nav2 node exited')])

# A STACK TRACE ON EVERY CRASH, since 2026-09-27 (NAV_PLAN.md N1). nav2 has
# segfaulted (exit -11) at least five times in two days; each time the dump
# went to the host's apport and was lost, so every "cause" in nav2.yaml is a
# correlation. backward_ros's libbackward.so installs a signal handler when
# loaded: the trace of the crashing thread lands in /tmp/nav.log, right
# before nav2_supervise.sh's restart line. Costs nothing until a crash.
BACKWARD = '/opt/ros/jazzy/lib/libbackward.so'
STOP_ALL['additional_env'] = {'LD_PRELOAD': BACKWARD} if os.path.exists(BACKWARD) else {}


def generate_launch_description():
    nodes = [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', **STOP_ALL, parameters=[CONFIG],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', **STOP_ALL, parameters=[CONFIG]),
        # behavior_server ALSO goes through the smoother. It publishes to
        # `cmd_vel` by default, which on this rover is the wheels -- so every
        # recovery was a step change straight into the PID, bypassing the very
        # node added to prevent that. Harmless while the only recovery was a
        # 0.10 m/s reverse; not harmless now that Spin is back and commands
        # 1.5 rad/s from a standstill, which is exactly the slip that corrupts
        # the odometry nav2 is steering by. Remapped 2026-09-11, TODO 40.
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', **STOP_ALL, parameters=[CONFIG],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', **STOP_ALL, parameters=[CONFIG]),
        # The smoother sits between the controller and the wheels, so the
        # controller publishes cmd_vel_nav and only smoothed output reaches
        # /cmd_vel. Without it a step change in commanded velocity goes straight
        # to the PID, and on a skid-steer that is how you get wheel slip -- which
        # corrupts the very odometry nav2 is steering by.
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', **STOP_ALL, parameters=[CONFIG],
             # -> turn_shaper.py -> /cmd_vel: the smoothed command is in REAL
             # turn rates; turn_shaper converts them for this skid-steer.
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel_smoothed')]),
        # THE LAST LINE, independent of the map (SENSOR_FUSION_PLAN M5,
        # NAV_PLAN N7, 2026-09-28): cmd_vel_smoothed -> collision_monitor ->
        # cmd_vel_turn -> turn_shaper -> /cmd_vel. "approach" caps the speed so
        # the rover's real outline, moved along the CURRENT command (turns and
        # reversing included), stays >= time_before_collision from any LiDAR
        # point. Open floor: full speed. A wall ahead: it slows, smoothly, by
        # itself. It reads /scan directly, so a costmap that is wrong or late
        # cannot make the rover fast into something the LiDAR can see.
        # goal_exec and reach's escape publish /cmd_vel directly and keep
        # their own swept checks.
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen', **STOP_ALL, parameters=[CONFIG]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen', **STOP_ALL,
             parameters=[{'use_sim_time': False,
                          'autostart': True,
                          'node_names': LIFECYCLE}]),
    ]
    return LaunchDescription(nodes)
