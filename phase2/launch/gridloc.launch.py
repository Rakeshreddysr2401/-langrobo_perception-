"""gridloc.launch.py — NVIDIA's occupancy-grid localizer (Isaac ROS 5.0), the FAST first guess for
phase2/tools/relocalize.py. Plan: ISAAC_ROS_5_UPGRADE.md B2.

    ros2 launch gridloc.launch.py map:=/maps/last.yaml

/scan -> LaserScantoFlatScan -> OccupancyGridLocalizer, all under /gridloc. It answers ONE global
search per call of /gridloc/trigger_grid_search_localization on /gridloc/localization_result
(map frame of the given map). It never publishes /initialpose or TF: relocalize.py checks the
answer against the scan itself and decides. Measured on the twin (2026-10-25, map twin_home,
3 spots standing still): 4.0-4.5 cm, 0.6 s per answer once up (3.5 s to start); ours 5.5-7.2 s.
On the cramped real 3.5 m room it was WRONG (10-27 % of beams on walls) -- hence the check.

Needs orin-nav:2.1 (docker/2.0/Dockerfile.features). The map is read once at start: ./rover
lidar starts this on maps/last, ./rover slam uses it once and stops it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/maps/last.yaml'),
        ComposableNodeContainer(
            name='gridloc_container', namespace='gridloc', package='rclcpp_components',
            executable='component_container', output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='isaac_ros_pointcloud_utils',
                    plugin='nvidia::isaac_ros::pointcloud_utils::LaserScantoFlatScanNode',
                    name='scan_to_flatscan', namespace='gridloc',
                    remappings=[('scan', '/scan')]),
                ComposableNode(
                    package='isaac_ros_occupancy_grid_localizer',
                    plugin='nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode',
                    name='occupancy_grid_localizer', namespace='gridloc',
                    parameters=[map_yaml, {'loc_result_frame': 'map', 'map_yaml_path': map_yaml}]),
            ]),
    ])
