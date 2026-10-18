"""vo_isaac.launch.py — the packaged Isaac ROS 4.6 cuVSLAM on the D555's IR pair.

Replaces the pyCuVSLAM wheel (vo_node.py) from orin-nav:1.2 on. Isaac ROS 4.4's
isaac_ros_visual_slam was a Thor build (5280 SVE instructions; SIGILL on this
Orin); 4.6's has none and carries sm_87 GPU code. Measured 2026-10-18 on a
synthetic stereo feed: 30 Hz, 0.10 m/s commanded = 0.10 m/s tracked, 25 % of a
core (vo_node: 26 %).

This node TRACKS and publishes /vo/odom (remapped below). nodes/vo_isaac.py,
started next to it, owns the rest of what the rover reads: the optical frames
it tracks in, /vo/status and the guarded map -> odom. So both TF publishers
here are off -- fusion2 owns odom -> base_link and vo_isaac.py the map -> odom
correction.

    ros2 launch vo_isaac.launch.py [ground:=true]
    (SLAM correction on/off is vo_isaac.py's slam param, not this file's)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

CAM_NS = '/camera/camera0'


def generate_launch_description():
    ground = LaunchConfiguration('ground')
    as_bool = lambda s: PythonExpression(["'", s, "'.lower() == 'true'"])

    node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'num_cameras': 2,
            'min_num_images': 2,
            # Published by vo_isaac.py under camera0_link with vo_node's axis
            # swap and camera_info's baseline -- NOT the driver's infra TF,
            # whose infra2 sign is the opposite of REP-104 (vo_node.py).
            'camera_optical_frames': ['vslam_left_optical', 'vslam_right_optical'],
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            # image_rect_raw is already rectified by the driver
            'rectified_images': True,
            'enable_image_denoising': False,
            # enable_sync:=false: the IR pair is near- but not bit-identical in
            # time; vo_node allowed 5 ms of pairing slop, as does this default
            'sync_matching_threshold_ms': 5.0,
            'image_qos': 'SENSOR_DATA',
            'publish_odom_to_base_tf': False,
            'publish_map_to_odom_tf': False,
            # ALWAYS ON, whatever ./rover's SLAM says. The landmark count -- the
            # health signal fusion2 weights VO by and drops it under 20 -- is
            # only published by the SLAM backend: with it off, landmarks_cloud
            # and observations_cloud are silent (measured 2026-10-18), and a
            # landmark count of 0 would make fusion2 ignore VO entirely. Cost:
            # ~5 % of a core. Whether SLAM's correction is USED is vo_isaac.py's
            # slam param: it publishes map -> odom only with slam:=true.
            'enable_localization_n_mapping': True,
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': False,
            # A ground rover. Off by default to match vo_node, which had no such
            # option for odometry -- try ground:=true against LiDAR truth.
            'enable_ground_constraint_in_odometry': as_bool(ground),
            'enable_ground_constraint_in_slam': True,
        }],
        remappings=[
            ('visual_slam/image_0', f'{CAM_NS}/infra1/image_rect_raw'),
            ('visual_slam/camera_info_0', f'{CAM_NS}/infra1/camera_info'),
            ('visual_slam/image_1', f'{CAM_NS}/infra2/image_rect_raw'),
            ('visual_slam/camera_info_1', f'{CAM_NS}/infra2/camera_info'),
            # odom -> base_link already, vo_node's contract: published from C++
            # rather than re-published through Python (vo_isaac.py, CPU)
            ('visual_slam/tracking/odometry', '/vo/odom'),
        ])

    return LaunchDescription([
        DeclareLaunchArgument('ground', default_value='false'),
        ComposableNodeContainer(
            name='vslam_container', namespace='', package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[node], output='screen'),
    ])
