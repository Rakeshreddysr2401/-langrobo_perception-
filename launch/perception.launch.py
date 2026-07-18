"""LangRobo Jetson perception bringup: nvblox mapping + Nav2.

Profiles (mode launch argument):
  sim  — consume the rover_sim Gazebo stand-in running on the dev laptop:
         depth from /cam_1/*, wheel-odom TF from the sim, sim time,
         odom-frame navigation (no SLAM: the sim camera has no stereo pair).
  real — RealSense D555 (Ethernet/DDS) + RTAB-Map + nvblox + Nav2 + the
         detections_3d YOLO node, wall clock. RTAB-Map provides
         map→odom→base_link with a PERSISTENT map database
         (/data/rtabmap.db), so saved locations stay valid across reboots;
         goals from the Pi5 arrive in `map`.
         The ESP32 rover consumes plain Twist on /cmd_vel (Nav2's native
         output) — the TwistStamped stamper is sim-only (mecanum controller).

WHY RTAB-MAP AND NOT cuVSLAM: NVIDIA's JetPack-7 (noble-jetpack) Isaac ROS
debs ship libcuvslam.so built for Thor-class ARMv9 CPUs — its static
initialiser SIGILLs on the Orin Nano's Cortex-A78AE (verified 2026-07-13 on
releases 4.3 and 4.4 via gdb). nvblox is open source and runs fine on the
GPU, so the split is: RTAB-Map (CPU, ~1 core — affordable with the voice
stack parked) for localization, nvblox (GPU) for reconstruction + costmaps.

TF NOTE (real): base_link→camera0_link is STATIC, measured with the pan/tilt
servos centred. The servo angles are deliberately NOT in TF — the visual
odometry tracks the camera and absorbs head pans as apparent base rotation;
publishing the joint too would double-count it. See JETSON_D555_SETUP.md §3
in the Pi5 repo.

LEGACY (2026-07-18): nothing launches this file anymore — its callers
run_perception_{sim,real}.sh were deleted; superseded by scripts/run_all.sh
+ the per-stage scripts. Kept for the rationale above; Phase 5 deletion
candidate (see SIM_REAL_PARITY.md §3 note).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('langrobo_perception')
    nvblox_config_base = os.path.join(
        get_package_share_directory('nvblox_examples_bringup'),
        'config', 'nvblox', 'nvblox_base.yaml')
    nvblox_config_sim = os.path.join(pkg_share, 'config', 'nvblox_sim.yaml')
    nvblox_config_real = os.path.join(pkg_share, 'config', 'nvblox_real.yaml')
    nav2_config_sim = os.path.join(pkg_share, 'config', 'nav2_sim.yaml')
    nav2_config_real = os.path.join(pkg_share, 'config', 'nav2_real.yaml')

    mode = LaunchConfiguration('mode')
    run_nav2 = LaunchConfiguration('run_nav2')
    use_sim_time = LaunchConfiguration('use_sim_time')

    is_sim = IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    is_real = IfCondition(PythonExpression(["'", mode, "' == 'real'"]))
    is_sim_nav2 = IfCondition(PythonExpression(
        ["'", mode, "' == 'sim' and '", run_nav2, "'.lower() == 'true'"]))
    is_real_nav2 = IfCondition(PythonExpression(
        ["'", mode, "' == 'real' and '", run_nav2, "'.lower() == 'true'"]))

    declare_mode = DeclareLaunchArgument(
        'mode', default_value='sim', choices=['sim', 'real'],
        description="Perception profile: Gazebo stand-in or D555 + RTAB-Map + nvblox + Nav2.")
    declare_run_nav2 = DeclareLaunchArgument(
        'run_nav2', default_value='True',
        description='Start Nav2 alongside nvblox.')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Sim profile follows the Gazebo /clock from the laptop. '
                    'Ignored by the real profile (always wall clock).')
    # Camera mount relative to base_link, measured with servos CENTRED.
    declare_cam_x = DeclareLaunchArgument(
        'cam_x', default_value='0.10', description='camera forward of base_link (m)')
    declare_cam_y = DeclareLaunchArgument(
        'cam_y', default_value='0.0', description='camera left of base_link (m)')
    declare_cam_z = DeclareLaunchArgument(
        'cam_z', default_value='0.25', description='camera above floor (m)')
    declare_cam_pitch = DeclareLaunchArgument(
        'cam_pitch', default_value='0.0', description='camera pitch (rad, +down)')
    declare_localization = DeclareLaunchArgument(
        'localization', default_value='False',
        description='RTAB-Map mode: False = mapping (grow /data/rtabmap.db), '
                    'True = localization-only against the existing database. '
                    'Map with it False first, then flip True for daily use.')

    # ══════════════════ SIM profile (unchanged behaviour) ══════════════════

    nvblox_node_sim = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=[
            nvblox_config_base,
            nvblox_config_sim,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('camera_0/depth/image', '/cam_1/depth/image_rect_raw'),
            ('camera_0/depth/camera_info', '/cam_1/depth/camera_info'),
        ],
        condition=is_sim,
    )

    # Sim stand-in for cuVSLAM: identity map->odom keeps the Pi5's map-frame
    # goal contract; cuVSLAM replaces this publisher in the real profile.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sim_map_to_odom',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=is_sim,
    )

    # Sim-only: the mecanum controller wants TwistStamped; the REAL rover's
    # ESP32 takes Nav2's plain Twist on /cmd_vel directly.
    cmd_vel_stamper = Node(
        package='langrobo_perception',
        executable='cmd_vel_stamper',
        name='cmd_vel_stamper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=is_sim_nav2,
    )

    # Sim stand-in for the camera snapshot feed; in the real profile
    # detections_3d publishes the compressed look() feed itself.
    sim_camera_relay = Node(
        package='langrobo_perception',
        executable='sim_camera_relay',
        name='sim_camera_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=is_sim,
    )

    nav2_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_config_sim,
            'use_sim_time': use_sim_time,
            'use_composition': 'False',
            'autostart': 'True',
            # MPPI controller_server has segfaulted post-goal on this build;
            # respawn lets the lifecycle manager recover instead of taking
            # the whole nav stack down permanently.
            'use_respawn': 'True',
        }.items(),
        condition=is_sim_nav2,
    )

    # ══════════════════ REAL profile (D555 + cuVSLAM) ══════════════════════

    # Rigid camera extrinsic at pan=0/tilt=0 — see TF NOTE in the docstring.
    base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera0',
        arguments=[
            '--x', LaunchConfiguration('cam_x'),
            '--y', LaunchConfiguration('cam_y'),
            '--z', LaunchConfiguration('cam_z'),
            '--pitch', LaunchConfiguration('cam_pitch'),
            '--frame-id', 'base_link', '--child-frame-id', 'camera0_link'],
        condition=is_real,
    )

    # RealSense D555 driver — plain wrapper launch, no emitter splitter (that
    # existed to feed cuVSLAM dot-free infra; RTAB-Map uses color, which the
    # emitter doesn't touch, so the emitter stays ON for best depth).
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('realsense2_camera'),
            'launch', 'rs_launch.py')),
        launch_arguments={
            'camera_name': 'camera0',
            # realsense-ros ≥4.58.2 defaults camera_namespace to 'camera',
            # giving /camera/camera0/… topics. Keep that default — passing ''
            # is a malformed CLI arg and the launch system ignores it.
            'depth_module.depth_profile': '848x480x30',
            'rgb_camera.color_profile': '848x480x30',
            'align_depth.enable': 'true',       # registered depth for RTAB-Map
            'pointcloud.enable': 'true',        # Nav2 collision monitor source
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
        }.items(),
        condition=is_real,
    )

    # RTAB-Map visual odometry: color+registered depth → odom→base_link TF.
    # Force3DoF: wheeled robot on a floor — kill roll/pitch/z drift.
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'approx_sync': True,
            'approx_sync_max_interval': 0.05,  # D555 DDS color/depth arrive ~33 ms apart
            'qos': 2,                       # SENSOR_DATA (live driver)
            'Reg/Force3DoF': 'true',
            'Odom/ResetCountdown': '1',     # auto-recover after tracking loss
        }],
        remappings=[
            ('rgb/image', '/camera/camera0/color/image_raw'),
            ('depth/image', '/camera/camera0/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/camera0/color/camera_info'),
        ],
        condition=is_real,
    )

    # RTAB-Map SLAM: map→odom TF + persistent map database. Saved locations
    # on the Pi5 (~/.langrobo/locations.json) are poses in THIS map frame —
    # the database makes them survive reboots.
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
            'subscribe_depth': True,
            'approx_sync': True,
            'qos_image': 2,
            'qos_camera_info': 2,
            'database_path': '/data/rtabmap.db',
            'Reg/Force3DoF': 'true',
            # RTAB-Map library params are STRING-typed ("true"/"false") — force
            # str so launch's yaml inference doesn't turn the text into a bool.
            'Mem/IncrementalMemory': ParameterValue(PythonExpression(
                ["'false' if '", LaunchConfiguration('localization'),
                 "'.lower() == 'true' else 'true'"]), value_type=str),
        }],
        remappings=[
            ('rgb/image', '/camera/camera0/color/image_raw'),
            ('depth/image', '/camera/camera0/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/camera0/color/camera_info'),
        ],
        condition=is_real,
    )

    nvblox_node_real = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=[
            nvblox_config_base,
            nvblox_config_real,
            {'use_sim_time': False},
        ],
        remappings=[
            # Raw (unaligned) depth + its own camera_info — independent of the
            # aligned stream RTAB-Map consumes.
            ('camera_0/depth/image', '/camera/camera0/depth/image_rect_raw'),
            ('camera_0/depth/camera_info', '/camera/camera0/depth/camera_info'),
        ],
        condition=is_real,
    )

    # YOLO 3D detections for the Pi5 brain (also serves the look() JPEG feed).
    detections_3d = Node(
        package='langrobo_perception',
        executable='detections_3d',
        name='detections_3d',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=is_real,
    )

    nav2_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_config_real,
            'use_sim_time': 'False',
            'use_composition': 'False',
            'autostart': 'True',
            'use_respawn': 'True',
        }.items(),
        condition=is_real_nav2,
    )

    return LaunchDescription([
        declare_mode,
        declare_run_nav2,
        declare_use_sim_time,
        declare_cam_x,
        declare_cam_y,
        declare_cam_z,
        declare_cam_pitch,
        declare_localization,
        # sim
        map_to_odom,
        sim_camera_relay,
        nvblox_node_sim,
        cmd_vel_stamper,
        nav2_sim,
        # real
        base_to_camera,
        realsense,
        rgbd_odometry,
        rtabmap_slam,
        nvblox_node_real,
        detections_3d,
        nav2_real,
    ])
