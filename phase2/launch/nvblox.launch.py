#!/usr/bin/env python3
"""nvblox — build the room as the rover drives.

WHAT IT DOES
    Depth images plus a pose become a TSDF (truncated signed distance field): a
    grid of voxels each storing how far it is to the nearest surface. That gives
    a mesh you can look at, and an ESDF slice at floor height that is a 2D
    occupancy grid -- the blueprint, and what nav2 will plan on.

WHERE THE POSE COMES FROM
    TF, `global_frame` -> the depth image's frame. The chain is

        odom --(fusion_node, 20 Hz)--> base_link --(static)--> camera0_depth_optical_frame

    so the map is built on the FUSED pose, not on raw cuVSLAM. That matters more
    here than anywhere: nvblox writes depth wherever the pose claims the robot
    is, so a teleport smears geometry into the map permanently. Phase 1's whole
    point was to stop that reaching this node.

    global_frame is `odom`, not `map`. There is no map frame until something
    localizes against a saved map -- Phase 2c. Pointing this at a frame nothing
    publishes produces an empty map and a confusing hunt.

VOXEL SIZE
    5 cm. Small enough to see a chair leg, large enough that a room fits in GPU
    memory and the ESDF keeps up. The Phase 1 drift gate was set at 10 cm for
    exactly this reason -- two voxels.

COLOUR IS OFF
    The camera runs with colour disabled, because enabling it with
    `enable_sync:=false` gates the IR pair behind colour alignment and starves
    the stereo stream cuVSLAM needs. The map is geometry-only.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nvblox_ros',
            executable='nvblox_node',
            name='nvblox_node',
            output='screen',
            parameters=[{
                'global_frame': 'odom',
                'num_cameras': 1,
                'use_color': False,
                'use_depth': True,
                'use_lidar': False,

                # static_tsdf gives a mesh to look at and an ESDF slice to plan
                # on. The dynamic modes track moving objects and cost more; the
                # room is not moving.
                'mapping_type': 'static_tsdf',
                'voxel_size': 0.05,

                'publish_esdf_distance_slice': True,
                'esdf_mode': '2d',

                # The floor slice. The camera sits 16.3 cm up and pitches down
                # 1.3 deg, so a slice taken at exactly 0 would clip the floor
                # itself and fill the map with phantom obstacles. Taken from
                # just above the ground plane instead.
                'static_mapper.esdf_slice_min_height': 0.10,
                'static_mapper.esdf_slice_max_height': 0.60,
                'static_mapper.esdf_slice_height': 0.20,

                # How far out to trust depth into the map. Stereo error grows
                # with range SQUARED -- z^2 * sigma_d / (f * b), with f 448.3 and
                # the D555's 9.49 cm baseline:
                #
                #     2 m -> 1.9 cm    4 m -> 7.5 cm    6 m -> 16.9 cm
                #     3 m -> 4.2 cm    5 m -> 11.8 cm
                #
                # 5 m costs ~12 cm of wall fuzz, under 2.5 voxels, and reaches
                # the far wall of a normal room from a corner. 6 m is 3.4 voxels
                # and the walls start to smear rather than sharpen.
                'static_mapper.projective_integrator_max_integration_distance_m': 5.0,
                'static_mapper.projective_integrator_truncation_distance_vox': 4.0,

                'max_mapping_height_m': 2.0,
                'map_clearing_radius_m': 0.0,      # 0 = never clear, we are mapping a room
                'update_mesh_rate_hz': 5.0,
                'update_esdf_rate_hz': 5.0,
            }],
            remappings=[
                ('camera_0/depth/image', '/camera/camera0/depth/image_rect_raw'),
                ('camera_0/depth/camera_info', '/camera/camera0/depth/camera_info'),
            ],
        ),
    ])
