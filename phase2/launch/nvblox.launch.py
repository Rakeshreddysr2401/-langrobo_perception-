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

                # THE SLICE IS THE ROVER'S OWN HEIGHT, NOT THE CAMERA'S RANGE.
                #
                # nvblox builds a 3D model; the 2D map is a horizontal slice
                # through it, and anything inside the band becomes an obstacle.
                # Set the band by what the ROVER COLLIDES WITH:
                #
                #   camera        17 cm
                #   rover height  22 cm   <- this is the number that matters
                #   table         29 cm   <- 7 cm of clearance above the rover
                #
                #   0 - 10 cm   skipped. The camera pitches down 1.3 deg, so at
                #               3 m the floor itself reads up to 6.8 cm high and
                #               would fill the map with phantom obstacles.
                #   10 - 22 cm  the OBSTACLE band. Anything here, the rover hits.
                #   above 22 cm ignored. The rover drives under it.
                #
                # With the old 0.60 m ceiling a 29 cm table was inside the band,
                # so driving under one painted obstacles in every direction --
                # the rover was surrounded by a tabletop it could comfortably fit
                # beneath. Table LEGS still span 0-29 cm and stay flagged, which
                # is what actually needs avoiding.
                #
                # Heights are in the odom frame, whose z=0 is where base_link
                # started -- ground level. Raise this if the rover grows a mast;
                # lower it and it will drive into things it cannot clear.
                'static_mapper.esdf_slice_min_height': 0.10,
                'static_mapper.esdf_slice_max_height': 0.22,
                'static_mapper.esdf_slice_height': 0.16,

                # How far out to trust depth into the map. Stereo error grows
                # with range SQUARED -- z^2 * sigma_d / (f * b), with f 448.3 and
                # the D555's 9.49 cm baseline:
                #
                #     2 m -> 1.9 cm    4 m -> 7.5 cm    6 m -> 16.9 cm
                #     3 m -> 4.2 cm    5 m -> 11.8 cm
                #
                # 3 m, and the reason is the SLICE, not the voxel size.
                #
                # The obstacle band is only 12 cm tall (0.10-0.22 m, the rover's
                # height). Depth error at 5 m is ~11.8 cm -- the whole height of
                # that band -- so a distant wall's points scatter vertically out
                # of it. The wall is never marked solid, the ray passes THROUGH,
                # and free space is written beyond it.
                #
                # Measured 2026-08-22 at 5 m: walls appeared only near the rover
                # and never at the room's edges, and the map claimed 150 m2 of
                # free floor for a room nothing like that size. Both symptoms are
                # the same cause.
                #
                #   range   depth error   vs the 12 cm band
                #     2 m       1.9 cm    well inside
                #     3 m       4.2 cm    about a third -- usable
                #     5 m      11.8 cm    the entire band
                #
                # An earlier version of this file raised the limit to 5 m to
                # reach far walls. It reached them with data too noisy to use.
                # Solid walls at 3 m beat scattered walls at 5 m: drive closer.
                'static_mapper.projective_integrator_max_integration_distance_m': 3.0,
                'static_mapper.projective_integrator_truncation_distance_vox': 4.0,

                # DO NOT LET THE MAP FORGET.
                #
                # nvblox decays the TSDF by default: every voxel's weight is
                # multiplied by tsdf_decay_factor (0.95) at decay_tsdf_rate_hz
                # (5 Hz), and once it falls low enough the block is DEALLOCATED.
                # That is a 2.7 second half-life:
                #
                #     not seen for  3 s -> 50% weight
                #                  10 s ->  7.7%
                #                  30 s ->  0.05%, effectively erased
                #
                # It is the right behaviour for a scene full of moving things,
                # where a stale observation is worse than none. It is exactly
                # wrong for surveying a static room: measured 2026-08-22, a full
                # room loop ended with LESS map than before it -- 8.74 m2 of
                # floor down to 3.04 m2 -- because everything seen early in the
                # loop had been deleted by the time the rover came back round.
                #
                # 0.0 disables it. The cost is that a moving object leaves a
                # permanent ghost; the room does not move.
                'decay_tsdf_rate_hz': 0.0,
                'decay_dynamic_occupancy_rate_hz': 0.0,

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
