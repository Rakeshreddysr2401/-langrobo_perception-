#!/usr/bin/env python3
"""nvblox — build the room as the rover drives.

WHAT IT DOES
    Depth images plus a pose become a TSDF (truncated signed distance field): a
    grid of voxels each storing how far it is to the nearest surface. That gives
    a mesh you can look at, and an ESDF slice at floor height that is a 2D
    occupancy grid -- the blueprint, and what nav2 will plan on.

WHERE THE POSE COMES FROM
    TF, `global_frame` -> the depth image's frame. The chain is

        odom --(fusion2, 20 Hz)--> base_link --(static)--> camera0_depth_optical_frame

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
                # Set the band by what the ROVER COLLIDES WITH (measured
                # 2026-09-24, description/params.yaml):
                #
                #   frame top        22.0 cm
                #   LiDAR puck top   26.1 cm  <- the highest point of the rover
                #   table            29   cm  <- still clears it
                #
                #   0 - 5 cm    skipped: one 5 cm voxel, the floor itself.
                #   5 - 27 cm   the OBSTACLE band. A 6-8 cm remote, shoe or box
                #               is in it; so is anything the puck would hit.
                #   above 27 cm ignored. The rover drives under it.
                #
                # This used to start at 10 cm, because the TF said the camera
                # was level while it was not: the floor then rose with range
                # (6.8 cm at 3 m) and filled the map with phantom obstacles.
                # The price was that everything under 10 cm was invisible. The
                # camera's tilt is now MEASURED off the floor in its own depth
                # (./rover camera --floor, 2026-09-25: roll -1.4 deg, pitch
                # ~0) and is in the TF, so the floor lands at z ~ 0 and one
                # voxel of margin is enough. If phantom obstacles appear on
                # bare floor, re-run --floor before raising this.
                #
                # Things under ~5 cm (a flat cable) stay invisible to the map;
                # the axle is 4.1 cm up, so about 1.5-2 cm is what it can
                # climb. Heights are in the odom frame: z = 0 is the floor.
                'static_mapper.esdf_slice_min_height': 0.05,
                'static_mapper.esdf_slice_max_height': 0.27,
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
