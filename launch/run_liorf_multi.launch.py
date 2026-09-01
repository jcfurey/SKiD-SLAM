"""Distributed multi-robot SLAM: one liorf pipeline per robot plus map fusion.

ROS 2 port of ``run_liorf_multi.launch``. Robots exchange SOLiD descriptors on
the global ``/solid/...`` topics; each robot's own topics stay under its
``robot_id`` prefix.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('liorf')
    include = os.path.join(share, 'launch', 'include')

    params = ':'.join([
        os.path.join(share, 'config', 'lio_sam_default.yaml'),
        os.path.join(share, 'config', 'mapfusion_solid.yaml'),
    ])

    robots = [LaunchConfiguration(n).perform(context) for n in ('robot0', 'robot1', 'robot2')]
    robots = [r for r in robots if r]
    no = LaunchConfiguration('no').perform(context)
    fleet_frame = LaunchConfiguration('fleet_frame').perform(context)
    earth_frame = LaunchConfiguration('earth_frame').perform(context)
    alignment_mode = LaunchConfiguration('alignment_mode').perform(context)
    if alignment_mode not in ('map_fusion', 'ecef', 'detached'):
        raise ValueError(
            'alignment_mode must be map_fusion, ecef, or detached')

    datums = [
        LaunchConfiguration('robot0_datum').perform(context).strip(),
        LaunchConfiguration('robot1_datum').perform(context).strip(),
        LaunchConfiguration('robot2_datum').perform(context).strip(),
    ]
    if alignment_mode == 'ecef':
        missing = [robots[i] for i in range(len(robots)) if not datums[i]]
        if missing:
            raise ValueError(
                'ecef alignment requires an explicit datum for: ' +
                ', '.join(missing))

    use_map_fusion = alignment_mode == 'map_fusion'
    map_fusion_frame = fleet_frame if use_map_fusion else ''
    fixed_frame = earth_frame if alignment_mode == 'ecef' else (
        fleet_frame if use_map_fusion else robots[0] + '/map')

    actions = []
    for index, robot in enumerate(robots):
        others = [r for r in robots if r != robot]
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include, 'module_loam.launch.py')),
            launch_arguments={
                'robot': robot,
                'id1': others[0] if len(others) > 0 else '',
                'id2': others[1] if len(others) > 1 else '',
                'no': no,
                'params': params,
                # Each platform keeps a numerically local map. The first map
                # defines the fleet-map gauge; other maps remain detached
                # until map fusion publishes an accepted alignment.
                'map_frame': robot + '/map',
                'map_fusion_frame': map_fusion_frame,
                'map_fusion_anchor': (
                    'true' if use_map_fusion and index == 0 else 'false'),
                'use_map_fusion': 'true' if use_map_fusion else 'false',
                'earth_frame': earth_frame,
                'geographic_frame_mode': (
                    'ecef_anchored' if alignment_mode == 'ecef'
                    else 'local_only'),
                'map_datum': datums[index] if alignment_mode == 'ecef' else '',
            }.items(),
        ))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(include, 'module_rviz.launch.py')),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'fixed_frame': fixed_frame,
        }.items(),
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('no', default_value='1'),
        DeclareLaunchArgument('robot0', default_value='jackal0'),
        DeclareLaunchArgument('robot1', default_value='jackal1'),
        # Set to an empty string to run with two robots, as the ROS 1 launch did.
        DeclareLaunchArgument('robot2', default_value=''),
        DeclareLaunchArgument(
            'fleet_frame', default_value='map',
            description='Parent frame for accepted inter-platform map alignments'),
        DeclareLaunchArgument(
            'alignment_mode', default_value='map_fusion',
            description='map_fusion, ecef, or detached'),
        DeclareLaunchArgument('earth_frame', default_value='earth'),
        DeclareLaunchArgument(
            'robot0_datum', default_value='',
            description='lat,lon,ellipsoid_height_m,map_yaw_rad'),
        DeclareLaunchArgument('robot1_datum', default_value=''),
        DeclareLaunchArgument('robot2_datum', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
