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

    actions = []
    for robot in robots:
        others = [r for r in robots if r != robot]
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include, 'module_loam.launch.py')),
            launch_arguments={
                'robot': robot,
                'id1': others[0] if len(others) > 0 else '',
                'id2': others[1] if len(others) > 1 else '',
                'no': no,
                'params': params,
            }.items(),
        ))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(include, 'module_rviz.launch.py')),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items(),
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('no', default_value='1'),
        DeclareLaunchArgument('robot0', default_value='jackal0'),
        DeclareLaunchArgument('robot1', default_value='jackal1'),
        # Set to an empty string to run with two robots, as the ROS 1 launch did.
        DeclareLaunchArgument('robot2', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
