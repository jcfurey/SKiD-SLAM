"""Run liorf on the Moon configuration (single robot).

Paper dataset configuration ported from commit 0611f71; see
config/moon.yaml for what changed in the port.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('liorf')
    include = os.path.join(share, 'launch', 'include')

    params = ':'.join([
        os.path.join(share, 'config', 'moon.yaml'),
        # mapFusion is started by module_loam and needs its own parameters
        # even in a single-robot run.
        os.path.join(share, 'config', 'mapfusion_solid.yaml'),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='jackal0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include, 'module_loam.launch.py')),
            launch_arguments={
                'robot': LaunchConfiguration('robot'),
                'params': params,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include, 'module_rviz.launch.py')),
            launch_arguments={'rviz': LaunchConfiguration('rviz')}.items(),
        ),
    ])
