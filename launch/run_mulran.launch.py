"""Run liorf on the MulRan configuration (single robot).

ROS 2 port of ``run_mulran.launch``.
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
        os.path.join(share, 'config', 'mulran.yaml'),
        # mapFusion is started by module_loam; give it real values instead of
        # the uninitialised ones the ROS 1 launch left it with.
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
