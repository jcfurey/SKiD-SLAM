"""Two-robot distributed SLAM.

ROS 2 port of ``run_lio_sam_multi.launch``; identical to
``run_liorf_multi.launch.py`` with only two robots enabled.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('liorf')

    return LaunchDescription([
        DeclareLaunchArgument('no', default_value='1'),
        DeclareLaunchArgument('robot0', default_value='jackal0'),
        DeclareLaunchArgument('robot1', default_value='jackal1'),
        DeclareLaunchArgument('fleet_frame', default_value='map'),
        DeclareLaunchArgument('alignment_mode', default_value='map_fusion'),
        DeclareLaunchArgument('earth_frame', default_value='earth'),
        DeclareLaunchArgument('robot0_datum', default_value=''),
        DeclareLaunchArgument('robot1_datum', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'run_liorf_multi.launch.py')),
            launch_arguments={
                'no': LaunchConfiguration('no'),
                'robot0': LaunchConfiguration('robot0'),
                'robot1': LaunchConfiguration('robot1'),
                'robot2': '',
                'fleet_frame': LaunchConfiguration('fleet_frame'),
                'alignment_mode': LaunchConfiguration('alignment_mode'),
                'earth_frame': LaunchConfiguration('earth_frame'),
                'robot0_datum': LaunchConfiguration('robot0_datum'),
                'robot1_datum': LaunchConfiguration('robot1_datum'),
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
    ])
