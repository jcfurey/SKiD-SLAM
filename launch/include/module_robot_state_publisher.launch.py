"""Publishes the sensor-frame TF tree from robot.urdf.xacro.

Not started by the ``run_*`` launch files by default, matching the ROS 1 setup
where the include was commented out.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(
        get_package_share_directory('liorf'),
        'launch', 'include', 'config', 'robot.urdf.xacro')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            respawn=True,
            parameters=[{
                'robot_description': Command(['xacro ', urdf]),
            }],
            output='screen',
        ),
    ])
