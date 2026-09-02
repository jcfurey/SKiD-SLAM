"""Run the ZeroMQ inter-robot bridge on one robot.

Every robot runs its own bridge with its own endpoints:

    ros2 launch liorf run_zmq_bridge.launch.py \\
        robot:=jackal0 \\
        bind_endpoint:=tcp://0.0.0.0:7447 \\
        peers:=tcp://192.168.1.12:7447,tcp://192.168.1.13:7447

See doc/FIELD_COMMUNICATION.md for the deployment.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.lower() in ('true', '1')


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('liorf')
    robot = LaunchConfiguration('robot').perform(context)
    bind_endpoint = LaunchConfiguration('bind_endpoint').perform(context)
    peers = [peer for peer in
             LaunchConfiguration('peers').perform(context).split(',') if peer]
    respawn = _as_bool(LaunchConfiguration('respawn').perform(context))

    overrides = {'robot_id': robot, 'zmq.peer_endpoints': peers}
    if bind_endpoint:
        overrides['zmq.bind_endpoint'] = bind_endpoint

    return [Node(
        package='liorf',
        executable='liorf_zmqBridge',
        name='liorf_zmqBridge_' + robot,
        parameters=[
            os.path.join(share, 'config', 'zmq_bridge.yaml'),
            os.path.join(share, 'config', 'mapfusion_solid.yaml'),
            overrides,
        ],
        output='screen',
        respawn=respawn,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='jackal0'),
        DeclareLaunchArgument('bind_endpoint', default_value=''),
        DeclareLaunchArgument(
            'peers', default_value='',
            description='comma-separated peer bind endpoints'),
        DeclareLaunchArgument(
            'respawn', default_value='true',
            description='Restart the bridge after an unexpected exit'),
        OpaqueFunction(function=launch_setup),
    ])
