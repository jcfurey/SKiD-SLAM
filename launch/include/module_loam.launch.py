"""One robot's SLAM pipeline: deskew, IMU pre-integration, mapping and map fusion.

Included once per robot by the ``run_*.launch.py`` files. Every node reads the
same parameter files; ``robot_id`` selects the topic namespace prefix the node
uses, mirroring the ROS 1 launch structure.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    id1 = LaunchConfiguration('id1').perform(context)
    id2 = LaunchConfiguration('id2').perform(context)
    no = int(LaunchConfiguration('no').perform(context))
    params = [
        path for path in
        LaunchConfiguration('params').perform(context).split(':') if path]
    pcm_matrix_folder = LaunchConfiguration('pcm_matrix_folder').perform(context)
    use_map_fusion = (
        LaunchConfiguration('use_map_fusion').perform(context).lower() in
        ('true', '1'))
    map_frame = LaunchConfiguration('map_frame').perform(context)
    map_fusion_frame = LaunchConfiguration('map_fusion_frame').perform(context)
    map_fusion_anchor = (
        LaunchConfiguration('map_fusion_anchor').perform(context).lower() in
        ('true', '1'))
    earth_frame = LaunchConfiguration('earth_frame').perform(context)
    geographic_frame_mode = LaunchConfiguration('geographic_frame_mode').perform(context)
    map_datum = LaunchConfiguration('map_datum').perform(context).strip()

    common = {'robot_id': robot, 'no': no}
    if map_frame:
        common['liorf.mapFrame'] = map_frame
    if map_fusion_frame:
        common['liorf.mapFusionFrame'] = map_fusion_frame
    common['liorf.mapFusionAnchor'] = map_fusion_anchor
    if earth_frame:
        common['liorf.earthFrame'] = earth_frame
    if geographic_frame_mode:
        common['liorf.geographicFrameMode'] = geographic_frame_mode
    if map_datum:
        values = [value.strip() for value in map_datum.split(',')]
        if len(values) != 4:
            raise ValueError(
                'map_datum must be latitude,longitude,ellipsoid_height,yaw')
        latitude, longitude, altitude, yaw = (float(value) for value in values)
        common.update({
            'liorf.mapDatumConfigured': True,
            'liorf.mapDatumLatitude': latitude,
            'liorf.mapDatumLongitude': longitude,
            'liorf.mapDatumAltitude': altitude,
            'liorf.mapDatumYaw': yaw,
        })
    suffix = '_' + robot if robot else ''

    nodes = [
        Node(
            package='liorf',
            executable='liorf_imageProjection',
            name='liorf_imageProjection' + suffix,
            parameters=params + [common],
            output='screen',
            respawn=True,
        ),
        Node(
            package='liorf',
            executable='liorf_mapOptmization',
            name='liorf_mapOptmization' + suffix,
            parameters=params + [common],
            output='screen',
            respawn=False,
        ),
        # This executable hosts two nodes in one process, so a process-wide
        # `name=` would rename both. Remap each node name individually instead.
        Node(
            package='liorf',
            executable='liorf_imuPreintegration',
            parameters=params + [common],
            output='screen',
            respawn=True,
            ros_arguments=[
                '-r', 'liorf_imuPreintegration:__node:=liorf_imuPreintegration' + suffix,
                '-r', 'liorf_transformFusion:__node:=liorf_transformFusion' + suffix,
            ],
        ),
    ]

    if use_map_fusion:
        nodes.append(Node(
            package='liorf',
            executable='liorf_mapFusion',
            name='liorf_mapFusion' + suffix,
            parameters=params + [{
                **common,
                'robot_id': robot,
                'id_1': id1,
                'id_2': id2,
                'no': no,
                'pcm_matrix_folder': pcm_matrix_folder,
            }],
            output='screen',
            respawn=False,
        ))

    return nodes


def generate_launch_description():
    default_pcm = os.path.join(get_package_share_directory('liorf'), 'config')

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='jackal0'),
        DeclareLaunchArgument('id1', default_value='jackal1'),
        DeclareLaunchArgument('id2', default_value='jackal2'),
        DeclareLaunchArgument('no', default_value='1'),
        DeclareLaunchArgument(
            'params', default_value='',
            description='Colon separated list of ROS 2 parameter files'),
        DeclareLaunchArgument(
            'use_map_fusion', default_value='true',
            description='Also start the inter-robot map fusion node'),
        DeclareLaunchArgument(
            'map_frame', default_value='',
            description='Optional fully resolved local map frame override'),
        DeclareLaunchArgument(
            'map_fusion_frame', default_value='',
            description='Optional parent frame for accepted inter-robot alignment'),
        DeclareLaunchArgument(
            'map_fusion_anchor', default_value='false',
            description='Define map_fusion_frame -> map_frame identity as the gauge'),
        DeclareLaunchArgument('earth_frame', default_value=''),
        DeclareLaunchArgument(
            'geographic_frame_mode', default_value='',
            description='Optional local_only or ecef_anchored override'),
        DeclareLaunchArgument(
            'map_datum', default_value='',
            description=(
                'Optional latitude,longitude,ellipsoid_height_m,map_yaw_rad')),
        DeclareLaunchArgument('pcm_matrix_folder', default_value=default_pcm),
        OpaqueFunction(function=launch_setup),
    ])
