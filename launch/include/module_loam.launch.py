"""One robot's SLAM pipeline: deskew, IMU pre-integration, mapping and map fusion.

Included once per robot by the ``run_*.launch.py`` files. Every node reads the
same parameter files; ``robot_id`` selects the topic namespace prefix the node
uses, mirroring the ROS 1 launch structure.
"""

import copy
import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _deep_merge(destination, source):
    for key, value in source.items():
        if (isinstance(value, dict) and
                isinstance(destination.get(key), dict)):
            _deep_merge(destination[key], value)
        else:
            destination[key] = copy.deepcopy(value)


def _load_parameter_files(paths):
    """Merge wildcard ROS parameter files in their declared order.

    Lyrical's rcl YAML loader does not reliably let a flat dotted launch
    override replace the equivalent nested key from another file. Producing
    one parameter tree here removes that ambiguous duplicate representation.
    """
    merged = {}
    for path in paths:
        with open(path, 'r', encoding='utf-8') as handle:
            document = yaml.safe_load(handle) or {}
        wildcard = document.get('/**', {})
        parameters = wildcard.get('ros__parameters', {})
        if not isinstance(parameters, dict):
            raise ValueError(
                f'{path}: /**.ros__parameters must be a mapping')
        _deep_merge(merged, parameters)
    return merged


def _with_overrides(parameters, overrides):
    result = copy.deepcopy(parameters)
    for dotted_name, value in overrides.items():
        parts = dotted_name.split('.')
        target = result
        for part in parts[:-1]:
            existing = target.get(part)
            if existing is None:
                existing = {}
                target[part] = existing
            if not isinstance(existing, dict):
                raise ValueError(
                    f'parameter namespace {dotted_name!r} collides with a value')
            target = existing
        target[parts[-1]] = value
    return result


def _default_pcm_directory():
    return os.path.join(tempfile.gettempdir(), 'skid_slam_pcm')


def _as_bool(value):
    return value.lower() in ('true', '1')


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    id1 = LaunchConfiguration('id1').perform(context)
    id2 = LaunchConfiguration('id2').perform(context)
    no = int(LaunchConfiguration('no').perform(context))
    parameter_paths = [
        path for path in
        LaunchConfiguration('params').perform(context).split(':') if path]
    base_parameters = _load_parameter_files(parameter_paths)
    pcm_matrix_folder = LaunchConfiguration('pcm_matrix_folder').perform(context)
    os.makedirs(pcm_matrix_folder, exist_ok=True)
    use_map_fusion = _as_bool(
        LaunchConfiguration('use_map_fusion').perform(context))
    respawn = _as_bool(LaunchConfiguration('respawn').perform(context))
    map_frame = LaunchConfiguration('map_frame').perform(context)
    map_fusion_frame = LaunchConfiguration('map_fusion_frame').perform(context)
    map_fusion_anchor = _as_bool(
        LaunchConfiguration('map_fusion_anchor').perform(context))
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
    node_parameters = _with_overrides(base_parameters, common)

    nodes = [
        Node(
            package='liorf',
            executable='liorf_imageProjection',
            name='liorf_imageProjection' + suffix,
            parameters=[node_parameters],
            output='screen',
            respawn=respawn,
        ),
        Node(
            package='liorf',
            executable='liorf_mapOptmization',
            name='liorf_mapOptmization' + suffix,
            parameters=[node_parameters],
            output='screen',
            respawn=False,
        ),
        # This executable hosts two nodes in one process, so a process-wide
        # `name=` would rename both. Remap each node name individually instead.
        Node(
            package='liorf',
            executable='liorf_imuPreintegration',
            parameters=[node_parameters],
            output='screen',
            respawn=respawn,
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
            parameters=[_with_overrides(base_parameters, {
                **common,
                'robot_id': robot,
                'id_1': id1,
                'id_2': id2,
                'no': no,
                'pcm_matrix_folder': pcm_matrix_folder,
            })],
            output='screen',
            respawn=False,
        ))

    return nodes


def generate_launch_description():
    # The maximum-clique implementation exchanges its graph through a file.
    # Keep that runtime artifact away from the installed (and, with symlink
    # installs, source-controlled) package config directory.
    default_pcm = _default_pcm_directory()

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
            'respawn', default_value='true',
            description='Restart front-end and IMU nodes after an unexpected exit'),
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
