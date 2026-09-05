#!/usr/bin/env python3
"""Synthetic ROS/ZeroMQ graph lifecycle bench; no bag or physical robot needed.

Starts two real mapping nodes, two SOLiD fusion nodes, two bridges, and bounded
sensor/protocol drivers. Tests actual pose correction, owner-state revision,
withdrawal across an interrupted link, stale replay, and publisher echo handling.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time


def write_json(path, value):
    temporary = path.with_suffix('.partial')
    temporary.write_text(json.dumps(value))
    temporary.replace(path)


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def worker(args):
    import rclpy
    from liorf.msg import CloudInfo, LoopConstraint, ScanData
    from nav_msgs.msg import Path as RosPath
    from sensor_msgs.msg import PointCloud2, PointField
    rclpy.init()
    node = rclpy.create_node('graph_sync_fixture_' + args.robot)
    robot = args.robot
    directory = Path(args.artifact_dir)
    status = {'robot': robot, 'epoch': 0, 'keyframes': 0, 'x': None,
              'scan_bytes_sent': 0, 'factor_messages_sent': 0}
    cloud_pub = node.create_publisher(CloudInfo, f'/{robot}/liorf/deskew/cloud_info', 10)
    factor_pub = node.create_publisher(LoopConstraint, '/solid/loop_info_global', 100)
    scan_pub = node.create_publisher(ScanData, '/solid/scan_data', 10)

    def cloud_received(message):
        status['epoch'] = int(message.trajectory_epoch)
        status['keyframes'] = max(status['keyframes'], int(message.imu_available) + 1)

    def path_received(message):
        if message.poses:
            status['x'] = message.poses[-1].pose.position.x

    node.create_subscription(CloudInfo, f'/{robot}/liorf/mapping/cloud_info', cloud_received, 10)
    node.create_subscription(RosPath, f'/{robot}/liorf/mapping/path', path_received, 10)

    def cloud():
        message = PointCloud2()
        message.height = 1
        message.width = 4
        message.point_step = 16
        message.row_step = 64
        message.is_dense = True
        message.fields = [PointField(name=name, offset=offset, datatype=7, count=1)
                          for name, offset in [('x', 0), ('y', 4), ('z', 8), ('intensity', 12)]]
        message.data = b''.join(struct.pack('<ffff', x, y, 0, 1)
                                for x, y in [(1, 0), (2, 0), (1, 1), (2, 2)])
        return message

    seed_index = 0
    last_seed = last_factor = last_scan = 0.0
    command = {}
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        tick = time.monotonic()
        if seed_index < 3 and cloud_pub.get_subscription_count() and tick - last_seed > 0.5:
            message = CloudInfo()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = 'base_link'
            message.odom_available = 1
            message.initial_guess_x = float(0 if seed_index < 2 else args.distance)
            message.cloud_deskewed = cloud()
            message.cloud_deskewed.header = message.header
            cloud_pub.publish(message)
            seed_index += 1
            last_seed = tick
        command = read_json(directory / (robot + '_command.json')) or command
        if command and tick - last_factor > 0.25:
            for record in command.get('factors', []):
                for recipient in ('jackal0', 'jackal1'):
                    message = LoopConstraint()
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.authority_id = 'jackal1'
                    message.authority_epoch = command['authority_epoch']
                    message.revision = record.get('revision', 1)
                    message.retracted = record.get('retracted', False)
                    message.robot_id = recipient
                    message.from_robot_id = 'jackal0'
                    message.to_robot_id = 'jackal1'
                    message.from_trajectory_epoch = command['epoch_a']
                    message.to_trajectory_epoch = command['epoch_b']
                    message.index_from = message.index_to = record['index']
                    message.from_pose.orientation.w = message.to_pose.orientation.w = 1.0
                    message.from_pose.position.x = record['index'] * 10.0
                    message.to_pose.position.x = record['index'] * 8.0
                    message.relative_pose.pose.orientation.w = 1.0
                    for diagonal in (0, 7, 14, 21, 28, 35):
                        message.relative_pose.covariance[diagonal] = 1e-6
                    factor_pub.publish(message)
                    status['factor_messages_sent'] += 1
            if command.get('local_loop'):
                message = LoopConstraint()
                message.robot_id = message.from_robot_id = message.to_robot_id = 'jackal1'
                message.index_from, message.index_to = 0, 1
                message.relative_pose.pose.orientation.w = 1.0
                message.relative_pose.pose.position.x = 6.0
                for diagonal in (0, 7, 14, 21, 28, 35):
                    message.relative_pose.covariance[diagonal] = 1e-8
                factor_pub.publish(message)
            last_factor = tick
        if command.get('load') and tick - last_scan > 0.1:
            message = ScanData()
            message.header.stamp = node.get_clock().now().to_msg()
            message.robot_id, message.robot_id_receive = 'jackal1', 'jackal0'
            message.trajectory_epoch = command['epoch_b']
            message.keyframe_index = 999999
            message.available = True
            message.scan_cloud = cloud()
            message.scan_cloud.width = 65536
            message.scan_cloud.row_step = 1048576
            message.scan_cloud.data = bytes(1048576)
            scan_pub.publish(message)
            status['scan_bytes_sent'] += 1048576
            last_scan = tick
        write_json(directory / (robot + '_status.json'), status)
    node.destroy_node()
    rclpy.shutdown()


def stop(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)


def bench(args):
    from ament_index_python.packages import get_package_prefix
    if args.domain_a == args.domain_b or args.port_a == args.port_b:
        raise ValueError('domains and ports must be distinct')
    if not all(0 <= domain <= 232 for domain in (args.domain_a, args.domain_b)):
        raise ValueError('domain IDs must be in [0,232]')
    if not all(1024 <= port <= 65535 for port in (args.port_a, args.port_b)):
        raise ValueError('ports must be in [1024,65535]')
    directory = Path(args.artifact_dir) if args.artifact_dir else Path(tempfile.mkdtemp(prefix='skid_graph_sync_'))
    if args.artifact_dir:
        directory.mkdir(parents=True, exist_ok=False)
    executables = Path(get_package_prefix('liorf')) / 'lib/liorf'
    processes, handles = [], []
    summary = {'artifact_dir': str(directory), 'phases': [], 'passed': False}

    def launch(name, command, domain):
        environment = os.environ.copy()
        environment.update(ROS_DOMAIN_ID=str(domain), ROS2CLI_DISABLE_DAEMON='1',
                           ROS_LOG_DIR=str(directory / 'ros_logs'))
        handle = (directory / (name + '.log')).open('w')
        handles.append(handle)
        process = subprocess.Popen(command, env=environment, stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process

    def bridge(robot, domain, port, peer):
        return launch('bridge_' + robot + '_' + str(len(processes)),
            [str(executables / 'liorf_zmqBridge'), '--ros-args', '-p', f'robot_id:={robot}',
             '-p', f'zmq.bind_endpoint:=tcp://127.0.0.1:{port}',
             '-p', f'zmq.peer_endpoints:=["tcp://127.0.0.1:{peer}"]',
             '-p', 'zmq.report_period_s:=1.0'], domain)

    def status(robot):
        return read_json(directory / (robot + '_status.json'))

    def wait_for(name, predicate, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for process in processes:
                if process.poll() is not None:
                    raise RuntimeError(f'{name}: process exited: {process.args[0]} ({process.returncode})')
            if predicate():
                snapshot = {'name': name, 'a': status('jackal0'), 'b': status('jackal1')}
                summary['phases'].append(snapshot)
                print(name + ': ' + json.dumps(snapshot), flush=True)
                return
            time.sleep(0.1)
        raise TimeoutError(f'{name}: a={status("jackal0")}, b={status("jackal1")}')

    try:
        for robot, peer, domain, distance in [('jackal0', 'jackal1', args.domain_a, 10),
                                               ('jackal1', 'jackal0', args.domain_b, 8)]:
            common = ['--ros-args', '-p', f'robot_id:={robot}',
                      '-p', 'mapfusion.graph_sync.period_s:=0.25']
            launch('mapping_' + robot, [str(executables / 'liorf_mapOptmization'), *common,
                '-p', 'liorf.loopClosureEnableFlag:=false', '-p', 'liorf.sensor:=velodyne',
                '-p', 'mapfusion.interRobot.remote_odometry_translation_stddev_m:=0.005'], domain)
            launch('fusion_' + robot, [str(executables / 'liorf_mapFusion'), *common,
                '-p', f'id_1:={peer}', '-p', 'id_2:=""'], domain)
            launch('fixture_' + robot, [sys.executable, str(Path(__file__).resolve()), '--worker',
                '--robot', robot, '--distance', str(distance), '--artifact-dir', str(directory)], domain)
        bridge('jackal0', args.domain_a, args.port_a, args.port_b)
        bridge_b = bridge('jackal1', args.domain_b, args.port_b, args.port_a)
        wait_for('seeded_local_trajectories', lambda: all(status(robot).get('keyframes', 0) == 2
                 for robot in ('jackal0', 'jackal1')))
        wait_for('baseline', lambda: abs((status('jackal0').get('x') or 0) - 10) < 0.02 and
                 abs((status('jackal1').get('x') or 0) - 8) < 0.02)
        command = {'epoch_a': status('jackal0')['epoch'], 'epoch_b': status('jackal1')['epoch'],
                   'authority_epoch': time.time_ns(), 'load': True,
                   'factors': [{'index': 0}, {'index': 1}]}
        command_path = directory / 'jackal1_command.json'
        write_json(command_path, command)
        wait_for('distributed_correction', lambda: status('jackal0')['x'] < 9)
        command['local_loop'] = True
        write_json(command_path, command)
        wait_for('revised_peer_motion', lambda: status('jackal0')['x'] < 7)
        stop(bridge_b)
        processes.remove(bridge_b)
        command['factors'] = [{'index': 0, 'revision': 2, 'retracted': True},
                              {'index': 1, 'revision': 2, 'retracted': True}]
        write_json(command_path, command)
        # Keep the bridge offline long enough to miss multiple actual publications.
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            time.sleep(0.1)
        if status('jackal0')['x'] > 7:
            raise AssertionError('the isolated domains still communicated with the bridge stopped')
        bridge('jackal1', args.domain_b, args.port_b, args.port_a)
        wait_for('recovered_withdrawals', lambda: abs(status('jackal0')['x'] - 10) < 0.02 and
                 abs(status('jackal1')['x'] - 6) < 0.02)
        command['factors'] = [{'index': 0}, {'index': 1}]
        write_json(command_path, command)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            if abs(status('jackal0')['x'] - 10) > 0.02:
                raise AssertionError('stale replay resurrected a withdrawn factor')
            time.sleep(0.1)
        summary['phases'].append({'name': 'stale_replay_rejected', 'a': status('jackal0'),
                                  'b': status('jackal1')})
        summary['passed'] = True
    finally:
        for process in reversed(processes):
            stop(process)
        for handle in handles:
            handle.close()
        write_json(directory / 'result.json', summary)
        print('Artifacts: ' + str(directory), flush=True)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--robot', help=argparse.SUPPRESS)
    parser.add_argument('--distance', type=float, help=argparse.SUPPRESS)
    parser.add_argument('--domain-a', type=int)
    parser.add_argument('--domain-b', type=int)
    parser.add_argument('--port-a', type=int)
    parser.add_argument('--port-b', type=int)
    parser.add_argument('--artifact-dir')
    args = parser.parse_args()
    if args.worker:
        try:
            return worker(args)
        except KeyboardInterrupt:
            return 0
    if None in (args.domain_a, args.domain_b, args.port_a, args.port_b):
        parser.error('explicit --domain-a/--domain-b and --port-a/--port-b are required')
    return bench(args)


if __name__ == '__main__':
    sys.exit(main())
