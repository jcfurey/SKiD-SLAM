#!/usr/bin/env python3
"""Check the actual deskew node with scan endpoints between sparse IMU samples."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud, read_points
from std_msgs.msg import Header
import yaml
from sparse_sensor_test_utils import arguments, parameters


def main():
    args = arguments('Sparse-IMU deskew regression')
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('skid_sparse_deskew_check')
    agent = 'SparseDeskewTest'
    config = parameters(agent, 'lidar0', {'NumberOfChannels': 16, 'MeasurementsPerCycle': 512})
    config['use_sim_time'] = False
    imu_pub = node.create_publisher(Imu, config['liorf.imuTopic'], qos_profile_sensor_data)
    scan_pub = node.create_publisher(PointCloud2, config['liorf.pointCloudTopic'], qos_profile_sensor_data)
    outputs = []
    node.create_subscription(PointCloud2, f'/{agent}/liorf/deskew/cloud_deskewed', outputs.append, 10)
    log_path = output_dir/'skid-sparse-deskew.log'
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', dir=output_dir) as params:
        yaml.safe_dump({'/**': {'ros__parameters': config}}, params)
        params.flush()
        with log_path.open('w') as log:
            process = subprocess.Popen([
                args.executable, '--ros-args',
                '--params-file', params.name], stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            try:
                def spin_for(seconds):
                    until = time.monotonic()+seconds
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=.01)
                        if process.poll() is not None:
                            raise RuntimeError('Deskew node exited; see '+str(log_path))
                deadline = time.monotonic()+10
                while not imu_pub.get_subscription_count() or not scan_pub.get_subscription_count():
                    if time.monotonic() >= deadline:
                        raise RuntimeError('Deskew subscriptions unavailable')
                    spin_for(.05)
                epoch = node.get_clock().now().nanoseconds
                # Each 75 ms scan starts between 60 ms IMU capture intervals.
                for tick in range(34):
                    imu = Imu()
                    imu.header.stamp = Time(nanoseconds=epoch+tick*60000000).to_msg()
                    imu.header.frame_id = agent+'/imu_body'
                    imu.orientation.w = 1.
                    imu.angular_velocity.z = 1.
                    imu.linear_acceleration.z = 9.80665
                    imu_pub.publish(imu)
                    spin_for(.005)
                spin_for(.2)
                fields = [PointField(name=n, offset=o, datatype=t, count=1)
                          for n,o,t in [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),
                                        ('t',16,6),('ring',20,4),('reflectivity',22,4),
                                        ('range',24,6),('ambient',28,4)]]
                angles = np.linspace(-2.2,2.2,32)
                expected = np.column_stack([4*np.cos(angles),4*np.sin(angles),np.linspace(-.2,.2,32)])
                for scan in range(5):
                    points = []
                    for index, point in enumerate(expected):
                        offset = index/31*.075
                        c,s = math.cos(offset),math.sin(offset)
                        xyz = [c*point[0]+s*point[1], -s*point[0]+c*point[1],point[2]]
                        points.append((*xyz,float(index+1),round(offset*1e9),index%16,0,
                                       round(float(np.linalg.norm(point))*1000),0))
                    header = Header(stamp=Time(nanoseconds=epoch+25000000+scan*200000000).to_msg(),
                                    frame_id=agent+'/lidar0')
                    scan_pub.publish(create_cloud(header,fields,points))
                    spin_for(.2)
                spin_for(.5)
                assert len(outputs) >= 3, 'No deskewed scans'
                maximum, checked = 0.,0
                for output in outputs:
                    for point in read_points(output, field_names=('x','y','z','intensity'), skip_nans=True):
                        index = round(float(point['intensity']))-1
                        assert 0 <= index < len(expected)
                        error = np.linalg.norm([float(point['x'])-expected[index,0],
                                                float(point['y'])-expected[index,1],
                                                float(point['z'])-expected[index,2]])
                        maximum = max(maximum,float(error))
                        checked += 1
                assert checked >= 90, f'Too few retained deskew points: {checked}'
                assert maximum < 1e-4, f'Sparse IMU deskew error {maximum:.6f} m'
            finally:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGKILL)
                        process.wait()
                node.destroy_node()
                rclpy.shutdown()
    report = {'passed': True,'imu_interval_s': .06,'scan_duration_s': .075,
              'scan_start_between_imu_samples': True,'points_checked': checked,
              'maximum_deskew_error_m': maximum}
    (output_dir/'skid-sparse-deskew.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
