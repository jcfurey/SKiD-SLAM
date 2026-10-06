#!/usr/bin/env python3
"""Exercise the real SLAM IMU solver with lidar corrections faster than the IMU."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import yaml

from sparse_sensor_test_utils import arguments, parameters


def main():
    args = arguments('Sparse-IMU solver regression')
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('skid_sparse_imu_check')
    agent = 'SparseTimingTest'
    config = parameters(agent, 'lidar0', {'NumberOfChannels': 16, 'MeasurementsPerCycle': 512})
    config['use_sim_time'] = False
    imu_pub = node.create_publisher(Imu, config['liorf.imuTopic'], qos_profile_sensor_data)
    correction = node.create_publisher(Odometry, f'/{agent}/liorf/mapping/odometry_incremental', 10)
    mapping = node.create_publisher(Odometry, f'/{agent}/liorf/mapping/odometry', 10)
    outputs = []
    node.create_subscription(Odometry, f'/{agent}/slam/odometry/imu_incremental', outputs.append, 10)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', dir=output_dir) as params:
        yaml.safe_dump({'/**': {'ros__parameters': config}}, params)
        params.flush()
        log_path = output_dir/'skid-sparse-imu.log'
        with log_path.open('w') as log:
            process = subprocess.Popen([
                args.executable, '--ros-args',
                '--params-file', params.name], stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            try:
                deadline = time.monotonic()+10
                while imu_pub.get_subscription_count() == 0 or correction.get_subscription_count() == 0:
                    if process.poll() is not None or time.monotonic() >= deadline:
                        raise RuntimeError('Sparse-IMU solver did not start')
                    rclpy.spin_once(node, timeout_sec=.05)
                epoch = node.get_clock().now().nanoseconds
                started = time.monotonic()
                for tick in range(300):
                    stamp = Time(nanoseconds=epoch+tick*20000000).to_msg()
                    if tick % 5 == 0:
                        imu = Imu()
                        imu.header.stamp = stamp
                        imu.header.frame_id = agent+'/imu_body'
                        imu.orientation.w = 1.0
                        imu.linear_acceleration.z = 9.80665
                        imu_pub.publish(imu)
                    pose = Odometry()
                    pose.header.stamp = stamp
                    pose.header.frame_id = agent+'/slam/odom'
                    pose.pose.pose.orientation.w = 1.0
                    correction.publish(pose)
                    pose.header.frame_id = agent+'/slam/map'
                    mapping.publish(pose)
                    until = started+(tick+1)*.02
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=max(0., min(.01, until-time.monotonic())))
                    if process.poll() is not None:
                        raise RuntimeError('Sparse IMU corrections aborted the solver; see '+str(log_path))
                assert len(outputs) > 20, 'No propagated IMU estimates'
                for output in outputs:
                    p = output.pose.pose.position
                    assert all(math.isfinite(v) for v in (p.x,p.y,p.z))
                    assert math.sqrt(p.x*p.x+p.y*p.y+p.z*p.z) < .1
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                node.destroy_node()
                rclpy.shutdown()
    text = log_path.read_text()
    assert 'lidar correction without IMU samples' in text, 'Empty integration intervals were not exercised'
    report = {'passed': True, 'imu_hz': 10, 'correction_hz': 50,
              'propagated_estimates': len(outputs), 'empty_intervals_skipped': True,
              'solver_survived': True, 'stationary_position_error_limit_m': .1}
    (output_dir/'skid-sparse-imu.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
