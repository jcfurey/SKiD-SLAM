"""Parameters and CLI for sensor timing tests against the production nodes."""
import argparse


def arguments(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--executable', required=True)
    parser.add_argument('--output-dir', required=True)
    return parser.parse_args()


def parameters(agent, sensor_name, sensor):
    identity = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    return {
        'robot_id': agent, 'use_sim_time': False,
        'liorf.pointCloudTopic': f'/{agent}/lidar/{sensor_name}',
        'liorf.imuTopic': f'/{agent}/imu/data',
        'liorf.gpsTopic': f'/{agent}/unused_gps',
        'liorf.odomTopic': 'slam/odometry/imu',
        'liorf.lidarFrame': 'slam/lidar', 'liorf.baselinkFrame': 'slam/lidar',
        'liorf.odometryFrame': 'slam/odom', 'liorf.mapFrame': f'{agent}/slam/map',
        'liorf.sensor': 'ouster', 'liorf.N_SCAN': int(sensor['NumberOfChannels']),
        'liorf.Horizon_SCAN': int(sensor['MeasurementsPerCycle']),
        # Separate lists avoid YAML aliases, which rcl's parser rejects.
        'liorf.extrinsicRot': identity.copy(), 'liorf.extrinsicRPY': identity.copy(),
        'liorf.extrinsicTrans': [0., 0., 0.],
        'liorf.imuType': 0, 'liorf.imuRate': 100., 'liorf.imuGravity': 9.80665,
        'liorf.imuAccNoise': .02, 'liorf.imuGyrNoise': .005,
        'liorf.imuAccBiasN': .0002, 'liorf.imuGyrBiasN': .00005,
        'liorf.useImuHeadingInitialization': False,
        'liorf.lidarMinRange': .5, 'liorf.lidarMaxRange': 30.,
        'liorf.numberOfCores': 2, 'liorf.downsampleRate': 1,
        'liorf.point_filter_num': 1, 'liorf.loopClosureEnableFlag': False,
    }
