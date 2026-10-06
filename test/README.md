# Sensor timing regressions

The production nodes support IMU capture intervals longer than lidar correction
intervals. `test_sparse_imu_solver` feeds the GTSAM node stationary gravity at
10 Hz and lidar corrections at 50 Hz. Empty preintegrations must leave the
graph unchanged, without a singular IMU factor or zero bias sigmas. The test
requires finite, stationary propagated estimates and verifies that the empty
interval guard was exercised.

`test_sparse_imu_deskew` sends 75 ms Ouster scans whose endpoints fall between
60 ms IMU samples. It verifies known rotating geometry after deskewing, which
requires retaining the samples bracketing both scan endpoints.

`test_scan_matching_jacobian` reads the production point-to-plane expressions
and compares all three rotation derivatives with central numeric differences
over 100 poses. This catches the pitch coefficient's former `sin(pitch)` in
place of `cos(pitch)`.

Run these through CTest after building the package:

```bash
ctest --test-dir build/liorf --output-on-failure \
  -R 'test_sparse_imu|test_scan_matching_jacobian'
```

The node fixtures use separate ROS domains 204 and 205, run the executables
from the build tree, and write their JSON reports and logs under that tree's
`Testing` directory. They require `rclpy`, `sensor_msgs_py`, NumPy and YAML;
these test dependencies are declared in the package manifest. They do not
require a simulator, dataset or another repository.
