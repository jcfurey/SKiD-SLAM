# Accepted loop factor for one platform's keyframe graph.
#
# The relative pose maps index_to into index_from, matching the GTSAM
# BetweenFactor(index_from, index_to, measurement) convention used by the
# consumer. PoseWithCovariance uses the standard ROS row-major order:
# [x, y, z, rotation about X, rotation about Y, rotation about Z].
std_msgs/Header header
string robot_id
int64 index_from
int64 index_to
geometry_msgs/PoseWithCovariance relative_pose

# Registration diagnostics. A non-paper legacy pipeline may set unsupported
# fields to NaN/zero while still supplying a valid covariance.
float64 registration_error_m2
float64 overlap_ratio
uint64 registration_inliers
