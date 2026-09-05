# Accepted loop factor delivered to one platform's distributed keyframe graph.
#
# robot_id is the recipient. The endpoint robot IDs distinguish locally owned
# poses from the sparse remote poses in paper Equation (6). Legacy same-robot
# producers set both endpoint IDs to robot_id.
std_msgs/Header header
# Revision authority is the observing map-fusion process. Epochs use system
# time (nanoseconds), never simulated ROS time. A later epoch replaces every
# claim from an earlier process with the same authority_id.
# revision == 0 denotes the legacy, non-retractable contract.
string authority_id
uint64 authority_epoch
uint64 revision
bool retracted
string robot_id
string from_robot_id
uint64 from_trajectory_epoch
int64 index_from
string to_robot_id
uint64 to_trajectory_epoch
int64 index_to

# Endpoint estimates in each endpoint owner's own map frame. Their absolute
# frames generally differ; only relative motion between poses belonging to the
# same robot is used to build the partial remote odometry graph.
geometry_msgs/Pose from_pose
geometry_msgs/Pose to_pose

# The relative pose maps the to endpoint into the from endpoint, matching the
# GTSAM BetweenFactor(from, to, measurement) convention used by the consumer.
# PoseWithCovariance uses the standard ROS row-major order:
# [x, y, z, rotation about X, rotation about Y, rotation about Z].
geometry_msgs/PoseWithCovariance relative_pose

# Registration diagnostics. A non-paper legacy pipeline may set unsupported
# fields to NaN/zero while still supplying a valid covariance.
float64 registration_error_m2
float64 overlap_ratio
uint64 registration_inliers
