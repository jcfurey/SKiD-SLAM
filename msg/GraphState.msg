# Pull-based correction exchange. Requests contain at most the configured
# batch size of indices and no poses. Responses contain matching indices and
# poses from the owner's LOCAL-ONLY graph, avoiding distributed feedback.
# A response with a different trajectory_epoch announces an owner restart;
# the recipient must discard factors tied to the old trajectory.
std_msgs/Header header
bool request
string owner_robot_id
string requester_robot_id
uint64 trajectory_epoch
uint64 revision
int64[] keyframe_indices
geometry_msgs/Pose[] poses
