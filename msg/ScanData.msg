# One keyframe's feature scan, sent in reply to a ScanRequest.
std_msgs/Header header

# Robot that owns the scan.
string robot_id

# Robot that requested it.
string robot_id_receive

# Keyframe index within the owning robot's trajectory.
int64 keyframe_index
uint64 trajectory_epoch

# False when the owner no longer holds this scan, for example because its own
# bounded cache has evicted it. The requester should stop asking rather than
# spending its retry budget. scan_cloud is empty in that case.
bool available

sensor_msgs/PointCloud2 scan_cloud
