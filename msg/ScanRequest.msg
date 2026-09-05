# Request for one peer keyframe's feature scan.
#
# Announcements (ContextInfo) carry the SOLiD descriptor only. A scan moves
# between robots only when a descriptor match has already justified attempting
# registration, which is what keeps steady-state inter-robot traffic
# proportional to the descriptor rather than to the point cloud.
std_msgs/Header header

# Requesting robot.
string robot_id

# Robot that owns the scan.
string robot_id_receive

# Keyframe index within the owning robot's trajectory.
int64 keyframe_index
uint64 trajectory_epoch
