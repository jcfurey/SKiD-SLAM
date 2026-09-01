# Robot Name
string robot_id

# Robot Name Receive
string robot_id_receive

# Cloud Info
std_msgs/Header header

# SOLiD dimension
int64 num_ring
int64 num_sector
int64 num_height

# Present robot pose
float32 pose_x
float32 pose_y
float32 pose_z
float32 pose_roll
float32 pose_pitch
float32 pose_yaw
float32 pose_intensity

# Keyframe index within the publishing robot's trajectory. Identifies this
# place for a later ScanRequest, so the scan itself need not be announced.
int64 keyframe_index

# Point cloud messages
#
# Empty when the announcement is descriptor-only, which is the default for
# peers: the receiver asks for the scan with a ScanRequest once a descriptor
# match warrants registration. Populated when mapfusion.comms.announce_scans
# is set, and always populated for the loopback announcement a node sends to
# itself, which costs no inter-robot bandwidth.
sensor_msgs/PointCloud2 scan_cloud  # extracted feature cloud

# SOLiD info
float32[] asolid
float32[] rsolid

# Scan context info
float32[] scan_context_bin
float32[] ring_key
