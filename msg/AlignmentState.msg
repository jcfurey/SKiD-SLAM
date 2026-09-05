# Revisioned fleet alignment, including explicit withdrawal. The accompanying
# Odometry retains the existing frame convention. Empty/invalid geometry is
# permitted only when valid is false.
string authority_id
uint64 authority_epoch
uint64 revision
bool valid
nav_msgs/Odometry alignment
