# Structured trace of one inter-robot loop candidate as it moves through
# descriptor retrieval, scan acquisition, registration, and PCM.
std_msgs/Header header

# The map-fusion node that evaluated the candidate.
string observer_robot_id

# Candidate identity. query is the newly processed place; match is the
# retrieved place. Times are the original keyframe timestamps in seconds.
string query_robot_id
int64 query_keyframe_index
float64 query_time
string match_robot_id
int64 match_keyframe_index
float64 match_time

# Descriptor retrieval metadata. Ranks are one-based. descriptor_distance is
# NaN for position-search fallbacks that did not compare descriptors.
uint64 candidate_rank
float64 descriptor_distance
int32 sector_shift

# stage: descriptor, scan, registration, or pcm.
# decision: selected, deferred, accepted, or rejected.
string stage
string decision
string reason

# Registration result. relative_pose maps the match frame into the query
# frame, which is the convention consumed by the evaluation harness.
bool registration_attempted
bool registration_accepted
string registration_status
string registration_detail
geometry_msgs/PoseWithCovariance relative_pose
uint64 source_points
uint64 target_points
uint64 coarse_correspondences
uint64 coarse_rotation_inliers
uint64 coarse_translation_inliers
uint64 fine_inliers
bool fine_converged
uint64 fine_iterations
float64 fine_error
uint64 metric_inliers
float64 overlap_ratio
float64 truncated_mse_m2
float64 uncertainty_variance_scale
float64 uncertainty_condition_number
uint64 uncertainty_clamped_modes
float64 coarse_time_ms
float64 fine_time_ms
float64 metric_time_ms

# Pairwise-consistency maximization result. Every registered candidate gets a
# PCM event whenever the clique is recomputed.
bool pcm_evaluated
bool pcm_accepted
# True once the candidate has satisfied the configured publication policy.
# Commitments are monotonic because the current back-end cannot retract a
# factor that was already published.
bool pcm_committed
uint64 pcm_clique_size
