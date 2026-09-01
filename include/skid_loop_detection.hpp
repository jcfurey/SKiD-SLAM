#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

// Shared SOLiD place-recognition primitives.
//
// The paper uses one descriptor and one registration pipeline for intra- and
// inter-robot loops. This header holds the parts of that pipeline that need no
// ROS, PCL, or KD-tree dependency, so the local mapping node and the map-fusion
// node evaluate identical descriptor distances and identical candidate rules.
namespace liorf::loop_detection {

// R-SOLiD is the range-wise, yaw-invariant key used for retrieval. A-SOLiD is
// the angle-wise key whose circular shift recovers the yaw between two places.
struct Descriptor {
  Eigen::VectorXf range;
  Eigen::VectorXf angular;

  bool sized(int knn_feature_dim, int num_sectors) const noexcept;
  bool finite() const noexcept;
};

struct Config {
  int knn_feature_dim = 40;
  int num_sectors = 60;

  // Retrieval width over the yaw-invariant key.
  int num_nearest_matches = 50;
  // Candidates kept after the full SOLiD distance is evaluated.
  int num_match_candidates = 1;
  // Full SOLiD range distance accepted as a place match.
  float distance_threshold = 0.15F;

  // Revisit exclusion. A candidate closer than either gap describes the place
  // the platform is standing in, not a revisit of an earlier one.
  int min_index_gap = 30;
  double min_time_gap_s = 30.0;
};

// Returns an empty string when every configuration value is valid.
std::string validate(const Config& config);

struct Comparison {
  // 1 - cosine similarity of the two R-SOLiD keys, in [0, 2].
  float range_distance = std::numeric_limits<float>::infinity();
  // Circular A-SOLiD shift, in sectors, minimizing the L1 residual.
  int sector_shift = 0;
  float angular_distance = std::numeric_limits<float>::infinity();
  // Yaw of the rotation that maps query-frame coordinates into candidate-frame
  // coordinates, in (-pi, pi].
  float yaw_rad = 0.0F;

  bool valid() const noexcept;
};

// Yaw implied by a circular A-SOLiD shift. Sector index increases with
// azimuth and the shift carries query sector `i` onto candidate sector
// `i + shift`, so the shift is that many sector widths of azimuth rotation
// from the query frame into the candidate frame, wrapped into (-pi, pi].
float sectorShiftToYaw(int sector_shift, int num_sectors) noexcept;

// Cosine distance on R-SOLiD plus the minimum-L1 circular alignment of
// A-SOLiD. Returns an invalid comparison for mismatched, empty, non-finite, or
// zero-norm descriptors instead of producing a NaN distance.
Comparison compare(const Descriptor& query, const Descriptor& candidate);

struct Candidate {
  std::size_t index = 0;
  float range_distance = std::numeric_limits<float>::infinity();
  int sector_shift = 0;
  float yaw_rad = 0.0F;
};

// Descriptor database with exact nearest-neighbor retrieval.
//
// Retrieval ranks stored keys by Euclidean distance between L2-normalized
// R-SOLiD vectors, which is monotonic in the cosine distance reported by
// compare(). The shortlist therefore only bounds how many A-SOLiD circular
// alignments are evaluated; it never changes which candidate ranks first.
// Retrieval is linear in the number of stored keyframes, which at loop-closure
// rates costs far less than the registration it feeds.
class Index {
 public:
  explicit Index(const Config& config);

  // Non-empty when the configuration was rejected. A rejected index stores
  // nothing and returns no candidates.
  const std::string& error() const noexcept { return error_; }
  const Config& config() const noexcept { return config_; }
  std::size_t size() const noexcept { return descriptors_.size(); }

  // Appends a descriptor and returns its assigned index. Returns
  // std::size_t(-1) without storing anything when the descriptor is unusable.
  std::size_t add(const Descriptor& descriptor, double time_s);

  const Descriptor& descriptor(std::size_t index) const;
  double time(std::size_t index) const;

  // Revisit candidates for a place observed at `time_s` and stored (or about
  // to be stored) at `query_index`, best match first.
  std::vector<Candidate> search(
    const Descriptor& descriptor,
    double time_s,
    std::size_t query_index) const;

  // Convenience wrapper for the most recently added descriptor.
  std::vector<Candidate> searchLatest() const;

  void clear();

 private:
  Config config_;
  std::string error_;
  std::vector<Descriptor> descriptors_;
  std::vector<Eigen::VectorXf> normalized_range_;
  std::vector<double> times_;
};

}  // namespace liorf::loop_detection
