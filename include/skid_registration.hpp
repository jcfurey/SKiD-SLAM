#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace liorf::registration {

using PointCloud = std::vector<Eigen::Vector3f>;

enum class Status {
  kSuccess,
  kInvalidConfiguration,
  kEmptySource,
  kEmptyTarget,
  kInsufficientSourcePoints,
  kInsufficientTargetPoints,
  kInsufficientCoarseCorrespondences,
  kCoarseRegistrationFailed,
  kInvalidCoarseSolution,
  kInsufficientCoarseInliers,
  kFineRegistrationFailed,
  kInsufficientFineInliers,
  kInvalidMetric,
  kInsufficientMetricInliers,
  kInsufficientOverlap,
  kTruncatedMseTooLarge,
};

const char* toString(Status status) noexcept;

struct Config {
  // KISS-Matcher coarse registration.
  float coarse_voxel_size_m = 2.0F;
  bool coarse_use_voxel_sampling = true;
  bool coarse_use_quatro = false;
  float coarse_linearity_threshold = 0.99F;
  int coarse_max_correspondences = 5000;
  float coarse_normal_radius_gain = 3.5F;
  float coarse_fpfh_radius_gain = 5.0F;
  float coarse_robin_noise_bound_gain = 1.0F;
  float coarse_solver_noise_bound_gain = 0.75F;
  bool coarse_clamp_noise_bounds = true;
  std::size_t min_coarse_correspondences = 5;
  std::size_t min_coarse_inliers = 3;

  // Small-GICP fine registration.
  double fine_downsampling_resolution_m = 0.5;
  double fine_max_correspondence_distance_m = 2.0;
  double fine_rotation_epsilon_rad = 0.1 * 3.14159265358979323846 / 180.0;
  double fine_translation_epsilon_m = 1.0e-3;
  int fine_num_neighbors = 10;
  int fine_num_threads = 4;
  int fine_max_iterations = 30;
  std::size_t min_fine_inliers = 20;

  // Equation (10) gate. The distance is in metres and the score is in m^2.
  double truncated_mse_max_correspondence_distance_m = 3.0;
  double max_truncated_mse_m2 = 3.0;
  std::size_t min_metric_inliers = 20;
  double min_overlap_ratio = 0.10;
};

struct TruncatedMse {
  double value_m2 = std::numeric_limits<double>::infinity();
  std::size_t correspondence_count = 0;
  std::size_t evaluated_source_count = 0;
  double overlap_ratio = 0.0;

  bool valid() const noexcept;
};

struct Result {
  Status status = Status::kInvalidConfiguration;
  std::string detail;
  Eigen::Isometry3d T_target_source = Eigen::Isometry3d::Identity();

  std::size_t source_points = 0;
  std::size_t target_points = 0;
  std::size_t coarse_correspondences = 0;
  std::size_t coarse_rotation_inliers = 0;
  std::size_t coarse_translation_inliers = 0;
  std::size_t fine_inliers = 0;
  bool fine_converged = false;
  std::size_t fine_iterations = 0;
  double fine_error = std::numeric_limits<double>::infinity();
  TruncatedMse metric;

  double coarse_seconds = 0.0;
  double fine_seconds = 0.0;
  double metric_seconds = 0.0;

  bool accepted() const noexcept { return status == Status::kSuccess; }
};

// Returns an empty string when every configuration value is valid.
std::string validate(const Config& config);

// Computes the paper's truncated mean squared nearest-neighbor error after
// applying T_target_source. Points beyond max_correspondence_distance_m are
// omitted from the mean and represented separately by overlap_ratio.
TruncatedMse computeTruncatedMse(
  const PointCloud& source,
  const PointCloud& target,
  const Eigen::Isometry3d& T_target_source,
  double max_correspondence_distance_m);

// Coarse global registration with KISS-Matcher followed by Small-GICP fine
// alignment. Both transforms use the explicit target <- source convention.
Result registerClouds(
  const PointCloud& source,
  const PointCloud& target,
  const Config& config = Config());

}  // namespace liorf::registration
