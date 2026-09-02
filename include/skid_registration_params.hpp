#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>

#include "skid_registration.hpp"

namespace liorf::registration {

// Declares the paper's registration parameters under `prefix` and returns the
// validated configuration.
//
// `declare` must be callable as declare(name, default_value), returning the
// parameter value with the same type as the default. Both loop-closure paths
// call this, so intra- and inter-robot registration is gated by one parameter
// set and the two nodes cannot drift apart on names, defaults, or validation.
template <typename Declare>
Config declareConfig(
  const Declare& declare,
  const std::string& prefix,
  double max_truncated_mse_default) {
  Config config;

  // KISS-Matcher coarse global alignment.
  config.coarse_voxel_size_m =
    static_cast<float>(declare(prefix + "coarse_voxel_size_m", 2.0));
  config.coarse_use_voxel_sampling =
    declare(prefix + "coarse_use_voxel_sampling", true);
  config.coarse_use_quatro = declare(prefix + "coarse_use_quatro", false);
  config.coarse_linearity_threshold =
    static_cast<float>(declare(prefix + "coarse_linearity_threshold", 0.99));
  config.coarse_max_correspondences =
    declare(prefix + "coarse_max_correspondences", 5000);
  config.coarse_normal_radius_gain =
    static_cast<float>(declare(prefix + "coarse_normal_radius_gain", 3.5));
  config.coarse_fpfh_radius_gain =
    static_cast<float>(declare(prefix + "coarse_fpfh_radius_gain", 5.0));
  config.coarse_robin_noise_bound_gain = static_cast<float>(
    declare(prefix + "coarse_robin_noise_bound_gain", 0.5));
  config.coarse_solver_noise_bound_gain = static_cast<float>(
    declare(prefix + "coarse_solver_noise_bound_gain", 0.375));
  config.coarse_clamp_noise_bounds =
    declare(prefix + "coarse_clamp_noise_bounds", true);

  // Small-GICP fine alignment.
  config.fine_downsampling_resolution_m =
    declare(prefix + "fine_downsampling_resolution_m", 0.5);
  config.fine_max_correspondence_distance_m =
    declare(prefix + "fine_max_correspondence_distance_m", 2.0);
  config.fine_rotation_epsilon_rad =
    declare(prefix + "fine_rotation_epsilon_rad", 0.0017453292519943296);
  config.fine_translation_epsilon_m =
    declare(prefix + "fine_translation_epsilon_m", 0.001);
  config.fine_num_neighbors = declare(prefix + "fine_num_neighbors", 10);
  config.fine_num_threads = declare(prefix + "fine_num_threads", 4);
  config.fine_max_iterations = declare(prefix + "fine_max_iterations", 30);

  // Paper Equation (10): the distance is in metres and the score in m^2.
  config.truncated_mse_max_correspondence_distance_m =
    declare(prefix + "truncated_mse_max_correspondence_distance_m", 3.0);
  config.max_truncated_mse_m2 =
    declare(prefix + "max_truncated_mse_m2", max_truncated_mse_default);
  config.min_overlap_ratio = declare(prefix + "min_overlap_ratio", 0.10);

  // Full factor covariance: the Small-GICP Hessian supplies directional
  // observability, while these values calibrate its physical scale.
  config.nominal_rotation_stddev_rad =
    declare(prefix + "nominal_rotation_stddev_rad", 0.05);
  config.nominal_translation_stddev_m =
    declare(prefix + "nominal_translation_stddev_m", 0.20);
  config.uncertainty_reference_mse_m2 =
    declare(prefix + "uncertainty_reference_mse_m2", 0.04);
  config.uncertainty_min_information_ratio =
    declare(prefix + "uncertainty_min_information_ratio", 0.01);
  config.uncertainty_max_variance_scale =
    declare(prefix + "uncertainty_max_variance_scale", 100.0);

  // Counts are declared signed so a negative value is reported rather than
  // wrapping into an unreachable std::size_t threshold.
  const int min_coarse_correspondences =
    declare(prefix + "min_coarse_correspondences", 5);
  const int min_coarse_inliers = declare(prefix + "min_coarse_inliers", 3);
  const int min_fine_inliers = declare(prefix + "min_fine_inliers", 20);
  const int min_metric_inliers = declare(prefix + "min_metric_inliers", 20);
  if (min_coarse_correspondences < 0 || min_coarse_inliers < 0 ||
      min_fine_inliers < 0 || min_metric_inliers < 0) {
    throw std::invalid_argument(
      prefix + "* registration inlier counts cannot be negative");
  }
  config.min_coarse_correspondences =
    static_cast<std::size_t>(min_coarse_correspondences);
  config.min_coarse_inliers = static_cast<std::size_t>(min_coarse_inliers);
  config.min_fine_inliers = static_cast<std::size_t>(min_fine_inliers);
  config.min_metric_inliers = static_cast<std::size_t>(min_metric_inliers);

  const std::string error = validate(config);
  if (!error.empty()) {
    throw std::invalid_argument(
      prefix + "* registration parameters are invalid: " + error);
  }
  return config;
}

}  // namespace liorf::registration
