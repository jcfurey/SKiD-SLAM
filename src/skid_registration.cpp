#include "skid_registration.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <utility>

#include <Eigen/Eigenvalues>

#include <kiss_matcher/KISSMatcher.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration_helper.hpp>

namespace liorf::registration {
namespace {

using Clock = std::chrono::steady_clock;

double elapsedSeconds(const Clock::time_point& start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

bool finiteTransform(const Eigen::Isometry3d& transform) {
  if (!transform.matrix().allFinite()) {
    return false;
  }
  const double determinant = transform.linear().determinant();
  const double orthogonality_error =
    (transform.linear().transpose() * transform.linear() -
     Eigen::Matrix3d::Identity()).norm();
  return std::isfinite(determinant) && std::isfinite(orthogonality_error) &&
         std::abs(determinant - 1.0) < 1.0e-3 && orthogonality_error < 1.0e-3;
}

PointCloud finitePoints(const PointCloud& input) {
  PointCloud output;
  output.reserve(input.size());
  for (const auto& point : input) {
    if (point.allFinite()) {
      output.push_back(point);
    }
  }
  return output;
}

Result rejected(Status status, std::string detail) {
  Result result;
  result.status = status;
  result.detail = std::move(detail);
  return result;
}

}  // namespace

const char* toString(Status status) noexcept {
  switch (status) {
    case Status::kSuccess:
      return "success";
    case Status::kInvalidConfiguration:
      return "invalid_configuration";
    case Status::kEmptySource:
      return "empty_source";
    case Status::kEmptyTarget:
      return "empty_target";
    case Status::kInsufficientSourcePoints:
      return "insufficient_source_points";
    case Status::kInsufficientTargetPoints:
      return "insufficient_target_points";
    case Status::kInsufficientCoarseCorrespondences:
      return "insufficient_coarse_correspondences";
    case Status::kCoarseRegistrationFailed:
      return "coarse_registration_failed";
    case Status::kInvalidCoarseSolution:
      return "invalid_coarse_solution";
    case Status::kInsufficientCoarseInliers:
      return "insufficient_coarse_inliers";
    case Status::kFineRegistrationFailed:
      return "fine_registration_failed";
    case Status::kInsufficientFineInliers:
      return "insufficient_fine_inliers";
    case Status::kInvalidMetric:
      return "invalid_metric";
    case Status::kInsufficientMetricInliers:
      return "insufficient_metric_inliers";
    case Status::kInsufficientOverlap:
      return "insufficient_overlap";
    case Status::kTruncatedMseTooLarge:
      return "truncated_mse_too_large";
    case Status::kInvalidUncertainty:
      return "invalid_uncertainty";
  }
  return "unknown";
}

bool TruncatedMse::valid() const noexcept {
  return std::isfinite(value_m2) && correspondence_count > 0 &&
         evaluated_source_count > 0 && std::isfinite(overlap_ratio);
}

bool PoseUncertainty::valid() const noexcept {
  if (!covariance.allFinite() || !information.allFinite() ||
      !std::isfinite(variance_scale) || variance_scale < 1.0 ||
      !std::isfinite(condition_number) || condition_number < 1.0) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Matrix6d> covariance_solver(covariance);
  Eigen::SelfAdjointEigenSolver<Matrix6d> information_solver(information);
  return covariance_solver.info() == Eigen::Success &&
         information_solver.info() == Eigen::Success &&
         covariance_solver.eigenvalues().minCoeff() > 0.0 &&
         information_solver.eigenvalues().minCoeff() > 0.0;
}

std::string validate(const Config& config) {
  auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
  auto unitInterval = [](double value) {
    return std::isfinite(value) && value >= 0.0 && value <= 1.0;
  };

  if (!positive(config.coarse_voxel_size_m) || config.coarse_voxel_size_m < 5.0e-3F) {
    return "coarse_voxel_size_m must be finite and at least 0.005 m";
  }
  if (!std::isfinite(config.coarse_linearity_threshold) ||
      config.coarse_linearity_threshold < 0.0F ||
      config.coarse_linearity_threshold > 1.0F) {
    return "coarse_linearity_threshold must be in [0, 1]";
  }
  if (config.coarse_max_correspondences < 3) {
    return "coarse_max_correspondences must be at least 3";
  }
  if (!positive(config.coarse_normal_radius_gain) ||
      !positive(config.coarse_fpfh_radius_gain) ||
      !positive(config.coarse_robin_noise_bound_gain) ||
      !positive(config.coarse_solver_noise_bound_gain)) {
    return "coarse radius and noise-bound gains must be finite and positive";
  }
  if (config.coarse_solver_noise_bound_gain >
      config.coarse_robin_noise_bound_gain) {
    return "coarse_solver_noise_bound_gain cannot exceed coarse_robin_noise_bound_gain";
  }
  if (config.min_coarse_correspondences < 3 || config.min_coarse_inliers < 3) {
    return "coarse correspondence and inlier minima must be at least 3";
  }
  if (!positive(config.fine_downsampling_resolution_m) ||
      !positive(config.fine_max_correspondence_distance_m) ||
      !positive(config.fine_rotation_epsilon_rad) ||
      !positive(config.fine_translation_epsilon_m)) {
    return "fine registration distances and convergence tolerances must be finite and positive";
  }
  if (config.fine_num_neighbors < 3 || config.fine_num_threads < 1 ||
      config.fine_max_iterations < 1 || config.min_fine_inliers < 3) {
    return "fine registration counts are below their minimum values";
  }
  if (!positive(config.truncated_mse_max_correspondence_distance_m) ||
      !positive(config.max_truncated_mse_m2) || config.min_metric_inliers < 1 ||
      !unitInterval(config.min_overlap_ratio)) {
    return "truncated-MSE gate values are invalid";
  }
  if (!positive(config.nominal_rotation_stddev_rad) ||
      !positive(config.nominal_translation_stddev_m) ||
      !positive(config.uncertainty_reference_mse_m2) ||
      !positive(config.uncertainty_min_information_ratio) ||
      config.uncertainty_min_information_ratio > 1.0 ||
      !std::isfinite(config.uncertainty_max_variance_scale) ||
      config.uncertainty_max_variance_scale < 1.0) {
    return "registration uncertainty values are invalid";
  }
  return {};
}

TruncatedMse computeTruncatedMse(
  const PointCloud& source,
  const PointCloud& target,
  const Eigen::Isometry3d& T_target_source,
  double max_correspondence_distance_m) {
  TruncatedMse metric;
  if (source.empty() || target.empty() || !finiteTransform(T_target_source) ||
      !std::isfinite(max_correspondence_distance_m) ||
      max_correspondence_distance_m <= 0.0) {
    return metric;
  }

  const auto finite_target = finitePoints(target);
  if (finite_target.empty()) {
    return metric;
  }

  auto target_points = std::make_shared<small_gicp::PointCloud>(finite_target);
  small_gicp::KdTree<small_gicp::PointCloud> target_tree(target_points);
  const double max_distance_sq =
    max_correspondence_distance_m * max_correspondence_distance_m;
  double squared_error_sum = 0.0;

  for (const auto& point : source) {
    if (!point.allFinite()) {
      continue;
    }
    ++metric.evaluated_source_count;
    const Eigen::Vector3d transformed = T_target_source * point.cast<double>();
    Eigen::Vector4d query;
    query << transformed, 1.0;
    std::size_t index = 0;
    double squared_distance = std::numeric_limits<double>::infinity();
    if (target_tree.nearest_neighbor_search(query, &index, &squared_distance) == 0 ||
        !std::isfinite(squared_distance) || squared_distance > max_distance_sq) {
      continue;
    }
    squared_error_sum += squared_distance;
    ++metric.correspondence_count;
  }

  if (metric.evaluated_source_count > 0) {
    metric.overlap_ratio = static_cast<double>(metric.correspondence_count) /
                           static_cast<double>(metric.evaluated_source_count);
  }
  if (metric.correspondence_count > 0) {
    metric.value_m2 = squared_error_sum /
                      static_cast<double>(metric.correspondence_count);
  }
  return metric;
}

PoseUncertainty estimatePoseUncertainty(
  const Matrix6d& raw_information,
  const TruncatedMse& metric,
  const Config& config) {
  PoseUncertainty uncertainty;
  if (!validate(config).empty() || !raw_information.allFinite() ||
      !metric.valid()) {
    return uncertainty;
  }

  Eigen::Matrix<double, 6, 1> nominal_stddev;
  nominal_stddev <<
    config.nominal_rotation_stddev_rad,
    config.nominal_rotation_stddev_rad,
    config.nominal_rotation_stddev_rad,
    config.nominal_translation_stddev_m,
    config.nominal_translation_stddev_m,
    config.nominal_translation_stddev_m;
  const Matrix6d scale = nominal_stddev.asDiagonal();
  const Matrix6d symmetric_information =
    0.5 * (raw_information + raw_information.transpose());
  const Matrix6d dimensionless_information =
    scale * symmetric_information * scale;

  Eigen::SelfAdjointEigenSolver<Matrix6d> solver(dimensionless_information);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
      !solver.eigenvectors().allFinite()) {
    return uncertainty;
  }

  const double strongest_information = solver.eigenvalues().maxCoeff();
  if (!std::isfinite(strongest_information) || strongest_information <= 0.0) {
    return uncertainty;
  }
  if (solver.eigenvalues().minCoeff() < -1.0e-9 * strongest_information) {
    return uncertainty;
  }

  Eigen::Matrix<double, 6, 1> normalized_information =
    solver.eigenvalues() / strongest_information;
  const double weakest_information = normalized_information.minCoeff();
  uncertainty.condition_number = weakest_information > 0.0
    ? 1.0 / weakest_information
    : 1.0 / config.uncertainty_min_information_ratio;
  uncertainty.condition_number = std::min(
    uncertainty.condition_number,
    1.0 / config.uncertainty_min_information_ratio);

  for (Eigen::Index index = 0; index < normalized_information.size(); ++index) {
    if (normalized_information(index) <
        config.uncertainty_min_information_ratio) {
      normalized_information(index) =
        config.uncertainty_min_information_ratio;
      ++uncertainty.clamped_modes;
    }
  }

  const double residual_scale = std::max(
    1.0, metric.value_m2 / config.uncertainty_reference_mse_m2);
  const double overlap_scale = 1.0 / std::max(
    metric.overlap_ratio, std::numeric_limits<double>::epsilon());
  uncertainty.variance_scale = std::clamp(
    residual_scale * overlap_scale,
    1.0,
    config.uncertainty_max_variance_scale);

  const Matrix6d normalized_covariance =
    solver.eigenvectors() * normalized_information.cwiseInverse().asDiagonal() *
    solver.eigenvectors().transpose();
  const Matrix6d normalized_precision =
    solver.eigenvectors() * normalized_information.asDiagonal() *
    solver.eigenvectors().transpose();
  const Matrix6d inverse_scale = nominal_stddev.cwiseInverse().asDiagonal();

  uncertainty.covariance = uncertainty.variance_scale *
    scale * normalized_covariance * scale;
  uncertainty.information =
    (1.0 / uncertainty.variance_scale) *
    inverse_scale * normalized_precision * inverse_scale;
  uncertainty.covariance =
    0.5 * (uncertainty.covariance + uncertainty.covariance.transpose());
  uncertainty.information =
    0.5 * (uncertainty.information + uncertainty.information.transpose());
  return uncertainty;
}

Result registerClouds(
  const PointCloud& source_input,
  const PointCloud& target_input,
  const Config& config) {
  const std::string configuration_error = validate(config);
  if (!configuration_error.empty()) {
    return rejected(Status::kInvalidConfiguration, configuration_error);
  }
  if (source_input.empty()) {
    return rejected(Status::kEmptySource, "source cloud is empty");
  }
  if (target_input.empty()) {
    return rejected(Status::kEmptyTarget, "target cloud is empty");
  }

  const PointCloud source = finitePoints(source_input);
  const PointCloud target = finitePoints(target_input);
  if (source.size() < config.min_coarse_correspondences) {
    return rejected(
      Status::kInsufficientSourcePoints,
      "source cloud has too few finite points for coarse registration");
  }
  if (target.size() < config.min_coarse_correspondences) {
    return rejected(
      Status::kInsufficientTargetPoints,
      "target cloud has too few finite points for coarse registration");
  }

  Result result;
  result.source_points = source.size();
  result.target_points = target.size();

  const auto coarse_start = Clock::now();
  kiss_matcher::RegistrationSolution coarse_solution;
  try {
    const kiss_matcher::KISSMatcherConfig kiss_config(
      config.coarse_voxel_size_m,
      config.coarse_use_voxel_sampling,
      config.coarse_use_quatro,
      config.coarse_linearity_threshold,
      config.coarse_max_correspondences,
      config.coarse_normal_radius_gain,
      config.coarse_fpfh_radius_gain,
      config.coarse_robin_noise_bound_gain,
      config.coarse_solver_noise_bound_gain,
      config.coarse_clamp_noise_bounds);
    kiss_matcher::KISSMatcher matcher(kiss_config);
    const auto matched = matcher.match(source, target);
    const auto& source_matched = std::get<0>(matched);
    const auto& target_matched = std::get<1>(matched);
    result.coarse_correspondences = source_matched.size();
    if (source_matched.size() != target_matched.size() ||
        source_matched.size() < config.min_coarse_correspondences) {
      result.status = Status::kInsufficientCoarseCorrespondences;
      result.detail = "KISS-Matcher retained too few geometrically consistent correspondences";
      result.coarse_seconds = elapsedSeconds(coarse_start);
      return result;
    }

    Eigen::Matrix<double, 3, Eigen::Dynamic> source_matrix(3, source_matched.size());
    Eigen::Matrix<double, 3, Eigen::Dynamic> target_matrix(3, target_matched.size());
    for (std::size_t i = 0; i < source_matched.size(); ++i) {
      source_matrix.col(i) = source_matched[i].cast<double>();
      target_matrix.col(i) = target_matched[i].cast<double>();
    }
    coarse_solution = matcher.solve(source_matrix, target_matrix);
    result.coarse_rotation_inliers = matcher.getNumRotationInliers();
    result.coarse_translation_inliers = matcher.getNumFinalInliers();
  } catch (const std::exception& error) {
    result.status = Status::kCoarseRegistrationFailed;
    result.detail = std::string("KISS-Matcher failed: ") + error.what();
    result.coarse_seconds = elapsedSeconds(coarse_start);
    return result;
  }
  result.coarse_seconds = elapsedSeconds(coarse_start);

  Eigen::Isometry3d coarse_transform = Eigen::Isometry3d::Identity();
  coarse_transform.linear() = coarse_solution.rotation;
  coarse_transform.translation() = coarse_solution.translation;
  if (!coarse_solution.valid || !finiteTransform(coarse_transform)) {
    result.status = Status::kInvalidCoarseSolution;
    result.detail = "KISS-Matcher returned an invalid or non-rigid transform";
    return result;
  }
  if (result.coarse_rotation_inliers < config.min_coarse_inliers ||
      result.coarse_translation_inliers < config.min_coarse_inliers) {
    result.status = Status::kInsufficientCoarseInliers;
    result.detail = "KISS-Matcher solver retained too few inliers";
    return result;
  }

  const auto fine_start = Clock::now();
  small_gicp::RegistrationResult fine_result;
  try {
    const small_gicp::PointCloud source_raw(source);
    const small_gicp::PointCloud target_raw(target);
    auto source_data = small_gicp::preprocess_points(
      source_raw,
      config.fine_downsampling_resolution_m,
      config.fine_num_neighbors,
      config.fine_num_threads);
    auto target_data = small_gicp::preprocess_points(
      target_raw,
      config.fine_downsampling_resolution_m,
      config.fine_num_neighbors,
      config.fine_num_threads);

    small_gicp::RegistrationSetting fine_config;
    fine_config.type = small_gicp::RegistrationSetting::GICP;
    fine_config.downsampling_resolution = config.fine_downsampling_resolution_m;
    fine_config.max_correspondence_distance = config.fine_max_correspondence_distance_m;
    fine_config.rotation_eps = config.fine_rotation_epsilon_rad;
    fine_config.translation_eps = config.fine_translation_epsilon_m;
    fine_config.num_threads = config.fine_num_threads;
    fine_config.max_iterations = config.fine_max_iterations;

    fine_result = small_gicp::align(
      *target_data.first,
      *source_data.first,
      *target_data.second,
      coarse_transform,
      fine_config);
  } catch (const std::exception& error) {
    result.status = Status::kFineRegistrationFailed;
    result.detail = std::string("Small-GICP failed: ") + error.what();
    result.fine_seconds = elapsedSeconds(fine_start);
    return result;
  }
  result.fine_seconds = elapsedSeconds(fine_start);
  result.T_target_source = fine_result.T_target_source;
  result.fine_converged = fine_result.converged;
  result.fine_iterations = fine_result.iterations;
  result.fine_inliers = fine_result.num_inliers;
  result.fine_error = fine_result.error;
  if (!fine_result.converged || !finiteTransform(result.T_target_source)) {
    result.status = Status::kFineRegistrationFailed;
    result.detail = "Small-GICP did not converge to a finite rigid transform";
    return result;
  }
  if (result.fine_inliers < config.min_fine_inliers) {
    result.status = Status::kInsufficientFineInliers;
    result.detail = "Small-GICP retained too few inliers";
    return result;
  }

  const auto metric_start = Clock::now();
  result.metric = computeTruncatedMse(
    source,
    target,
    result.T_target_source,
    config.truncated_mse_max_correspondence_distance_m);
  result.metric_seconds = elapsedSeconds(metric_start);
  if (!result.metric.valid()) {
    result.status = Status::kInvalidMetric;
    result.detail = "truncated MSE could not be evaluated";
    return result;
  }
  if (result.metric.correspondence_count < config.min_metric_inliers) {
    result.status = Status::kInsufficientMetricInliers;
    result.detail = "truncated MSE retained too few correspondences";
    return result;
  }
  if (result.metric.overlap_ratio < config.min_overlap_ratio) {
    result.status = Status::kInsufficientOverlap;
    result.detail = "truncated MSE overlap is below the configured minimum";
    return result;
  }
  if (result.metric.value_m2 > config.max_truncated_mse_m2) {
    result.status = Status::kTruncatedMseTooLarge;
    result.detail = "truncated MSE exceeds the configured threshold";
    return result;
  }

  result.uncertainty = estimatePoseUncertainty(
    fine_result.H, result.metric, config);
  if (!result.uncertainty.valid()) {
    result.status = Status::kInvalidUncertainty;
    result.detail = "Small-GICP Hessian could not produce a valid pose covariance";
    return result;
  }

  result.status = Status::kSuccess;
  result.detail = "coarse-to-fine registration accepted";
  return result;
}

}  // namespace liorf::registration
