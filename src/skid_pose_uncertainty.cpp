#include "skid_pose_uncertainty.hpp"

#include <cmath>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

namespace liorf::uncertainty {
namespace {

Matrix6d symmetric(const Matrix6d& matrix) {
  return 0.5 * (matrix + matrix.transpose());
}

}  // namespace

bool validCovariance(const Matrix6d& covariance) noexcept {
  if (!covariance.allFinite()) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Matrix6d> solver(symmetric(covariance));
  return solver.info() == Eigen::Success &&
         solver.eigenvalues().minCoeff() >= -1.0e-12;
}

bool positiveDefiniteCovariance(const Matrix6d& covariance) noexcept {
  if (!covariance.allFinite()) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Matrix6d> solver(symmetric(covariance));
  return solver.info() == Eigen::Success &&
         solver.eigenvalues().minCoeff() > 0.0;
}

PoseWithCovariance compose(
  const gtsam::Pose3& lhs,
  const Matrix6d& lhs_covariance,
  const gtsam::Pose3& rhs,
  const Matrix6d& rhs_covariance) {
  PoseWithCovariance result;
  if (!validCovariance(lhs_covariance) || !validCovariance(rhs_covariance)) {
    result.covariance = Matrix6d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    return result;
  }

  gtsam::Matrix6 H_lhs;
  gtsam::Matrix6 H_rhs;
  result.pose = lhs.compose(rhs, H_lhs, H_rhs);
  result.covariance = symmetric(
    H_lhs * lhs_covariance * H_lhs.transpose() +
    H_rhs * rhs_covariance * H_rhs.transpose());
  return result;
}

PoseWithCovariance inverse(
  const gtsam::Pose3& pose,
  const Matrix6d& covariance) {
  PoseWithCovariance result;
  if (!validCovariance(covariance)) {
    result.covariance = Matrix6d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    return result;
  }

  gtsam::Matrix6 H;
  result.pose = pose.inverse(H);
  result.covariance = symmetric(H * covariance * H.transpose());
  return result;
}

PoseWithCovariance between(
  const gtsam::Pose3& from,
  const Matrix6d& from_covariance,
  const gtsam::Pose3& to,
  const Matrix6d& to_covariance) {
  PoseWithCovariance result;
  if (!validCovariance(from_covariance) || !validCovariance(to_covariance)) {
    result.covariance = Matrix6d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    return result;
  }

  gtsam::Matrix6 H_from;
  gtsam::Matrix6 H_to;
  result.pose = from.between(to, H_from, H_to);
  result.covariance = symmetric(
    H_from * from_covariance * H_from.transpose() +
    H_to * to_covariance * H_to.transpose());
  return result;
}

PcmResidual pcmResidual(
  const gtsam::Pose3& inter_jk,
  const Matrix6d& inter_jk_covariance,
  const gtsam::Pose3& inter_il,
  const Matrix6d& inter_il_covariance,
  const gtsam::Pose3& inner_ij,
  const gtsam::Pose3& inner_kl,
  double local_rotation_stddev_rad,
  double local_translation_stddev_m) {
  PcmResidual result;
  if (!validCovariance(inter_jk_covariance) ||
      !validCovariance(inter_il_covariance) ||
      !std::isfinite(local_rotation_stddev_rad) ||
      !std::isfinite(local_translation_stddev_m) ||
      local_rotation_stddev_rad <= 0.0 || local_translation_stddev_m <= 0.0) {
    return result;
  }

  // A default-constructed OptionalJacobian is the portable way to skip one:
  // nullptr is ambiguous between OptionalJacobian's pointer constructors.
  gtsam::Matrix6 H_first_inter;
  const gtsam::Pose3 first = inner_ij.compose(
    inter_jk, {}, H_first_inter);

  gtsam::Matrix6 H_second_first;
  const gtsam::Pose3 second = first.compose(
    inner_kl, H_second_first, {});

  const gtsam::Matrix6 H_inverse_inter = -inter_il.AdjointMap();
  const gtsam::Pose3 inverse_inter = inter_il.inverse();

  gtsam::Matrix6 H_residual_second;
  gtsam::Matrix6 H_residual_inverse;
  result.pose = second.compose(
    inverse_inter, H_residual_second, H_residual_inverse);

  const gtsam::Matrix6 J_jk =
    H_residual_second * H_second_first * H_first_inter;
  const gtsam::Matrix6 J_il =
    H_residual_inverse * H_inverse_inter;
  Matrix6d pose_covariance =
    J_jk * inter_jk_covariance * J_jk.transpose() +
    J_il * inter_il_covariance * J_il.transpose();

  Eigen::Matrix<double, 6, 1> floor_variances;
  floor_variances <<
    local_rotation_stddev_rad * local_rotation_stddev_rad,
    local_rotation_stddev_rad * local_rotation_stddev_rad,
    local_rotation_stddev_rad * local_rotation_stddev_rad,
    local_translation_stddev_m * local_translation_stddev_m,
    local_translation_stddev_m * local_translation_stddev_m,
    local_translation_stddev_m * local_translation_stddev_m;
  pose_covariance += floor_variances.asDiagonal();

  gtsam::Matrix6 H_log;
  result.tangent = gtsam::Pose3::Logmap(result.pose, H_log);
  result.covariance = symmetric(H_log * pose_covariance * H_log.transpose());
  if (!validCovariance(result.covariance)) {
    return result;
  }

  Eigen::LLT<Matrix6d> decomposition(result.covariance);
  if (decomposition.info() != Eigen::Success) {
    return result;
  }
  result.information = decomposition.solve(Matrix6d::Identity());
  const double squared_distance =
    result.tangent.dot(result.information * result.tangent);
  if (!std::isfinite(squared_distance) || squared_distance < -1.0e-12) {
    return result;
  }
  result.mahalanobis_distance = std::sqrt(std::max(0.0, squared_distance));
  result.valid = true;
  return result;
}

bool pcmResidualPassesGate(
  const PcmResidual& residual,
  double max_mahalanobis_distance,
  double max_translation_residual_m,
  double max_rotation_residual_rad) noexcept {
  if (!residual.valid ||
      !std::isfinite(residual.mahalanobis_distance) ||
      !std::isfinite(max_mahalanobis_distance) ||
      !std::isfinite(max_translation_residual_m) ||
      !std::isfinite(max_rotation_residual_rad) ||
      max_mahalanobis_distance <= 0.0 ||
      max_translation_residual_m < 0.0 ||
      max_rotation_residual_rad < 0.0 ||
      residual.mahalanobis_distance >= max_mahalanobis_distance) {
    return false;
  }

  const double translation_residual_m = residual.pose.translation().norm();
  const double rotation_residual_rad = residual.tangent.head<3>().norm();
  if (!std::isfinite(translation_residual_m) ||
      !std::isfinite(rotation_residual_rad)) {
    return false;
  }
  return (max_translation_residual_m == 0.0 ||
          translation_residual_m <= max_translation_residual_m) &&
         (max_rotation_residual_rad == 0.0 ||
          rotation_residual_rad <= max_rotation_residual_rad);
}

}  // namespace liorf::uncertainty
