#pragma once

#include <limits>

#include <Eigen/Core>
#include <gtsam/geometry/Pose3.h>

namespace liorf::uncertainty {

using Matrix6d = Eigen::Matrix<double, 6, 6>;

struct PoseWithCovariance {
  gtsam::Pose3 pose;
  Matrix6d covariance = Matrix6d::Zero();
};

struct PcmResidual {
  gtsam::Pose3 pose;
  gtsam::Vector6 tangent = gtsam::Vector6::Zero();
  Matrix6d covariance = Matrix6d::Zero();
  Matrix6d information = Matrix6d::Zero();
  double mahalanobis_distance = std::numeric_limits<double>::infinity();
  bool valid = false;
};

bool validCovariance(const Matrix6d& covariance) noexcept;
bool positiveDefiniteCovariance(const Matrix6d& covariance) noexcept;

// First-order propagation for independent lhs * rhs poses.
PoseWithCovariance compose(
  const gtsam::Pose3& lhs,
  const Matrix6d& lhs_covariance,
  const gtsam::Pose3& rhs,
  const Matrix6d& rhs_covariance);

// First-order propagation for an independent relative pose from -> to.
PoseWithCovariance between(
  const gtsam::Pose3& from,
  const Matrix6d& from_covariance,
  const gtsam::Pose3& to,
  const Matrix6d& to_covariance);

// Paper Equation (11), with uncertainty propagated through the full SE(3)
// composition. The two local trajectory transforms are treated as fixed;
// their aggregate uncertainty is represented by the dimensionally explicit
// rotational/translational standard-deviation floors.
PcmResidual pcmResidual(
  const gtsam::Pose3& inter_jk,
  const Matrix6d& inter_jk_covariance,
  const gtsam::Pose3& inter_il,
  const Matrix6d& inter_il_covariance,
  const gtsam::Pose3& inner_ij,
  const gtsam::Pose3& inner_kl,
  double local_rotation_stddev_rad,
  double local_translation_stddev_m);

}  // namespace liorf::uncertainty
