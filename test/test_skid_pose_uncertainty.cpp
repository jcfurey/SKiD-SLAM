#include "skid_pose_uncertainty.hpp"

#include <gtest/gtest.h>

namespace {

using liorf::uncertainty::Matrix6d;

TEST(PoseUncertaintyPropagation, BetweenAddsIndependentIdentityCovariances) {
  Matrix6d first_covariance = Matrix6d::Identity();
  Matrix6d second_covariance = 2.0 * Matrix6d::Identity();
  const auto result = liorf::uncertainty::between(
    gtsam::Pose3(), first_covariance,
    gtsam::Pose3(), second_covariance);

  EXPECT_TRUE(result.pose.equals(gtsam::Pose3(), 1.0e-12));
  EXPECT_TRUE(result.covariance.isApprox(
    3.0 * Matrix6d::Identity(), 1.0e-12));
}

TEST(PcmUncertainty, ConsistentCycleHasZeroMahalanobisDistance) {
  const Matrix6d covariance = 0.01 * Matrix6d::Identity();
  const auto result = liorf::uncertainty::pcmResidual(
    gtsam::Pose3(), covariance,
    gtsam::Pose3(), covariance,
    gtsam::Pose3(), gtsam::Pose3(),
    0.05, 0.20);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.mahalanobis_distance, 0.0, 1.0e-12);
}

TEST(PcmUncertainty, UsesSeparateAngularAndLinearUnits) {
  const Matrix6d zero_covariance = Matrix6d::Zero();
  const gtsam::Pose3 translation_error(
    gtsam::Rot3(), gtsam::Point3(0.2, 0.0, 0.0));
  const gtsam::Pose3 rotation_error(
    gtsam::Rot3::Rx(0.1), gtsam::Point3(0.0, 0.0, 0.0));

  const auto translation = liorf::uncertainty::pcmResidual(
    translation_error, zero_covariance,
    gtsam::Pose3(), zero_covariance,
    gtsam::Pose3(), gtsam::Pose3(),
    0.05, 0.20);
  const auto rotation = liorf::uncertainty::pcmResidual(
    rotation_error, zero_covariance,
    gtsam::Pose3(), zero_covariance,
    gtsam::Pose3(), gtsam::Pose3(),
    0.05, 0.20);

  ASSERT_TRUE(translation.valid);
  ASSERT_TRUE(rotation.valid);
  EXPECT_NEAR(translation.mahalanobis_distance, 1.0, 1.0e-10);
  EXPECT_NEAR(rotation.mahalanobis_distance, 2.0, 1.0e-10);
}

}  // namespace
