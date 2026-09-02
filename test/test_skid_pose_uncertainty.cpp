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

TEST(PcmUncertainty, ClosesANonIdentityTwoRobotCycle) {
  const gtsam::Pose3 world_from_a_i(
    gtsam::Rot3::RzRyRx(0.04, -0.02, 0.3),
    gtsam::Point3(1.0, -0.5, 0.2));
  const gtsam::Pose3 world_from_a_j(
    gtsam::Rot3::RzRyRx(0.02, 0.01, 0.8),
    gtsam::Point3(4.0, 1.0, 0.4));
  const gtsam::Pose3 b_from_b_k(
    gtsam::Rot3::RzRyRx(-0.03, 0.02, -0.4),
    gtsam::Point3(-2.0, 0.5, 0.1));
  const gtsam::Pose3 b_from_b_l(
    gtsam::Rot3::RzRyRx(0.01, -0.04, 0.2),
    gtsam::Point3(0.5, 3.0, -0.2));
  const gtsam::Pose3 world_from_b(
    gtsam::Rot3::RzRyRx(0.1, -0.15, 1.2),
    gtsam::Point3(8.0, -3.0, 1.0));

  const gtsam::Pose3 world_from_b_k = world_from_b * b_from_b_k;
  const gtsam::Pose3 world_from_b_l = world_from_b * b_from_b_l;
  const gtsam::Pose3 inter_jk = world_from_a_j.between(world_from_b_k);
  const gtsam::Pose3 inter_il = world_from_a_i.between(world_from_b_l);
  const gtsam::Pose3 inner_ij = world_from_a_i.between(world_from_a_j);
  const gtsam::Pose3 inner_kl = b_from_b_k.between(b_from_b_l);
  const Matrix6d covariance = 0.01 * Matrix6d::Identity();

  const auto result = liorf::uncertainty::pcmResidual(
    inter_jk, covariance, inter_il, covariance,
    inner_ij, inner_kl, 0.05, 0.20);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.pose.equals(gtsam::Pose3(), 1.0e-9));
  EXPECT_NEAR(result.mahalanobis_distance, 0.0, 1.0e-8);

  // Supplying source <- target (the registration helper's legacy storage
  // direction) does not close this cycle and must not be mistaken for PCM's
  // target <- source measurements.
  const auto reversed = liorf::uncertainty::pcmResidual(
    inter_jk.inverse(), covariance, inter_il.inverse(), covariance,
    inner_ij, inner_kl, 0.05, 0.20);
  ASSERT_TRUE(reversed.valid);
  EXPECT_GT(reversed.mahalanobis_distance, 1.0);
}

TEST(PcmUncertainty, AbsoluteTranslationGateBoundsInflatedCovariance) {
  liorf::uncertainty::PcmResidual residual;
  residual.valid = true;
  residual.mahalanobis_distance = 0.25;
  residual.pose = gtsam::Pose3(
    gtsam::Rot3(), gtsam::Point3(3.0, 0.0, 0.0));
  residual.tangent = gtsam::Pose3::Logmap(residual.pose);

  EXPECT_TRUE(liorf::uncertainty::pcmResidualPassesGate(
    residual, 1.0, 0.0, 0.0));
  EXPECT_FALSE(liorf::uncertainty::pcmResidualPassesGate(
    residual, 1.0, 1.0, 0.0));
  EXPECT_TRUE(liorf::uncertainty::pcmResidualPassesGate(
    residual, 1.0, 3.0, 0.0));
}

TEST(PcmUncertainty, AbsoluteRotationGateIsIndependent) {
  liorf::uncertainty::PcmResidual residual;
  residual.valid = true;
  residual.mahalanobis_distance = 0.25;
  residual.pose = gtsam::Pose3(
    gtsam::Rot3::Rz(0.6), gtsam::Point3(0.1, 0.0, 0.0));
  residual.tangent = gtsam::Pose3::Logmap(residual.pose);

  EXPECT_FALSE(liorf::uncertainty::pcmResidualPassesGate(
    residual, 1.0, 1.0, 0.5));
  EXPECT_TRUE(liorf::uncertainty::pcmResidualPassesGate(
    residual, 1.0, 1.0, 0.7));
  EXPECT_FALSE(liorf::uncertainty::pcmResidualPassesGate(
    residual, 0.2, 1.0, 0.7));
}

TEST(PcmUncertainty, ResidualGateRejectsInvalidLimits) {
  liorf::uncertainty::PcmResidual residual;
  residual.valid = true;
  residual.mahalanobis_distance = 0.0;
  residual.pose = gtsam::Pose3();
  residual.tangent.setZero();

  EXPECT_FALSE(liorf::uncertainty::pcmResidualPassesGate(
    residual, 1.0, -1.0, 0.0));
  EXPECT_FALSE(liorf::uncertainty::pcmResidualPassesGate(
    residual, 0.0, 0.0, 0.0));
}

}  // namespace

namespace {

gtsam::Pose3 samplePose() {
  return gtsam::Pose3(
    gtsam::Rot3::RzRyRx(0.3, -0.2, 1.1), gtsam::Point3(4.0, -1.5, 0.7));
}

liorf::uncertainty::Matrix6d sampleCovariance() {
  Eigen::Matrix<double, 6, 1> diagonal;
  diagonal << 0.01, 0.02, 0.03, 0.10, 0.20, 0.30;
  liorf::uncertainty::Matrix6d covariance = diagonal.asDiagonal();
  // A little cross-coupling so the adjoint actually has to do something.
  covariance(0, 3) = covariance(3, 0) = 0.005;
  return covariance;
}

// True when rhs - lhs is positive semidefinite, i.e. rhs is at least as
// uncertain as lhs in every direction.
bool atLeastAsUncertain(
  const liorf::uncertainty::Matrix6d& rhs,
  const liorf::uncertainty::Matrix6d& lhs) {
  const liorf::uncertainty::Matrix6d difference = rhs - lhs;
  Eigen::SelfAdjointEigenSolver<liorf::uncertainty::Matrix6d> solver(
    0.5 * (difference + difference.transpose()));
  return solver.info() == Eigen::Success &&
         solver.eigenvalues().minCoeff() >= -1.0e-9;
}

}  // namespace

TEST(SkidPoseUncertaintyInverse, InvertingTwiceRestoresPoseAndCovariance) {
  const gtsam::Pose3 pose = samplePose();
  const liorf::uncertainty::Matrix6d covariance = sampleCovariance();

  const auto once = liorf::uncertainty::inverse(pose, covariance);
  ASSERT_TRUE(liorf::uncertainty::validCovariance(once.covariance));
  const auto twice =
    liorf::uncertainty::inverse(once.pose, once.covariance);

  EXPECT_TRUE(twice.pose.equals(pose, 1.0e-9));
  EXPECT_NEAR(
    0.0, (twice.covariance - covariance).cwiseAbs().maxCoeff(), 1.0e-9);
}

TEST(SkidPoseUncertaintyInverse, PreservesTotalUncertaintyScale) {
  const auto result =
    liorf::uncertainty::inverse(samplePose(), sampleCovariance());
  // The adjoint is a similarity transform, so the covariance stays positive
  // definite and none of it is lost.
  EXPECT_TRUE(liorf::uncertainty::positiveDefiniteCovariance(result.covariance));
  EXPECT_GT(result.covariance.trace(), 0.0);
}

TEST(SkidPoseUncertaintyInverse, RejectsAnInvalidCovariance) {
  liorf::uncertainty::Matrix6d bad = sampleCovariance();
  bad(0, 0) = -1.0;
  const auto result = liorf::uncertainty::inverse(samplePose(), bad);
  EXPECT_FALSE(liorf::uncertainty::validCovariance(result.covariance));
}

TEST(SkidPoseUncertaintyCompose, AnUncertainLeftOperandOnlyAddsUncertainty) {
  // This is the property the cross-peer loop factor relies on: treating the
  // map alignment as exact can only understate the factor's covariance, so
  // carrying its uncertainty is the conservative direction.
  const gtsam::Pose3 alignment = samplePose();
  const gtsam::Pose3 measured = gtsam::Pose3(
    gtsam::Rot3::RzRyRx(-0.1, 0.4, 0.2), gtsam::Point3(-2.0, 3.0, 1.0));
  const liorf::uncertainty::Matrix6d measurement_covariance =
    sampleCovariance();

  const auto exact_alignment = liorf::uncertainty::compose(
    alignment, liorf::uncertainty::Matrix6d::Zero(),
    measured, measurement_covariance);
  const auto uncertain_alignment = liorf::uncertainty::compose(
    alignment, sampleCovariance(), measured, measurement_covariance);

  EXPECT_TRUE(exact_alignment.pose.equals(uncertain_alignment.pose, 1.0e-12));
  EXPECT_TRUE(atLeastAsUncertain(
    uncertain_alignment.covariance, exact_alignment.covariance));
  EXPECT_GT(
    uncertain_alignment.covariance.trace(),
    exact_alignment.covariance.trace());
}

TEST(SkidPoseUncertaintyCrossPeer, CarryingAlignmentUncertaintyIsConservative) {
  // Mirrors how a cross-peer loop factor is built when its two endpoints were
  // registered against different peers: each is brought back through that
  // peer's map alignment before the two are differenced.
  const gtsam::Pose3 alignment_this = samplePose();
  const gtsam::Pose3 alignment_that = gtsam::Pose3(
    gtsam::Rot3::RzRyRx(0.05, 0.15, -0.6), gtsam::Point3(-3.0, 2.0, -0.4));
  const gtsam::Pose3 endpoint_this = gtsam::Pose3(
    gtsam::Rot3::RzRyRx(0.2, 0.1, 0.3), gtsam::Point3(1.0, 2.0, 3.0));
  const gtsam::Pose3 endpoint_that = gtsam::Pose3(
    gtsam::Rot3::RzRyRx(-0.3, 0.25, 0.9), gtsam::Point3(5.0, -1.0, 0.5));
  const liorf::uncertainty::Matrix6d registration_covariance =
    sampleCovariance();

  const auto build = [&](const liorf::uncertainty::Matrix6d& alignment_cov) {
    const auto inverse_this =
      liorf::uncertainty::inverse(alignment_this, alignment_cov);
    const auto inverse_that =
      liorf::uncertainty::inverse(alignment_that, alignment_cov);
    const auto local_this = liorf::uncertainty::compose(
      inverse_this.pose, inverse_this.covariance,
      endpoint_this, registration_covariance);
    const auto local_that = liorf::uncertainty::compose(
      inverse_that.pose, inverse_that.covariance,
      endpoint_that, registration_covariance);
    return liorf::uncertainty::between(
      local_that.pose, local_that.covariance,
      local_this.pose, local_this.covariance);
  };

  const auto alignment_treated_as_exact =
    build(liorf::uncertainty::Matrix6d::Zero());
  const auto alignment_uncertainty_carried = build(sampleCovariance());

  // The measurement itself is unchanged; only its weight is.
  EXPECT_TRUE(alignment_uncertainty_carried.pose.equals(
    alignment_treated_as_exact.pose, 1.0e-12));
  EXPECT_TRUE(liorf::uncertainty::positiveDefiniteCovariance(
    alignment_uncertainty_carried.covariance));
  EXPECT_TRUE(atLeastAsUncertain(
    alignment_uncertainty_carried.covariance,
    alignment_treated_as_exact.covariance));
  EXPECT_GT(
    alignment_uncertainty_carried.covariance.trace(),
    alignment_treated_as_exact.covariance.trace());
}

TEST(SkidPoseUncertaintyCrossPeer, ADiagonalFloorIsAValidCovariance) {
  // The map-fusion node builds its alignment floor exactly this way.
  Eigen::Matrix<double, 6, 1> variances;
  const double rotation_variance = 0.05 * 0.05;
  const double translation_variance = 0.20 * 0.20;
  variances << rotation_variance, rotation_variance, rotation_variance,
    translation_variance, translation_variance, translation_variance;
  const liorf::uncertainty::Matrix6d floor(variances.asDiagonal());

  EXPECT_TRUE(liorf::uncertainty::positiveDefiniteCovariance(floor));
  EXPECT_NEAR(rotation_variance, floor(0, 0), 1.0e-15);
  EXPECT_NEAR(translation_variance, floor(5, 5), 1.0e-15);

  // Adding a marginal to the floor keeps it a usable covariance and can only
  // increase it, which is what makes the floor a floor.
  const liorf::uncertainty::Matrix6d floored = floor + sampleCovariance();
  EXPECT_TRUE(liorf::uncertainty::positiveDefiniteCovariance(floored));
  EXPECT_TRUE(atLeastAsUncertain(floored, floor));
}
