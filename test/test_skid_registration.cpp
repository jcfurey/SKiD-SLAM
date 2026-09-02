#include "skid_registration.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace {

using liorf::registration::PointCloud;

TEST(TruncatedMse, UsesSquaredMetresAndReportsOverlapSeparately) {
  const PointCloud source{
    Eigen::Vector3f(0.0F, 0.0F, 0.0F),
    Eigen::Vector3f(1.0F, 0.0F, 0.0F),
    Eigen::Vector3f(100.0F, 0.0F, 0.0F),
  };
  const PointCloud target{
    Eigen::Vector3f(0.1F, 0.0F, 0.0F),
    Eigen::Vector3f(1.2F, 0.0F, 0.0F),
  };

  const auto metric = liorf::registration::computeTruncatedMse(
    source, target, Eigen::Isometry3d::Identity(), 0.5);

  ASSERT_TRUE(metric.valid());
  EXPECT_EQ(metric.correspondence_count, 2U);
  EXPECT_EQ(metric.evaluated_source_count, 3U);
  EXPECT_NEAR(metric.value_m2, 0.025, 1.0e-6);
  EXPECT_NEAR(metric.overlap_ratio, 2.0 / 3.0, 1.0e-12);
}

TEST(TruncatedMse, AppliesTargetFromSourceTransform) {
  const PointCloud source{
    Eigen::Vector3f(-1.0F, 2.0F, 0.5F),
    Eigen::Vector3f(3.0F, -2.0F, 1.0F),
  };
  Eigen::Isometry3d target_from_source = Eigen::Isometry3d::Identity();
  target_from_source.linear() =
    Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  target_from_source.translation() = Eigen::Vector3d(4.0, -1.0, 2.0);

  PointCloud target;
  for (const auto& point : source) {
    target.push_back((target_from_source * point.cast<double>()).cast<float>());
  }

  const auto metric = liorf::registration::computeTruncatedMse(
    source, target, target_from_source, 0.1);
  ASSERT_TRUE(metric.valid());
  EXPECT_EQ(metric.correspondence_count, source.size());
  EXPECT_NEAR(metric.value_m2, 0.0, 1.0e-11);
  EXPECT_DOUBLE_EQ(metric.overlap_ratio, 1.0);
}

TEST(TruncatedMse, RejectsInvalidInputs) {
  const PointCloud point{Eigen::Vector3f::Zero()};
  EXPECT_FALSE(liorf::registration::computeTruncatedMse(
    {}, point, Eigen::Isometry3d::Identity(), 1.0).valid());
  EXPECT_FALSE(liorf::registration::computeTruncatedMse(
    point, {}, Eigen::Isometry3d::Identity(), 1.0).valid());
  EXPECT_FALSE(liorf::registration::computeTruncatedMse(
    point, point, Eigen::Isometry3d::Identity(), 0.0).valid());
}

TEST(RegistrationConfig, RejectsDimensionallyInvalidValues) {
  liorf::registration::Config config;
  EXPECT_TRUE(liorf::registration::validate(config).empty());

  config.max_truncated_mse_m2 = -1.0;
  EXPECT_FALSE(liorf::registration::validate(config).empty());

  config = liorf::registration::Config();
  config.coarse_voxel_size_m = 0.001F;
  EXPECT_FALSE(liorf::registration::validate(config).empty());

  config = liorf::registration::Config();
  config.nominal_translation_stddev_m = 0.0;
  EXPECT_FALSE(liorf::registration::validate(config).empty());
}

TEST(RegistrationConfig, DoesNotPermitSilentKissNoiseBoundClamping) {
  liorf::registration::Config config;
  ASSERT_TRUE(liorf::registration::validate(config).empty());

  config.coarse_robin_noise_bound_gain = 0.75F;
  config.coarse_solver_noise_bound_gain = 0.5F;
  EXPECT_NE(
    liorf::registration::validate(config).find("1.0 m clamp"),
    std::string::npos);

  config.coarse_clamp_noise_bounds = false;
  EXPECT_TRUE(liorf::registration::validate(config).empty());
}

TEST(RegistrationUncertainty, GivesIsotropicHessianConfiguredPhysicalUnits) {
  liorf::registration::Config config;
  config.nominal_rotation_stddev_rad = 0.1;
  config.nominal_translation_stddev_m = 0.5;
  config.uncertainty_reference_mse_m2 = 0.04;

  Eigen::Matrix<double, 6, 1> inverse_variances;
  inverse_variances << 100.0, 100.0, 100.0, 4.0, 4.0, 4.0;
  const liorf::registration::TruncatedMse metric{0.04, 100, 100, 1.0};
  const auto uncertainty = liorf::registration::estimatePoseUncertainty(
    inverse_variances.asDiagonal(), metric, config);

  ASSERT_TRUE(uncertainty.valid());
  EXPECT_NEAR(uncertainty.variance_scale, 1.0, 1.0e-12);
  EXPECT_NEAR(uncertainty.covariance(0, 0), 0.01, 1.0e-12);
  EXPECT_NEAR(uncertainty.covariance(3, 3), 0.25, 1.0e-12);
  EXPECT_TRUE((uncertainty.covariance * uncertainty.information)
    .isApprox(liorf::registration::Matrix6d::Identity(), 1.0e-10));
}

TEST(RegistrationUncertainty, InflatesWeakModesAndPartialOverlap) {
  liorf::registration::Config config;
  config.nominal_rotation_stddev_rad = 0.1;
  config.nominal_translation_stddev_m = 0.5;
  config.uncertainty_reference_mse_m2 = 0.04;
  config.uncertainty_min_information_ratio = 0.01;

  Eigen::Matrix<double, 6, 1> information_diagonal;
  information_diagonal << 1.0, 100.0, 100.0, 4.0, 4.0, 4.0;
  const liorf::registration::TruncatedMse metric{0.08, 50, 100, 0.5};
  const auto uncertainty = liorf::registration::estimatePoseUncertainty(
    information_diagonal.asDiagonal(), metric, config);

  ASSERT_TRUE(uncertainty.valid());
  EXPECT_NEAR(uncertainty.variance_scale, 4.0, 1.0e-12);
  EXPECT_NEAR(uncertainty.covariance(0, 0), 4.0, 1.0e-10);
  EXPECT_NEAR(uncertainty.covariance(1, 1), 0.04, 1.0e-12);
  EXPECT_EQ(uncertainty.clamped_modes, 0U);
  EXPECT_NEAR(uncertainty.condition_number, 100.0, 1.0e-10);
}

TEST(Registration, RejectsEmptyCloudBeforeInvokingSolvers) {
  const PointCloud point{Eigen::Vector3f::Zero()};
  const auto result = liorf::registration::registerClouds({}, point);
  EXPECT_EQ(result.status, liorf::registration::Status::kEmptySource);
  EXPECT_FALSE(result.accepted());
}

TEST(Registration, RecoversLargeRigidTransformWithCoarseToFinePipeline) {
  PointCloud source;
  source.reserve(2200);

  // A non-repeating terrain patch plus two non-parallel walls gives Faster-PFH
  // distinctive local geometry without relying on random test data.
  for (int ix = 0; ix < 50; ++ix) {
    const float x = -6.0F + 0.25F * static_cast<float>(ix);
    for (int iy = 0; iy < 34; ++iy) {
      const float y = -4.0F + 0.25F * static_cast<float>(iy);
      const float z = 0.18F * std::sin(0.7F * x) +
                      0.11F * std::cos(1.1F * y) + 0.006F * x * y;
      source.emplace_back(x, y, z);
    }
  }
  for (int iy = 0; iy < 38; ++iy) {
    const float y = -4.0F + 0.23F * static_cast<float>(iy);
    for (int iz = 0; iz < 14; ++iz) {
      const float z = 0.25F * static_cast<float>(iz);
      source.emplace_back(-2.7F + 0.05F * std::sin(0.8F * y), y, z);
    }
  }

  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.linear() =
    (Eigen::AngleAxisd(2.1, Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(-0.08, Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX())).toRotationMatrix();
  expected.translation() = Eigen::Vector3d(4.0, -2.5, 1.2);

  PointCloud target;
  target.reserve(source.size());
  for (const auto& point : source) {
    target.push_back((expected * point.cast<double>()).cast<float>());
  }

  liorf::registration::Config config;
  config.coarse_voxel_size_m = 0.5F;
  config.coarse_linearity_threshold = 1.0F;
  config.fine_downsampling_resolution_m = 0.25;
  config.fine_max_correspondence_distance_m = 1.0;
  config.fine_num_threads = 2;
  config.min_fine_inliers = 50;
  config.truncated_mse_max_correspondence_distance_m = 0.5;
  config.max_truncated_mse_m2 = 0.01;
  config.min_metric_inliers = 100;
  config.min_overlap_ratio = 0.90;

  const auto result =
    liorf::registration::registerClouds(source, target, config);
  ASSERT_TRUE(result.accepted())
    << liorf::registration::toString(result.status) << ": " << result.detail
    << ", correspondences=" << result.coarse_correspondences
    << ", coarse inliers=" << result.coarse_translation_inliers
    << ", fine inliers=" << result.fine_inliers;

  const double translation_error =
    (result.T_target_source.translation() - expected.translation()).norm();
  const Eigen::Matrix3d rotation_error =
    result.T_target_source.linear().transpose() * expected.linear();
  const double angle_error = Eigen::AngleAxisd(rotation_error).angle();
  EXPECT_LT(translation_error, 0.05);
  EXPECT_LT(angle_error, 0.01);
  EXPECT_LT(result.metric.value_m2, 1.0e-4);
  EXPECT_TRUE(result.uncertainty.valid());
  EXPECT_TRUE((result.uncertainty.covariance * result.uncertainty.information)
    .isApprox(liorf::registration::Matrix6d::Identity(), 1.0e-8));
}

}  // namespace
