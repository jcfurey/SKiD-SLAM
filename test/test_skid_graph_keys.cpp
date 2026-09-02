#include "skid_graph_keys.hpp"
#include "skid_remote_graph.hpp"

#include <gtest/gtest.h>

#include <limits>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace {

TEST(SkidGraphKeys, LocalPosesUseExplicitXSymbolSpace) {
  const gtsam::Key key = liorf::graph_keys::localPose(42);

  EXPECT_TRUE(liorf::graph_keys::isLocalPose(key));
  EXPECT_EQ(gtsam::Symbol(key).chr(), 'x');
  EXPECT_EQ(gtsam::Symbol(key).index(), 42U);
  EXPECT_NE(key, static_cast<gtsam::Key>(42));
}

TEST(SkidGraphKeys, LocalPoseLookupIgnoresUnrelatedGraphValues) {
  gtsam::Values values;
  const gtsam::Pose3 first(
    gtsam::Rot3::RzRyRx(0.1, 0.2, 0.3),
    gtsam::Point3(1.0, 2.0, 3.0));
  const gtsam::Pose3 second(
    gtsam::Rot3::RzRyRx(-0.1, -0.2, -0.3),
    gtsam::Point3(4.0, 5.0, 6.0));

  values.insert(liorf::graph_keys::localPose(0), first);
  values.insert(gtsam::Symbol('r', 9001), gtsam::Pose3());
  values.insert(liorf::graph_keys::localPose(1), second);

  ASSERT_EQ(values.size(), 3U);
  EXPECT_TRUE(values.at<gtsam::Pose3>(
    liorf::graph_keys::localPose(0)).equals(first));
  EXPECT_TRUE(values.at<gtsam::Pose3>(
    liorf::graph_keys::localPose(1)).equals(second));
}

TEST(SkidGraphKeys, PeerRobotsAndKeyframesHaveDistinctRemoteKeys) {
  liorf::graph_keys::KeySpace keys("alpha");

  const gtsam::Key local = keys.pose("alpha", 7);
  const gtsam::Key beta_7 = keys.pose("beta", 7);
  const gtsam::Key beta_8 = keys.pose("beta", 8);
  const gtsam::Key gamma_7 = keys.pose("gamma", 7);

  EXPECT_EQ(local, liorf::graph_keys::localPose(7));
  EXPECT_TRUE(liorf::graph_keys::isRemotePose(beta_7));
  EXPECT_NE(beta_7, beta_8);
  EXPECT_NE(beta_7, gamma_7);
  EXPECT_EQ(keys.pose("beta", 7), beta_7);
  EXPECT_EQ(keys.remoteRobotCount(), 2U);
}

TEST(SkidGraphKeys, RejectsRemoteIndicesThatCannotBePackedLosslessly) {
  liorf::graph_keys::KeySpace keys("alpha");
  EXPECT_THROW(
    keys.pose("beta",
      static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()) + 1),
    std::out_of_range);
}

TEST(SkidRemoteGraph, BuildsASparsePeerTrajectoryOutOfOrder) {
  liorf::remote_graph::TrajectoryStore trajectory;
  const gtsam::Pose3 pose_10(
    gtsam::Rot3(), gtsam::Point3(10.0, 0.0, 0.0));
  const gtsam::Pose3 pose_20(
    gtsam::Rot3(), gtsam::Point3(20.0, 0.0, 0.0));
  const gtsam::Pose3 pose_15(
    gtsam::Rot3(), gtsam::Point3(15.0, 0.0, 0.0));

  EXPECT_FALSE(trajectory.insert("beta", 10, pose_10).has_value());
  const auto second = trajectory.insert("beta", 20, pose_20);
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->index_from, 10U);
  EXPECT_EQ(second->index_to, 20U);
  EXPECT_TRUE(second->measurement.equals(pose_10.between(pose_20)));

  const auto middle = trajectory.insert("beta", 15, pose_15);
  ASSERT_TRUE(middle.has_value());
  EXPECT_EQ(middle->index_from, 10U);
  EXPECT_EQ(middle->index_to, 15U);
  EXPECT_EQ(trajectory.size("beta"), 3U);
  EXPECT_FALSE(trajectory.insert("beta", 15, pose_15).has_value());
}

TEST(SkidRemoteGraph, SeedsEitherFactorOrientationConsistently) {
  const gtsam::Pose3 local(
    gtsam::Rot3::RzRyRx(0.1, -0.2, 0.3),
    gtsam::Point3(4.0, -2.0, 1.0));
  const gtsam::Pose3 remote(
    gtsam::Rot3::RzRyRx(-0.3, 0.1, 0.2),
    gtsam::Point3(7.0, 5.0, -1.0));

  const gtsam::Pose3 local_to_remote = local.between(remote);
  const gtsam::Pose3 remote_to_local = remote.between(local);
  EXPECT_TRUE(liorf::remote_graph::initialRemotePose(
    local, local_to_remote, true).equals(remote, 1.0e-12));
  EXPECT_TRUE(liorf::remote_graph::initialRemotePose(
    local, remote_to_local, false).equals(remote, 1.0e-12));
}

TEST(SkidRemoteGraph, DeduplicatesReversedEndpointOrientation) {
  const auto forward = liorf::remote_graph::canonicalFactorIdentity(
    "alpha", 12, "beta", 34);
  const auto reverse = liorf::remote_graph::canonicalFactorIdentity(
    "beta", 34, "alpha", 12);
  const auto different = liorf::remote_graph::canonicalFactorIdentity(
    "alpha", 13, "beta", 34);

  EXPECT_EQ(forward, reverse);
  EXPECT_NE(forward, different);
}

TEST(SkidRemoteGraph, DirectFactorsCanCorrectTheLocalTrajectory) {
  liorf::graph_keys::KeySpace keys("alpha");
  const gtsam::Key x0 = keys.pose("alpha", 0);
  const gtsam::Key x1 = keys.pose("alpha", 1);
  const gtsam::Key r0 = keys.pose("beta", 0);
  const gtsam::Key r1 = keys.pose("beta", 1);

  const auto tight = gtsam::noiseModel::Isotropic::Sigma(6, 1.0e-3);
  const auto odometry = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(x0, gtsam::Pose3(), tight));
  graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
    x0, x1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(10.0, 0.0, 0.0)),
    odometry));
  graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
    r0, r1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(8.0, 0.0, 0.0)),
    odometry));
  graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
    x0, r0, gtsam::Pose3(), tight));
  graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
    x1, r1, gtsam::Pose3(), tight));

  gtsam::Values initial;
  initial.insert(x0, gtsam::Pose3());
  initial.insert(x1,
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(10.0, 0.0, 0.0)));
  initial.insert(r0, gtsam::Pose3());
  initial.insert(r1,
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(8.0, 0.0, 0.0)));

  const gtsam::Values result =
    gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  const double corrected_x = result.at<gtsam::Pose3>(x1).translation().x();
  EXPECT_GT(corrected_x, 8.0);
  EXPECT_LT(corrected_x, 10.0);
  EXPECT_EQ(result.size(), 4U);
}

}  // namespace
