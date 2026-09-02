#include "skid_graph_keys.hpp"

#include <gtest/gtest.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

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

}  // namespace
