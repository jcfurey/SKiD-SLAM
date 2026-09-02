#include "loop_constraint_utils.hpp"

#include <limits>

#include <gtest/gtest.h>

namespace {

TEST(LoopConstraintMessage, ReordersRosAndGtsamCovarianceExactly) {
  liorf::uncertainty::Matrix6d covariance =
    liorf::uncertainty::Matrix6d::Zero();
  covariance.diagonal() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  covariance(0, 4) = 0.25;
  covariance(4, 0) = 0.25;

  std::array<double, 36> ros_covariance{};
  liorf::loop_constraint::covarianceToMessage(
    covariance, ros_covariance);

  EXPECT_DOUBLE_EQ(ros_covariance[0], 4.0);
  EXPECT_DOUBLE_EQ(ros_covariance[7], 5.0);
  EXPECT_DOUBLE_EQ(ros_covariance[14], 6.0);
  EXPECT_DOUBLE_EQ(ros_covariance[21], 1.0);
  EXPECT_DOUBLE_EQ(ros_covariance[28], 2.0);
  EXPECT_DOUBLE_EQ(ros_covariance[35], 3.0);
  EXPECT_DOUBLE_EQ(ros_covariance[3 * 6 + 1], 0.25);
  EXPECT_TRUE(liorf::loop_constraint::covarianceFromMessage(ros_covariance)
    .isApprox(covariance, 0.0));
}

TEST(LoopConstraintMessage, RoundTripsPoseAndDiagnostics) {
  const gtsam::Pose3 pose(
    gtsam::Rot3::RzRyRx(0.1, -0.2, 0.3),
    gtsam::Point3(4.0, -5.0, 6.0));
  const liorf::uncertainty::PoseWithCovariance measurement{
    pose, 0.5 * liorf::uncertainty::Matrix6d::Identity()};

  liorf::msg::LoopConstraint message;
  liorf::loop_constraint::populate(
    message, "platform7", 41, 93, measurement, 0.012, 0.75, 812);

  EXPECT_EQ(message.robot_id, "platform7");
  EXPECT_EQ(message.from_robot_id, "platform7");
  EXPECT_EQ(message.index_from, 41);
  EXPECT_EQ(message.to_robot_id, "platform7");
  EXPECT_EQ(message.index_to, 93);
  EXPECT_DOUBLE_EQ(message.registration_error_m2, 0.012);
  EXPECT_DOUBLE_EQ(message.overlap_ratio, 0.75);
  EXPECT_EQ(message.registration_inliers, 812U);
  EXPECT_TRUE(liorf::loop_constraint::poseFromMessage(
    message.relative_pose.pose).equals(pose, 1.0e-12));
  EXPECT_TRUE(liorf::loop_constraint::covarianceFromMessage(
    message.relative_pose.covariance).isApprox(
      measurement.covariance, 1.0e-12));
}

TEST(LoopConstraintMessage, CarriesBothInterRobotEndpointsAndOwnerPoses) {
  const gtsam::Pose3 from_pose(
    gtsam::Rot3::RzRyRx(0.0, 0.0, 0.1),
    gtsam::Point3(1.0, 2.0, 3.0));
  const gtsam::Pose3 to_pose(
    gtsam::Rot3::RzRyRx(0.0, 0.0, -0.2),
    gtsam::Point3(8.0, 9.0, 10.0));
  const liorf::uncertainty::PoseWithCovariance measurement{
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.5, 0.0, 0.0)),
    0.25 * liorf::uncertainty::Matrix6d::Identity()};

  liorf::msg::LoopConstraint message;
  liorf::loop_constraint::populateInterRobot(
    message, "alpha", "alpha", 12, from_pose,
    "beta", 34, to_pose, measurement, 0.02, 0.8, 500);

  EXPECT_EQ(message.robot_id, "alpha");
  EXPECT_EQ(message.from_robot_id, "alpha");
  EXPECT_EQ(message.index_from, 12);
  EXPECT_EQ(message.to_robot_id, "beta");
  EXPECT_EQ(message.index_to, 34);
  EXPECT_TRUE(liorf::loop_constraint::poseFromMessage(
    message.from_pose).equals(from_pose, 1.0e-12));
  EXPECT_TRUE(liorf::loop_constraint::poseFromMessage(
    message.to_pose).equals(to_pose, 1.0e-12));
  EXPECT_TRUE(liorf::loop_constraint::poseFromMessage(
    message.relative_pose.pose).equals(measurement.pose, 1.0e-12));
}

TEST(LoopConstraintMessage, RejectsNonFiniteOrZeroQuaternionPose) {
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 0.0;
  EXPECT_FALSE(liorf::loop_constraint::validPoseMessage(pose));

  pose.orientation.w = 1.0;
  EXPECT_TRUE(liorf::loop_constraint::validPoseMessage(pose));

  pose.position.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(liorf::loop_constraint::validPoseMessage(pose));
}

}  // namespace
