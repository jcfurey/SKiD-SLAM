#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <stdexcept>

#include <gtsam/geometry/Pose3.h>

#include "liorf/msg/loop_constraint.hpp"
#include "skid_pose_uncertainty.hpp"

namespace liorf::loop_constraint {

inline bool validPoseMessage(const geometry_msgs::msg::Pose& message) noexcept {
  const double quaternion_norm_squared =
    message.orientation.x * message.orientation.x +
    message.orientation.y * message.orientation.y +
    message.orientation.z * message.orientation.z +
    message.orientation.w * message.orientation.w;
  return std::isfinite(message.position.x) &&
         std::isfinite(message.position.y) &&
         std::isfinite(message.position.z) &&
         std::isfinite(quaternion_norm_squared) &&
         quaternion_norm_squared > 1.0e-12;
}

inline geometry_msgs::msg::Pose poseToMessage(const gtsam::Pose3& pose) {
  geometry_msgs::msg::Pose message;
  message.position.x = pose.translation().x();
  message.position.y = pose.translation().y();
  message.position.z = pose.translation().z();
  const Eigen::Quaterniond quaternion = pose.rotation().toQuaternion();
  message.orientation.x = quaternion.x();
  message.orientation.y = quaternion.y();
  message.orientation.z = quaternion.z();
  message.orientation.w = quaternion.w();
  return message;
}

inline gtsam::Pose3 poseFromMessage(const geometry_msgs::msg::Pose& message) {
  if (!validPoseMessage(message))
    throw std::invalid_argument("invalid pose message");
  const Eigen::Quaterniond quaternion = Eigen::Quaterniond(
      message.orientation.w, message.orientation.x,
      message.orientation.y, message.orientation.z).normalized();
  return gtsam::Pose3(
    gtsam::Rot3(quaternion),
    gtsam::Point3(
      message.position.x,
      message.position.y,
      message.position.z));
}

inline void covarianceToMessage(
  const uncertainty::Matrix6d& gtsam_covariance,
  std::array<double, 36>& ros_covariance) {
  // ROS: [tx, ty, tz, rx, ry, rz]. GTSAM: [rx, ry, rz, tx, ty, tz].
  constexpr std::array<std::size_t, 6> gtsam_index_for_ros{3, 4, 5, 0, 1, 2};
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      ros_covariance[row * 6 + column] = gtsam_covariance(
        gtsam_index_for_ros[row], gtsam_index_for_ros[column]);
    }
  }
}

inline uncertainty::Matrix6d covarianceFromMessage(
  const std::array<double, 36>& ros_covariance) {
  constexpr std::array<std::size_t, 6> ros_index_for_gtsam{3, 4, 5, 0, 1, 2};
  uncertainty::Matrix6d gtsam_covariance;
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      gtsam_covariance(row, column) = ros_covariance[
        ros_index_for_gtsam[row] * 6 + ros_index_for_gtsam[column]];
    }
  }
  return gtsam_covariance;
}

inline void populate(
  liorf::msg::LoopConstraint& message,
  const std::string& robot_id,
  std::int64_t index_from,
  std::int64_t index_to,
  const uncertainty::PoseWithCovariance& measurement,
  double registration_error_m2,
  double overlap_ratio,
  std::uint64_t registration_inliers) {
  message.robot_id = robot_id;
  message.from_robot_id = robot_id;
  message.index_from = index_from;
  message.to_robot_id = robot_id;
  message.index_to = index_to;
  message.relative_pose.pose = poseToMessage(measurement.pose);
  covarianceToMessage(
    measurement.covariance, message.relative_pose.covariance);
  message.registration_error_m2 = registration_error_m2;
  message.overlap_ratio = overlap_ratio;
  message.registration_inliers = registration_inliers;
}

inline void populateInterRobot(
  liorf::msg::LoopConstraint& message,
  const std::string& recipient_robot_id,
  const std::string& from_robot_id,
  std::int64_t index_from,
  const gtsam::Pose3& from_owner_frame_pose,
  const std::string& to_robot_id,
  std::int64_t index_to,
  const gtsam::Pose3& to_owner_frame_pose,
  const uncertainty::PoseWithCovariance& measurement,
  double registration_error_m2,
  double overlap_ratio,
  std::uint64_t registration_inliers) {
  message.robot_id = recipient_robot_id;
  message.from_robot_id = from_robot_id;
  message.index_from = index_from;
  message.from_pose = poseToMessage(from_owner_frame_pose);
  message.to_robot_id = to_robot_id;
  message.index_to = index_to;
  message.to_pose = poseToMessage(to_owner_frame_pose);
  message.relative_pose.pose = poseToMessage(measurement.pose);
  covarianceToMessage(
    measurement.covariance, message.relative_pose.covariance);
  message.registration_error_m2 = registration_error_m2;
  message.overlap_ratio = overlap_ratio;
  message.registration_inliers = registration_inliers;
}

}  // namespace liorf::loop_constraint
