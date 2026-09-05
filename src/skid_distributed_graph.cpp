#include "skid_distributed_graph.hpp"

#include <algorithm>
#include <gtsam/slam/BetweenFactor.h>
#include "skid_graph_keys.hpp"

namespace liorf::graph_sync {

std::unique_ptr<gtsam::ISAM2> rebuild(
    const gtsam::ISAM2& current,
    const std::vector<Constraint>& constraints,
    const PoseStore& corrections,
    const std::string& local_robot,
    const gtsam::SharedNoiseModel& remote_motion_noise) {
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  const auto estimate = current.calculateEstimate();
  for (const auto& factor : current.getFactorsUnsafe()) {
    if (factor && std::all_of(factor->keys().begin(), factor->keys().end(),
                             graph_keys::isLocalPose))
      graph.push_back(factor);
  }
  for (const auto& value : estimate) {
    if (graph_keys::isLocalPose(value.key))
      initial.insert(value.key, value.value);
  }

  graph_keys::KeySpace keys(local_robot);
  std::map<std::pair<std::string, std::uint64_t>,
           std::map<std::int64_t, gtsam::Pose3>> peers;
  for (const auto& message : constraints) {
    if (message.from_robot_id == message.to_robot_id)
      continue;  // Legacy local-only factors are preserved above.
    const bool from_local = message.from_robot_id == local_robot;
    const auto local_index = from_local ? message.index_from : message.index_to;
    const auto local_key = graph_keys::localPose(local_index);
    if (!initial.exists(local_key))
      continue;  // Replay can arrive before its local keyframe.
    const auto& peer = from_local ? message.to_robot_id : message.from_robot_id;
    const auto index = from_local ? message.index_to : message.index_from;
    const auto epoch = from_local ? message.to_trajectory_epoch :
                                    message.from_trajectory_epoch;
    const auto remote_key = keys.pose(peer, index);
    const auto measurement = loop_constraint::poseFromMessage(message.relative_pose.pose);
    if (!initial.exists(remote_key)) {
      initial.insert(remote_key, remote_graph::initialRemotePose(
          initial.at<gtsam::Pose3>(local_key), measurement, from_local));
    }
    const auto corrected = corrections.find({peer, epoch, index});
    const auto owner_pose = corrected == corrections.end() ?
        loop_constraint::poseFromMessage(from_local ? message.to_pose : message.from_pose) :
        corrected->second.pose;
    peers[{peer, epoch}][index] = owner_pose;
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        from_local ? local_key : remote_key,
        from_local ? remote_key : local_key, measurement,
        gtsam::noiseModel::Gaussian::Covariance(
            loop_constraint::covarianceFromMessage(message.relative_pose.covariance)));
  }
  for (const auto& peer : peers) {
    const auto& trajectory = peer.second;
    if (trajectory.empty())
      continue;
    auto previous = trajectory.begin();
    for (auto next = std::next(previous); next != trajectory.end(); ++next) {
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          keys.pose(peer.first.first, previous->first),
          keys.pose(peer.first.first, next->first),
          previous->second.between(next->second), remote_motion_noise);
      previous = next;
    }
  }
  auto replacement = std::make_unique<gtsam::ISAM2>(current.params());
  if (!graph.empty()) {
    replacement->update(graph, initial);
    for (int iteration = 0; iteration < 5; ++iteration)
      replacement->update();
    const auto result = replacement->calculateEstimate();
    if (!std::isfinite(graph.error(result)))
      throw std::runtime_error("distributed graph optimization is not finite");
  }
  return replacement;
}

}  // namespace liorf::graph_sync
