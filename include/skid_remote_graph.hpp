#pragma once

#include <cstdint>
#include <iterator>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>

#include <gtsam/geometry/Pose3.h>

namespace liorf::remote_graph {

using FactorIdentity = std::tuple<
  std::string, std::int64_t, std::string, std::int64_t>;

// Two observers may report the same physical registration with opposite
// endpoint orientation. Identify it independently of that orientation so an
// optimizer never counts the measurement twice.
inline FactorIdentity canonicalFactorIdentity(
  const std::string& from_robot_id,
  std::int64_t index_from,
  const std::string& to_robot_id,
  std::int64_t index_to) {
  std::pair<std::string, std::int64_t> first{from_robot_id, index_from};
  std::pair<std::string, std::int64_t> second{to_robot_id, index_to};
  if (second < first)
    std::swap(first, second);
  return std::make_tuple(
    first.first, first.second, second.first, second.second);
}

// A partial peer-trajectory measurement used by the distributed Equation (6)
// graph. Poses supplied by the peer live in its own map frame, but their
// relative transform is frame invariant and can therefore constrain remote
// variables in this robot's optimizer.
struct OdometryEdge {
  std::string robot_id;
  std::uint64_t index_from = 0;
  std::uint64_t index_to = 0;
  gtsam::Pose3 measurement;
};

// Seed a newly observed peer state in this optimizer's map frame. A
// BetweenFactor(from, to, measurement) satisfies to = from * measurement.
// The seed is only an initial value; the registration is represented exactly
// once by the factor itself and is not duplicated as a prior.
inline gtsam::Pose3 initialRemotePose(
  const gtsam::Pose3& local_pose,
  const gtsam::Pose3& from_to_measurement,
  bool local_is_from) {
  return local_is_from ?
    local_pose.compose(from_to_measurement) :
    local_pose.compose(from_to_measurement.inverse());
}

class TrajectoryStore {
 public:
  // Stores a peer keyframe once. Every keyframe after the first is joined to
  // one already-stored keyframe, yielding a bounded sparse spanning tree even
  // when accepted loop factors arrive out of temporal order.
  std::optional<OdometryEdge> insert(
    const std::string& robot_id,
    std::uint64_t keyframe_index,
    const gtsam::Pose3& owner_frame_pose) {
    auto& trajectory = trajectories_[robot_id];
    const auto [inserted_it, inserted] = trajectory.emplace(
      keyframe_index, owner_frame_pose);
    if (!inserted)
      return std::nullopt;

    if (trajectory.size() == 1)
      return std::nullopt;

    if (inserted_it != trajectory.begin()) {
      const auto predecessor = std::prev(inserted_it);
      return OdometryEdge{
        robot_id,
        predecessor->first,
        inserted_it->first,
        predecessor->second.between(inserted_it->second)};
    }

    const auto successor = std::next(inserted_it);
    return OdometryEdge{
      robot_id,
      inserted_it->first,
      successor->first,
      inserted_it->second.between(successor->second)};
  }

  bool contains(
    const std::string& robot_id, std::uint64_t keyframe_index) const {
    const auto trajectory = trajectories_.find(robot_id);
    return trajectory != trajectories_.end() &&
      trajectory->second.find(keyframe_index) != trajectory->second.end();
  }

  std::size_t size(const std::string& robot_id) const {
    const auto trajectory = trajectories_.find(robot_id);
    return trajectory == trajectories_.end() ? 0U : trajectory->second.size();
  }

 private:
  std::unordered_map<std::string, std::map<std::uint64_t, gtsam::Pose3>>
    trajectories_;
};

}  // namespace liorf::remote_graph
