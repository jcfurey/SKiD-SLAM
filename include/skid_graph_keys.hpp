#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>

namespace liorf::graph_keys {

// Keep locally owned trajectory poses in their own explicit symbol space.
// Remote poses will coexist in the same Values object during Equation (6)
// optimization, so a bare keyframe index is no longer a safe graph key.
inline gtsam::Key localPose(std::size_t keyframe_index) {
  return gtsam::Symbol('x', static_cast<std::uint64_t>(keyframe_index));
}

inline bool isLocalPose(gtsam::Key key) noexcept {
  return gtsam::Symbol(key).chr() == 'x';
}

inline bool isRemotePose(gtsam::Key key) noexcept {
  return gtsam::Symbol(key).chr() == 'r';
}

// Assigns compact, process-local namespaces to peer robot IDs. The namespace
// occupies the upper 24 bits of a Symbol's 56-bit index and the keyframe index
// occupies the lower 32 bits. The actual numeric key is deliberately local to
// one optimizer; robot IDs remain explicit on the ROS contract.
class KeySpace {
 public:
  explicit KeySpace(std::string local_robot_id)
      : local_robot_id_(std::move(local_robot_id)) {
    if (local_robot_id_.empty())
      throw std::invalid_argument("local robot ID must not be empty");
  }

  gtsam::Key pose(const std::string& robot_id, std::uint64_t keyframe_index) {
    if (robot_id.empty())
      throw std::invalid_argument("pose robot ID must not be empty");
    if (robot_id == local_robot_id_)
      return localPose(static_cast<std::size_t>(keyframe_index));
    if (keyframe_index > std::numeric_limits<std::uint32_t>::max())
      throw std::out_of_range("remote keyframe index exceeds 32 bits");

    auto namespace_it = remote_namespaces_.find(robot_id);
    if (namespace_it == remote_namespaces_.end()) {
      if (next_remote_namespace_ > kMaxRemoteNamespace)
        throw std::overflow_error("remote robot namespace exhausted");
      namespace_it = remote_namespaces_.emplace(
        robot_id, next_remote_namespace_++).first;
    }
    const std::uint64_t packed_index =
      (static_cast<std::uint64_t>(namespace_it->second) << 32U) |
      keyframe_index;
    return gtsam::Symbol('r', packed_index);
  }

  const std::string& localRobotId() const noexcept { return local_robot_id_; }
  std::size_t remoteRobotCount() const noexcept {
    return remote_namespaces_.size();
  }

 private:
  static constexpr std::uint32_t kMaxRemoteNamespace = 0x00ffffffU;

  std::string local_robot_id_;
  std::unordered_map<std::string, std::uint32_t> remote_namespaces_;
  std::uint32_t next_remote_namespace_ = 1;
};

}  // namespace liorf::graph_keys
