#pragma once

#include <chrono>
#include <map>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

#include "loop_constraint_utils.hpp"
#include "skid_remote_graph.hpp"

namespace liorf::graph_sync {

// Process generations must remain independent of /clock, including bag seeks.
// Each robot must keep its system clock monotonic across process restarts.
inline std::uint64_t newEpoch() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count());
}

using Constraint = liorf::msg::LoopConstraint;
using Identity = remote_graph::FactorIdentity;
using PoseIdentity = std::tuple<std::string, std::uint64_t, std::int64_t>;

inline Identity identity(const Constraint& message) {
  return remote_graph::canonicalFactorIdentity(
      message.from_robot_id, message.index_from,
      message.to_robot_id, message.index_to);
}

// Validate before consuming a revision: a malformed newer packet must not
// prevent a valid retransmission from repairing the receiver's graph.
inline bool valid(const Constraint& message) {
  if (message.from_robot_id.empty() || message.to_robot_id.empty() ||
      message.index_from < 0 || message.index_to < 0 ||
      message.index_from > std::numeric_limits<std::uint32_t>::max() ||
      message.index_to > std::numeric_limits<std::uint32_t>::max() ||
      (message.robot_id != message.from_robot_id &&
       message.robot_id != message.to_robot_id) ||
      (message.from_robot_id == message.to_robot_id &&
       message.index_from == message.index_to))
    return false;
  if (message.revision != 0 &&
      (message.authority_epoch == 0 ||
       (message.authority_id != message.from_robot_id &&
        message.authority_id != message.to_robot_id) ||
       message.from_trajectory_epoch == 0 || message.to_trajectory_epoch == 0))
    return false;
  if (message.retracted)
    return message.revision != 0;
  return loop_constraint::validPoseMessage(message.relative_pose.pose) &&
      uncertainty::positiveDefiniteCovariance(
          loop_constraint::covarianceFromMessage(message.relative_pose.covariance)) &&
      (message.from_robot_id == message.to_robot_id ||
       (loop_constraint::validPoseMessage(message.from_pose) &&
        loop_constraint::validPoseMessage(message.to_pose)));
}

// One latest claim (including withdrawals) per authority and physical pair.
// A receiver never counts reverse reports twice, nor lets one observer retract
// another observer's independent claim. Selection is identical on both robots.
class Ledger {
 public:
  bool observeTrajectory(const std::string& robot, std::uint64_t epoch) {
    if (epoch == 0 || epoch <= trajectory_epochs_[robot])
      return false;
    trajectory_epochs_[robot] = epoch;
    return true;
  }

  bool accept(const Constraint& message) {
    if (!valid(message) || !currentTrajectory(message))
      return false;
    const std::string authority = message.revision ? message.authority_id : "";
    if (message.revision) {
      auto& epoch = authority_epochs_[authority];
      if (message.authority_epoch < epoch)
        return false;
      if (message.authority_epoch > epoch) {
        for (auto& entry : claims_)
          entry.second.erase(authority);
        epoch = message.authority_epoch;
      }
    }
    auto& claims = claims_[identity(message)];
    const auto old = claims.find(authority);
    if (old != claims.end() && message.revision <= old->second.revision)
      return false;
    observeTrajectory(message.from_robot_id, message.from_trajectory_epoch);
    observeTrajectory(message.to_robot_id, message.to_trajectory_epoch);
    claims[authority] = message;
    return true;
  }

  std::vector<Constraint> active() const {
    std::vector<Constraint> result;
    for (const auto& entry : claims_) {
      const auto& claims = entry.second;
      const bool modern = claims.size() > claims.count("");
      for (const auto& claim : claims) {
        const auto& message = claim.second;
        if ((modern && message.revision == 0) || message.retracted ||
            !currentTrajectory(message))
          continue;
        result.push_back(message);
        break;
      }
    }
    return result;
  }

 private:
  bool currentTrajectory(const Constraint& message) const {
    const auto current = [this](const std::string& robot, std::uint64_t epoch) {
      const auto known = trajectory_epochs_.find(robot);
      return epoch == 0 || known == trajectory_epochs_.end() || epoch >= known->second;
    };
    return current(message.from_robot_id, message.from_trajectory_epoch) &&
        current(message.to_robot_id, message.to_trajectory_epoch);
  }

  std::map<Identity, std::map<std::string, Constraint>> claims_;
  std::map<std::string, std::uint64_t> authority_epochs_;
  std::map<std::string, std::uint64_t> trajectory_epochs_;
};

struct CorrectedPose {
  std::uint64_t revision = 0;
  gtsam::Pose3 pose;
};
using PoseStore = std::map<PoseIdentity, CorrectedPose>;

// Cyclic replay retains only the latest version of a pair, including its
// tombstone. It uses constant work per timer tick even during a long outage;
// recovery also works when a receiver restarts and loses its deduplication set.
class Replay {
 public:
  void put(const Constraint& message) { latest_[identity(message)] = message; }

  std::vector<Constraint> next(std::size_t limit) {
    std::vector<Constraint> result;
    if (latest_.empty())
      return result;
    auto it = have_cursor_ ? latest_.upper_bound(cursor_) : latest_.begin();
    for (std::size_t count = 0; count < limit && count < latest_.size(); ++count) {
      if (it == latest_.end())
        it = latest_.begin();
      result.push_back(it->second);
      cursor_ = it->first;
      have_cursor_ = true;
      ++it;
    }
    return result;
  }

  const std::map<Identity, Constraint>& latest() const { return latest_; }

 private:
  std::map<Identity, Constraint> latest_;
  Identity cursor_;
  bool have_cursor_ = false;
};

}  // namespace liorf::graph_sync
