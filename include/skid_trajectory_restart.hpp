#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "skid_comms.hpp"
#include "skid_graph_sync.hpp"

// What a trajectory restart invalidates in distributed map fusion.
//
// A robot whose mapping process restarts starts a new trajectory epoch. Its
// keyframe indices start again from zero in a new map frame, so whatever map
// fusion holds about the earlier trajectory is unusable: its descriptors
// would match places that no longer exist, its factors would bind the new
// keyframes that reuse those indices, and the registrations and map
// alignments it took part in are expressed in a frame that is gone.
//
// None of that applies to the other robots. Their places and scans, and the
// pairs, alignments and factors between them, stay valid and must be kept:
// descriptors are announced once, so a place dropped here is never offered
// again.
namespace liorf::trajectory_restart {

enum class EpochObservation {
  kStale,      // older than the robot's known trajectory; ignore the message
  kCurrent,    // the known trajectory, or the first one seen for the robot
  kRestarted,  // a newer trajectory replaced a known one
};

// The latest trajectory epoch seen for each robot.
//
// Epoch zero is the legacy "unknown" value. It is accepted until a nonzero
// epoch is known and stale afterwards. The first nonzero epoch is not a
// restart, because nothing was attributed to an earlier one.
class TrajectoryEpochs {
 public:
  EpochObservation observe(const std::string& robot, std::uint64_t epoch) {
    auto& known = epochs_[robot];
    if (known != 0 && epoch < known)
      return EpochObservation::kStale;
    if (epoch == known)
      return EpochObservation::kCurrent;
    const bool restarted = known != 0;
    known = epoch;
    return restarted ? EpochObservation::kRestarted : EpochObservation::kCurrent;
  }

  // True once any epoch, including the legacy zero, was observed for `robot`.
  bool known(const std::string& robot) const {
    return epochs_.count(robot) != 0;
  }

  // True when `epoch` belongs to a trajectory `robot` has since replaced.
  bool superseded(const std::string& robot, std::uint64_t epoch) const {
    const auto known = epochs_.find(robot);
    return known != epochs_.end() && known->second != 0 && epoch < known->second;
  }

 private:
  std::map<std::string, std::uint64_t> epochs_;
};

// Every trajectory of `robot` older than `epoch`.
struct Superseded {
  std::string robot;
  std::uint64_t epoch = 0;

  bool covers(const std::string& owner, std::uint64_t owner_epoch) const {
    return owner == robot && owner_epoch < epoch;
  }

  bool covers(const comms::ScanKey& key) const {
    return covers(key.robot_id, key.trajectory_epoch);
  }

  // A factor is invalid as soon as either endpoint is.
  bool covers(const graph_sync::Constraint& factor) const {
    return covers(factor.from_robot_id, factor.from_trajectory_epoch) ||
           covers(factor.to_robot_id, factor.to_trajectory_epoch);
  }

  // State that relates two robots without naming a trajectory, such as the
  // pair a node fuses or the alignment between two maps, is invalid when
  // either robot is the one that restarted.
  bool involves(const std::string& first, const std::string& second) const {
    return first == robot || second == robot;
  }
};

// Retracts every live factor with an endpoint on the superseded trajectory,
// under fresh revisions, and records the withdrawals in the replay so they
// keep being resent. Factors that do not touch it, and existing tombstones,
// are left as they are. Returns the withdrawals for immediate publication.
inline std::vector<graph_sync::Constraint> withdrawFactors(
    graph_sync::Replay& replay, const Superseded& superseded,
    std::uint64_t& revision) {
  std::vector<graph_sync::Constraint> withdrawals;
  for (const auto& entry : replay.latest()) {
    if (entry.second.retracted || !superseded.covers(entry.second))
      continue;
    auto withdrawal = entry.second;
    withdrawal.retracted = true;
    withdrawal.revision = ++revision;
    withdrawals.push_back(std::move(withdrawal));
  }
  for (const auto& withdrawal : withdrawals)
    replay.put(withdrawal);
  return withdrawals;
}

// Removes the places on the superseded trajectory from a bin table and from
// its scan-key lookup, and returns their bin ids. The remaining places keep
// their ids, so loop and parked candidates that refer to them stay valid.
//
// `Bin` needs `robotname` and `trajectory_epoch` members, as SOLiDBin has.
template <typename Bin>
std::set<int> eraseBins(
    std::unordered_map<int, Bin>& bins,
    std::unordered_map<comms::ScanKey, int, comms::ScanKeyHash>& bin_of_scan_key,
    const Superseded& superseded) {
  std::set<int> removed;
  for (auto it = bins.begin(); it != bins.end();) {
    if (superseded.covers(it->second.robotname, it->second.trajectory_epoch)) {
      removed.insert(it->first);
      it = bins.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = bin_of_scan_key.begin(); it != bin_of_scan_key.end();) {
    if (removed.count(it->second) != 0 || superseded.covers(it->first))
      it = bin_of_scan_key.erase(it);
    else
      ++it;
  }
  return removed;
}

// Descriptor matrix whose columns are addressed by bin id.
//
// A libnabo tree is built over a fixed matrix and cannot delete points. To
// forget some places without forgetting the rest, erase() compacts the matrix
// and the caller rebuilds its tree from matrix(); a tree must not be used
// across append() or erase(). Bin ids are stable; column positions are not,
// so search results go through bin().
class DescriptorColumns {
 public:
  std::size_t size() const noexcept { return bins_.size(); }
  bool empty() const noexcept { return bins_.empty(); }
  const Eigen::MatrixXf& matrix() const noexcept { return matrix_; }

  // Bin id held in `column`, or -1 for a column that does not exist, such as
  // the placeholder in a short search result.
  int bin(Eigen::Index column) const noexcept {
    return column >= 0 && static_cast<std::size_t>(column) < bins_.size() ?
        bins_[static_cast<std::size_t>(column)] : -1;
  }

  // Every descriptor must have the length of the first.
  void append(int bin, const Eigen::VectorXf& descriptor) {
    if (bins_.empty())
      matrix_.resize(descriptor.size(), 0);
    const Eigen::Index column = matrix_.cols();
    matrix_.conservativeResize(Eigen::NoChange, column + 1);
    matrix_.col(column) = descriptor;
    bins_.push_back(bin);
  }

  // Drops the columns of `removed` bins and keeps the rest in order. Returns
  // how many columns were dropped; the matrix is untouched when none were.
  std::size_t erase(const std::set<int>& removed) {
    std::vector<std::size_t> kept;
    kept.reserve(bins_.size());
    for (std::size_t column = 0; column < bins_.size(); ++column) {
      if (removed.count(bins_[column]) == 0)
        kept.push_back(column);
    }
    const std::size_t dropped = bins_.size() - kept.size();
    if (dropped == 0)
      return 0;
    Eigen::MatrixXf matrix(matrix_.rows(), static_cast<Eigen::Index>(kept.size()));
    std::vector<int> bins;
    bins.reserve(kept.size());
    for (const std::size_t column : kept) {
      matrix.col(static_cast<Eigen::Index>(bins.size())) =
          matrix_.col(static_cast<Eigen::Index>(column));
      bins.push_back(bins_[column]);
    }
    matrix_ = std::move(matrix);
    bins_ = std::move(bins);
    return dropped;
  }

 private:
  Eigen::MatrixXf matrix_;
  std::vector<int> bins_;
};

}  // namespace liorf::trajectory_restart
