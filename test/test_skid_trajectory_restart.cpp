#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>
#include <nabo/nabo.h>

#include "skid_trajectory_restart.hpp"

namespace {
namespace comms = liorf::comms;
namespace graph_sync = liorf::graph_sync;
namespace loop_constraint = liorf::loop_constraint;
namespace trajectory_restart = liorf::trajectory_restart;
namespace uncertainty = liorf::uncertainty;
using trajectory_restart::DescriptorColumns;
using trajectory_restart::EpochObservation;
using trajectory_restart::Superseded;
using trajectory_restart::TrajectoryEpochs;

// A three-robot team seen from alpha's map-fusion node. Every robot has its
// own epoch, so a check against the wrong one cannot pass by accident.
std::uint64_t epochOf(const std::string& robot) {
  static const std::map<std::string, std::uint64_t> epochs{
      {"alpha", 10}, {"bravo", 20}, {"charlie", 30}};
  return epochs.at(robot);
}

comms::ScanKey scanKey(const std::string& robot, std::int64_t keyframe,
                       std::uint64_t epoch) {
  comms::ScanKey key;
  key.robot_id = robot;
  key.keyframe_index = keyframe;
  key.trajectory_epoch = epoch;
  return key;
}

// Distinct for every bin, so each descriptor's nearest neighbour is itself.
Eigen::VectorXf descriptorOf(int bin) {
  Eigen::VectorXf descriptor(4);
  descriptor << static_cast<float>(bin), 0.5f * bin, -1.0f * bin, 1.0f;
  return descriptor;
}

// The fields of SOLiDBin that the purge reads.
struct Place {
  std::string robotname;
  std::uint64_t trajectory_epoch = 0;
};

struct Team {
  std::unordered_map<int, Place> bins;
  std::unordered_map<comms::ScanKey, int, comms::ScanKeyHash> bin_of_scan_key;
  DescriptorColumns columns;
  graph_sync::Replay replay;
  std::uint64_t revision = 0;
  int next_bin = 0;

  int place(const std::string& robot, std::int64_t keyframe) {
    const int bin = next_bin++;
    bins[bin] = {robot, epochOf(robot)};
    bin_of_scan_key[scanKey(robot, keyframe, epochOf(robot))] = bin;
    columns.append(bin, descriptorOf(bin));
    return bin;
  }

  graph_sync::Constraint factor(
      const std::string& from, std::int64_t index_from,
      const std::string& to, std::int64_t index_to, bool retracted = false) {
    graph_sync::Constraint message;
    loop_constraint::populateInterRobot(
        message, from, from, index_from, gtsam::Pose3(), to, index_to,
        gtsam::Pose3(),
        {gtsam::Pose3(), 1.0e-6 * uncertainty::Matrix6d::Identity()},
        0.01, 0.9, 100);
    message.authority_id = from;
    message.authority_epoch = 5;
    message.revision = ++revision;
    message.retracted = retracted;
    message.from_trajectory_epoch = epochOf(from);
    message.to_trajectory_epoch = epochOf(to);
    replay.put(message);
    return message;
  }

  std::set<int> binsOf(const std::string& robot) const {
    std::set<int> result;
    for (const auto& entry : bins)
      if (entry.second.robotname == robot) result.insert(entry.first);
    return result;
  }

  // Places alpha through charlie so each robot's columns are interleaved.
  void populate() {
    for (std::int64_t keyframe = 0; keyframe < 3; ++keyframe)
      for (const char* robot : {"alpha", "bravo", "charlie"})
        place(robot, keyframe);
  }
};

const graph_sync::Constraint& latest(
    const graph_sync::Replay& replay, const graph_sync::Constraint& message) {
  return replay.latest().at(graph_sync::identity(message));
}

// Every column still holds the descriptor of the bin it reports, and a tree
// rebuilt from the compacted matrix finds each remaining place as itself.
void expectSearchableExactly(const DescriptorColumns& columns,
                             const std::set<int>& expected) {
  std::set<int> indexed;
  for (std::size_t column = 0; column < columns.size(); ++column) {
    const int bin = columns.bin(static_cast<Eigen::Index>(column));
    indexed.insert(bin);
    EXPECT_TRUE(columns.matrix().col(static_cast<Eigen::Index>(column))
                    .isApprox(descriptorOf(bin)));
  }
  EXPECT_EQ(indexed, expected);
  ASSERT_FALSE(columns.empty());

  std::unique_ptr<Nabo::NNSearchF> tree(
      Nabo::NNSearchF::createKDTreeLinearHeap(columns.matrix()));
  Eigen::VectorXi indices(1);
  Eigen::VectorXf distances(1);
  for (const int bin : expected) {
    // libnabo skips zero-distance matches unless asked not to.
    tree->knn(descriptorOf(bin), indices, distances, 1, 0.0f,
              Nabo::NNSearchF::ALLOW_SELF_MATCH);
    EXPECT_EQ(columns.bin(indices[0]), bin);
  }
}

}  // namespace

TEST(SkidTrajectoryRestart, PeerRestartKeepsOtherPairsFactorsAndPlaces) {
  Team team;
  team.populate();
  const auto ab0 = team.factor("alpha", 0, "bravo", 0);
  const auto ab1 = team.factor("bravo", 1, "alpha", 2);
  const auto ac = team.factor("alpha", 1, "charlie", 0);
  const auto bc = team.factor("bravo", 2, "charlie", 1);
  const auto tombstone = team.factor("alpha", 2, "charlie", 2, true);
  const std::set<int> alpha = team.binsOf("alpha");
  const std::set<int> bravo = team.binsOf("bravo");
  const std::set<int> charlie = team.binsOf("charlie");
  const auto keys_before = team.bin_of_scan_key;
  const std::uint64_t last_revision = team.revision;

  TrajectoryEpochs epochs;
  for (const char* robot : {"alpha", "bravo", "charlie"})
    ASSERT_EQ(epochs.observe(robot, epochOf(robot)), EpochObservation::kCurrent);
  ASSERT_EQ(epochs.observe("charlie", 31), EpochObservation::kRestarted);
  const Superseded superseded{"charlie", 31};

  // Only factors with a charlie endpoint are withdrawn, each under a fresh
  // revision; the existing tombstone is not withdrawn a second time.
  const auto withdrawals = trajectory_restart::withdrawFactors(
      team.replay, superseded, team.revision);
  ASSERT_EQ(withdrawals.size(), 2U);
  std::set<graph_sync::Identity> withdrawn;
  for (const auto& message : withdrawals) {
    EXPECT_TRUE(message.retracted);
    EXPECT_GT(message.revision, last_revision);
    withdrawn.insert(graph_sync::identity(message));
  }
  EXPECT_EQ(withdrawn, (std::set<graph_sync::Identity>{
                           graph_sync::identity(ac), graph_sync::identity(bc)}));
  EXPECT_TRUE(latest(team.replay, ac).retracted);
  EXPECT_TRUE(latest(team.replay, bc).retracted);
  EXPECT_EQ(latest(team.replay, tombstone).revision, tombstone.revision);
  for (const auto& kept : {ab0, ab1}) {
    EXPECT_FALSE(latest(team.replay, kept).retracted);
    EXPECT_EQ(latest(team.replay, kept).revision, kept.revision);
  }

  // A receiver replaying the ledger keeps exactly the alpha-bravo factors.
  graph_sync::Ledger receiver;
  for (const auto& entry : team.replay.latest()) receiver.accept(entry.second);
  std::set<graph_sync::Identity> active;
  for (const auto& message : receiver.active())
    active.insert(graph_sync::identity(message));
  EXPECT_EQ(active, (std::set<graph_sync::Identity>{
                        graph_sync::identity(ab0), graph_sync::identity(ab1)}));

  // Only charlie's places go. Alpha's and bravo's keep their bin ids, scan
  // keys and descriptors, and stay searchable.
  const std::set<int> removed = trajectory_restart::eraseBins(
      team.bins, team.bin_of_scan_key, superseded);
  EXPECT_EQ(removed, charlie);
  EXPECT_EQ(team.binsOf("alpha"), alpha);
  EXPECT_EQ(team.binsOf("bravo"), bravo);
  EXPECT_TRUE(team.binsOf("charlie").empty());
  for (const auto& entry : keys_before) {
    const auto found = team.bin_of_scan_key.find(entry.first);
    if (entry.first.robot_id == "charlie") {
      EXPECT_EQ(found, team.bin_of_scan_key.end());
    } else {
      ASSERT_NE(found, team.bin_of_scan_key.end());
      EXPECT_EQ(found->second, entry.second);
    }
  }
  EXPECT_EQ(team.columns.erase(removed), charlie.size());
  std::set<int> remaining = alpha;
  remaining.insert(bravo.begin(), bravo.end());
  expectSearchableExactly(team.columns, remaining);

  // The pair with charlie goes; the alpha-bravo pair does not.
  EXPECT_TRUE(superseded.involves("alpha", "charlie"));
  EXPECT_FALSE(superseded.involves("alpha", "bravo"));
}

TEST(SkidTrajectoryRestart, PeerRestartForgetsOnlyItsScanTraffic) {
  comms::Config config;
  config.max_inflight_requests = 8;
  comms::RequestTracker requests(config);
  const auto old_charlie = scanKey("charlie", 4, 30);
  const auto new_charlie = scanKey("charlie", 4, 31);
  const auto bravo = scanKey("bravo", 4, 20);
  const auto abandoned_charlie = scanKey("charlie", 5, 30);
  ASSERT_EQ(requests.request(abandoned_charlie, 0.0),
            comms::RequestDecision::kSend);
  for (std::size_t attempt = 0; attempt < config.max_request_attempts; ++attempt)
    requests.expire(10.0 * (attempt + 1));
  ASSERT_TRUE(requests.isAbandoned(abandoned_charlie));
  ASSERT_EQ(requests.request(old_charlie, 50.0), comms::RequestDecision::kSend);
  ASSERT_EQ(requests.request(bravo, 50.0), comms::RequestDecision::kSend);
  ASSERT_EQ(requests.request(new_charlie, 50.0), comms::RequestDecision::kSend);

  const Superseded superseded{"charlie", 31};
  const auto covered = [&superseded](const comms::ScanKey& key) {
    return superseded.covers(key);
  };
  EXPECT_EQ(requests.eraseIf(covered), 2U);
  EXPECT_FALSE(requests.pending(old_charlie));
  EXPECT_FALSE(requests.isAbandoned(abandoned_charlie));
  EXPECT_TRUE(requests.pending(bravo));
  EXPECT_TRUE(requests.pending(new_charlie));
  EXPECT_EQ(requests.inflight(), 2U);

  comms::DeferredCandidateQueue deferred(config);
  const std::set<int> removed{7, 8};
  comms::DeferredCandidate with_charlie;
  with_charlie.query_bin = 1;
  with_charlie.candidate_bin = 7;
  with_charlie.missing = {old_charlie};
  comms::DeferredCandidate with_bravo;
  with_bravo.query_bin = 1;
  with_bravo.candidate_bin = 4;
  with_bravo.missing = {bravo};
  ASSERT_TRUE(deferred.park(with_charlie));
  ASSERT_TRUE(deferred.park(with_bravo));
  EXPECT_EQ(deferred.eraseIf([&removed](const comms::DeferredCandidate& entry) {
              return removed.count(entry.query_bin) ||
                     removed.count(entry.candidate_bin);
            }),
            1U);
  EXPECT_EQ(deferred.size(), 1U);
  EXPECT_EQ(deferred.dropped(), 0U);
  EXPECT_EQ(deferred.expired(), 0U);
  const auto ready = deferred.release(bravo);
  ASSERT_EQ(ready.size(), 1U);
  EXPECT_EQ(ready[0].candidate_bin, 4);
}

TEST(SkidTrajectoryRestart, OwnRestartRemovesOwnStaleDataAndKeepsPeers) {
  Team team;
  team.populate();
  const auto ab = team.factor("alpha", 0, "bravo", 0);
  const auto ca = team.factor("charlie", 1, "alpha", 1);
  const std::set<int> alpha = team.binsOf("alpha");
  std::set<int> peers = team.binsOf("bravo");
  const std::set<int> charlie = team.binsOf("charlie");
  peers.insert(charlie.begin(), charlie.end());

  TrajectoryEpochs epochs;
  ASSERT_EQ(epochs.observe("alpha", 10), EpochObservation::kCurrent);
  ASSERT_EQ(epochs.observe("alpha", 11), EpochObservation::kRestarted);
  const Superseded superseded{"alpha", 11};

  // Every factor this node authors has its own keyframe at one end, so all
  // of them are withdrawn.
  const auto withdrawals = trajectory_restart::withdrawFactors(
      team.replay, superseded, team.revision);
  EXPECT_EQ(withdrawals.size(), 2U);
  EXPECT_TRUE(latest(team.replay, ab).retracted);
  EXPECT_TRUE(latest(team.replay, ca).retracted);
  EXPECT_TRUE(trajectory_restart::withdrawFactors(
      team.replay, superseded, team.revision).empty());

  // Its own places go, because the new trajectory reuses their keyframe
  // indices. Peers' places stay matchable for the new keyframes.
  const std::set<int> removed = trajectory_restart::eraseBins(
      team.bins, team.bin_of_scan_key, superseded);
  EXPECT_EQ(removed, alpha);
  for (const auto& entry : team.bin_of_scan_key)
    EXPECT_NE(entry.first.robot_id, "alpha");
  team.columns.erase(removed);
  expectSearchableExactly(team.columns, peers);

  // Every pair fused here has this node at one end.
  EXPECT_TRUE(superseded.involves("alpha", "bravo"));
  EXPECT_TRUE(superseded.involves("charlie", "alpha"));
  EXPECT_FALSE(superseded.involves("bravo", "charlie"));

  // The new trajectory's places are not stale, even at a reused keyframe
  // index, and a new place takes a fresh bin id.
  EXPECT_TRUE(superseded.covers(scanKey("alpha", 0, 10)));
  EXPECT_FALSE(superseded.covers(scanKey("alpha", 0, 11)));
  EXPECT_FALSE(superseded.covers(scanKey("bravo", 0, 20)));
  team.bins[team.next_bin] = {"alpha", 11};
  team.columns.append(team.next_bin, descriptorOf(team.next_bin));
  ++team.next_bin;
  EXPECT_TRUE(trajectory_restart::eraseBins(
      team.bins, team.bin_of_scan_key, superseded).empty());
  peers.insert(team.next_bin - 1);
  expectSearchableExactly(team.columns, peers);
}

TEST(SkidTrajectoryRestart, StaleEpochMessagesAreStillRejected) {
  TrajectoryEpochs epochs;
  EXPECT_FALSE(epochs.known("charlie"));
  EXPECT_EQ(epochs.observe("charlie", 30), EpochObservation::kCurrent);
  EXPECT_TRUE(epochs.known("charlie"));
  EXPECT_EQ(epochs.observe("charlie", 30), EpochObservation::kCurrent);
  EXPECT_EQ(epochs.observe("charlie", 31), EpochObservation::kRestarted);
  // The superseded trajectory, and the legacy unknown epoch, stay rejected
  // and do not move the known epoch back.
  EXPECT_EQ(epochs.observe("charlie", 30), EpochObservation::kStale);
  EXPECT_EQ(epochs.observe("charlie", 0), EpochObservation::kStale);
  EXPECT_EQ(epochs.observe("charlie", 31), EpochObservation::kCurrent);
  EXPECT_TRUE(epochs.superseded("charlie", 30));
  EXPECT_FALSE(epochs.superseded("charlie", 31));
  EXPECT_FALSE(epochs.superseded("bravo", 1));

  // Legacy epoch zero is accepted until a real epoch is known, and the first
  // real epoch is not a restart because nothing belonged to an earlier one.
  EXPECT_EQ(epochs.observe("delta", 0), EpochObservation::kCurrent);
  EXPECT_TRUE(epochs.known("delta"));
  EXPECT_FALSE(epochs.superseded("delta", 0));
  EXPECT_EQ(epochs.observe("delta", 5), EpochObservation::kCurrent);
  EXPECT_EQ(epochs.observe("delta", 0), EpochObservation::kStale);
}

TEST(SkidTrajectoryRestart, DescriptorColumnsCompactWithoutRenumbering) {
  DescriptorColumns columns;
  EXPECT_TRUE(columns.empty());
  EXPECT_EQ(columns.bin(0), -1);
  for (const int bin : {3, 5, 8, 13}) columns.append(bin, descriptorOf(bin));
  EXPECT_EQ(columns.matrix().rows(), 4);
  EXPECT_EQ(columns.bin(-1), -1);
  EXPECT_EQ(columns.bin(4), -1);

  // Nothing to drop leaves the matrix, and any tree over it, untouched.
  const float* data = columns.matrix().data();
  EXPECT_EQ(columns.erase({1, 2}), 0U);
  EXPECT_EQ(columns.matrix().data(), data);

  EXPECT_EQ(columns.erase({5, 13}), 2U);
  ASSERT_EQ(columns.size(), 2U);
  EXPECT_EQ(columns.bin(0), 3);
  EXPECT_EQ(columns.bin(1), 8);
  expectSearchableExactly(columns, {3, 8});

  EXPECT_EQ(columns.erase({3, 8}), 2U);
  EXPECT_TRUE(columns.empty());
  EXPECT_EQ(columns.matrix().cols(), 0);
}
