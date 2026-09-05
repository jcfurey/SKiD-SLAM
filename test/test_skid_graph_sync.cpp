#include "skid_distributed_graph.hpp"
#include "skid_graph_keys.hpp"
#include "skid_pcm_commitment.hpp"

#include <gtest/gtest.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace {
using namespace liorf;

gtsam::Pose3 translation(double x) {
  return {gtsam::Rot3(), gtsam::Point3(x, 0, 0)};
}

graph_sync::Constraint constraint(std::int64_t index = 0, std::uint64_t revision = 1) {
  graph_sync::Constraint message;
  loop_constraint::populateInterRobot(message, "alpha", "alpha", index,
      translation(index * 10), "beta", index, translation(index * 8),
      {gtsam::Pose3(), 1.0e-6 * uncertainty::Matrix6d::Identity()}, 0.01, 0.9, 100);
  message.authority_id = "beta";
  message.authority_epoch = 30;
  message.revision = revision;
  message.from_trajectory_epoch = 10;
  message.to_trajectory_epoch = 20;
  return message;
}

TEST(GraphSync, DuplicateAndReorderedPacketsCannotResurrectWithdrawnFactors) {
  graph_sync::Ledger ledger;
  auto message = constraint();
  ASSERT_TRUE(ledger.accept(message));
  EXPECT_FALSE(ledger.accept(message));
  message.retracted = true;
  message.revision = 3;
  ASSERT_TRUE(ledger.accept(message));
  EXPECT_TRUE(ledger.active().empty());
  message.retracted = false;
  message.revision = 2;
  EXPECT_FALSE(ledger.accept(message));
  EXPECT_TRUE(ledger.active().empty());
  message.revision = 4;
  EXPECT_TRUE(ledger.accept(message));
  ASSERT_EQ(ledger.active().size(), 1U);
}

TEST(GraphSync, WithdrawalBeforeInitialDeliveryStillWins) {
  graph_sync::Ledger ledger;
  auto message = constraint(0, 2);
  message.retracted = true;
  ASSERT_TRUE(ledger.accept(message));
  EXPECT_FALSE(ledger.accept(constraint()));
  EXPECT_TRUE(ledger.active().empty());
}

TEST(GraphSync, InvalidRevisionDoesNotConsumeValidRetransmission) {
  graph_sync::Ledger ledger;
  auto message = constraint(0, 5);
  message.relative_pose.pose.orientation.w = 0;
  EXPECT_FALSE(ledger.accept(message));
  EXPECT_TRUE(ledger.accept(constraint(0, 5)));
}

TEST(GraphSync, NewAuthorityGenerationRetiresAllItsOldClaims) {
  graph_sync::Ledger ledger;
  ASSERT_TRUE(ledger.accept(constraint(0)));
  ASSERT_TRUE(ledger.accept(constraint(1)));
  auto newer = constraint(2);
  newer.authority_epoch = 31;
  ASSERT_TRUE(ledger.accept(newer));
  ASSERT_EQ(ledger.active().size(), 1U);
  EXPECT_EQ(ledger.active()[0].index_from, 2);
  EXPECT_FALSE(ledger.accept(constraint(0, 100)));
}

TEST(GraphSync, OwnerRestartInvalidatesOldKeyframeIdentities) {
  graph_sync::Ledger ledger;
  ASSERT_TRUE(ledger.accept(constraint()));
  ASSERT_TRUE(ledger.observeTrajectory("beta", 21));
  EXPECT_TRUE(ledger.active().empty());
  EXPECT_FALSE(ledger.accept(constraint(0, 100)));
  auto newer = constraint(0, 101);
  newer.to_trajectory_epoch = 21;
  EXPECT_TRUE(ledger.accept(newer));
  EXPECT_EQ(ledger.active().size(), 1U);
}

TEST(GraphSync, ReverseObserversDeduplicateWithoutRetractingEachOthersClaim) {
  graph_sync::Ledger ledger;
  auto first = constraint();
  ASSERT_TRUE(ledger.accept(first));
  auto reverse = first;
  std::swap(reverse.from_robot_id, reverse.to_robot_id);
  std::swap(reverse.from_trajectory_epoch, reverse.to_trajectory_epoch);
  std::swap(reverse.index_from, reverse.index_to);
  std::swap(reverse.from_pose, reverse.to_pose);
  reverse.authority_id = "alpha";
  ASSERT_TRUE(ledger.accept(reverse));
  ASSERT_EQ(ledger.active().size(), 1U);
  EXPECT_EQ(ledger.active()[0].authority_id, "alpha");
  first.retracted = true;
  first.revision = 2;
  ASSERT_TRUE(ledger.accept(first));
  EXPECT_EQ(ledger.active().size(), 1U);
  reverse.retracted = true;
  reverse.revision = 2;
  ASSERT_TRUE(ledger.accept(reverse));
  EXPECT_TRUE(ledger.active().empty());
}

TEST(GraphSync, BoundedReplayRecoversOfflineChangesAndAReceiverRestart) {
  graph_sync::Replay replay;
  for (int i = 0; i < 70; ++i) replay.put(constraint(i));
  graph_sync::Ledger receiver;
  for (const auto& message : replay.next(32)) receiver.accept(message);
  auto withdrawal = constraint(3, 2);
  withdrawal.retracted = true;
  replay.put(withdrawal);
  // Several batches are lost while the receiver is offline.
  replay.next(32);
  replay.next(32);
  receiver = graph_sync::Ledger();
  for (int pass = 0; pass < 3; ++pass) {
    const auto batch = replay.next(32);
    ASSERT_LE(batch.size(), 32U);
    for (const auto& message : batch) receiver.accept(message);
  }
  EXPECT_EQ(receiver.active().size(), 69U);
  EXPECT_FALSE(receiver.accept(constraint(3)));
  EXPECT_EQ(replay.latest().size(), 70U);
}

TEST(GraphSync, CorrectedPeerMotionAndRetractionChangeTheActualSolver) {
  gtsam::ISAM2 current;
  gtsam::NonlinearFactorGraph local;
  const auto x0 = graph_keys::localPose(0);
  const auto x1 = graph_keys::localPose(1);
  const auto tight = gtsam::noiseModel::Isotropic::Sigma(6, 0.001);
  const auto motion = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  local.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(x0, translation(0), tight);
  local.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(x0, x1, translation(10), motion);
  gtsam::Values initial;
  initial.insert(x0, translation(0));
  initial.insert(x1, translation(10));
  current.update(local, initial);
  graph_sync::Ledger ledger;
  ledger.accept(constraint(0));
  ledger.accept(constraint(1));
  auto fused = graph_sync::rebuild(current, ledger.active(), {}, "alpha", motion);
  EXPECT_NEAR(fused->calculateEstimate<gtsam::Pose3>(x1).x(), 9.0, 0.01);
  EXPECT_EQ(fused->calculateEstimate().size(), 4U);

  graph_sync::PoseStore corrections;
  corrections[{"beta", 20, 1}] = {2, translation(6)};
  fused = graph_sync::rebuild(*fused, ledger.active(), corrections, "alpha", motion);
  EXPECT_NEAR(fused->calculateEstimate<gtsam::Pose3>(x1).x(), 8.0, 0.01);
  // Rebuilding does not duplicate existing registrations or local factors.
  EXPECT_EQ(fused->getFactorsUnsafe().nrFactors(), 5U);

  auto withdrawal = constraint(1, 2);
  withdrawal.retracted = true;
  ledger.accept(withdrawal);
  fused = graph_sync::rebuild(*fused, ledger.active(), corrections, "alpha", motion);
  EXPECT_NEAR(fused->calculateEstimate<gtsam::Pose3>(x1).x(), 10.0, 0.001);
  EXPECT_EQ(fused->calculateEstimate().size(), 3U);
  withdrawal = constraint(0, 2);
  withdrawal.retracted = true;
  ledger.accept(withdrawal);
  fused = graph_sync::rebuild(*fused, ledger.active(), corrections, "alpha", motion);
  EXPECT_EQ(fused->calculateEstimate().size(), 2U);
  EXPECT_EQ(fused->getFactorsUnsafe().nrFactors(), 2U);
}

TEST(GraphSync, AFactorCanArriveBeforeItsLocalKeyframe) {
  gtsam::ISAM2 current;
  gtsam::NonlinearFactorGraph graph;
  const auto key = graph_keys::localPose(0);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      key, translation(0), gtsam::noiseModel::Isotropic::Sigma(6, 0.01));
  gtsam::Values values;
  values.insert(key, translation(0));
  current.update(graph, values);
  const auto rebuilt = graph_sync::rebuild(current, {constraint(5)}, {}, "alpha",
      gtsam::noiseModel::Isotropic::Sigma(6, 0.1));
  EXPECT_EQ(rebuilt->calculateEstimate().size(), 1U);
}

TEST(PcmRetraction, RequiresSustainedExclusionThenFullReadmissionTenure) {
  pcm::CommitmentTracker tracker({3, 2, 2});
  EXPECT_TRUE(tracker.update(4, {0, 1, 2}).empty());
  EXPECT_EQ(tracker.update(4, {0, 1, 2}), (std::vector<int>{0, 1, 2}));
  EXPECT_EQ(tracker.update(4, {1, 2, 3}), (std::vector<int>{0, 1, 2}));
  EXPECT_EQ(tracker.update(4, {1, 2, 3}), (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(tracker.update(4, {0, 1, 2}), (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(tracker.update(4, {0, 1, 2}), (std::vector<int>{0, 1, 2}));
  tracker.update(4, {});
  EXPECT_TRUE(tracker.update(4, {}).empty());
}

TEST(LoopConstraintMessage, NormalizesValidNonUnitQuaternions) {
  auto message = constraint();
  message.relative_pose.pose.orientation.w = 2;
  EXPECT_TRUE(loop_constraint::poseFromMessage(message.relative_pose.pose).equals(gtsam::Pose3()));
}
}  // namespace

#include "skid_message_validation.hpp"

TEST(MessageValidation, RejectsDescriptorLengthsBeforeEigenIndexing) {
  liorf::msg::ContextInfo message;
  message.robot_id = "jackal1";
  message.num_ring = 2;
  message.num_sector = 3;
  message.rsolid = {1, 2};
  message.asolid = {1, 2, 3};
  EXPECT_TRUE(liorf::messages::validContext(message, 2, 3, true));
  message.num_sector = 1000000;
  EXPECT_FALSE(liorf::messages::validContext(message, 2, 3, true));
  message.num_sector = 3;
  message.rsolid.pop_back();
  EXPECT_FALSE(liorf::messages::validContext(message, 2, 3, true));
  message.rsolid = {1, std::numeric_limits<float>::quiet_NaN()};
  EXPECT_FALSE(liorf::messages::validContext(message, 2, 3, true));
}

TEST(MessageValidation, RejectsPointCloudStrideOverflowAndTruncatedData) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.width = 1;
  cloud.height = 1;
  cloud.point_step = 16;
  cloud.row_step = 16;
  cloud.data.resize(16);
  EXPECT_TRUE(liorf::messages::validCloud(cloud));
  cloud.data.pop_back();
  EXPECT_FALSE(liorf::messages::validCloud(cloud));
  cloud.data.resize(16);
  cloud.width = std::numeric_limits<std::uint32_t>::max();
  EXPECT_FALSE(liorf::messages::validCloud(cloud));
  cloud.width = 1;
  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 15;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  cloud.fields.push_back(field);
  EXPECT_FALSE(liorf::messages::validCloud(cloud));
}

#include "skid_map_alignment.hpp"
TEST(MapAlignment, UsesConfiguredAnchorAndLeavesDisconnectedMapsDetached) {
  const std::vector<liorf::map_alignment::Edge> edges{
      {4, 5, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 0, 0))},
      {7, 8, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(10, 0, 0))},
      {5, 6, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 0, 0))}};
  const auto solved = liorf::map_alignment::solve(edges, 4, 6);
  ASSERT_TRUE(solved.has_value());
  EXPECT_NEAR(solved->x(), 5, 1e-6);
  EXPECT_FALSE(liorf::map_alignment::solve(edges, 4, 8).has_value());
  EXPECT_FALSE(liorf::map_alignment::solve({}, 4, 6).has_value());
}
