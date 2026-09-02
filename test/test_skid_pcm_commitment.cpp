#include "skid_pcm_commitment.hpp"

#include <gtest/gtest.h>

TEST(PcmCommitment, RejectsInvalidPolicy) {
  EXPECT_FALSE(liorf::pcm::validate({1, 1}).empty());
  EXPECT_FALSE(liorf::pcm::validate({5, 0}).empty());
  EXPECT_TRUE(liorf::pcm::validate({5, 3}).empty());
}

TEST(PcmCommitment, DoesNotCommitAStableButUndersizedClique) {
  liorf::pcm::CommitmentTracker tracker({4, 2});

  EXPECT_TRUE(tracker.update(3, {0, 1, 2}).empty());
  EXPECT_TRUE(tracker.update(3, {0, 1, 2}).empty());
  EXPECT_TRUE(tracker.update(3, {0, 1, 2}).empty());
}

TEST(PcmCommitment, UndersizedCliqueTenureDoesNotCarryIntoPublication) {
  liorf::pcm::CommitmentTracker tracker({4, 2});

  EXPECT_TRUE(tracker.update(3, {0, 1, 2}).empty());
  EXPECT_TRUE(tracker.update(3, {0, 1, 2}).empty());
  EXPECT_TRUE(tracker.update(4, {0, 1, 2, 3}).empty());
  EXPECT_EQ(tracker.update(4, {0, 1, 2, 3}),
            (std::vector<int>{0, 1, 2, 3}));
}

TEST(PcmCommitment, RequiresRepeatedMembershipInASupportedClique) {
  liorf::pcm::CommitmentTracker tracker({4, 3});

  EXPECT_TRUE(tracker.update(4, {0, 1, 2, 3}).empty());
  EXPECT_TRUE(tracker.update(5, {0, 1, 2, 3}).empty());
  EXPECT_EQ(tracker.update(6, {0, 1, 2, 3}),
            (std::vector<int>{0, 1, 2, 3}));
}

TEST(PcmCommitment, ResetsTenureWhenACandidateLeavesTheClique) {
  liorf::pcm::CommitmentTracker tracker({3, 3});

  EXPECT_TRUE(tracker.update(4, {0, 1, 2}).empty());
  EXPECT_TRUE(tracker.update(4, {0, 1, 2}).empty());
  EXPECT_EQ(tracker.update(4, {0, 2, 3}),
            (std::vector<int>{0, 2}));
  EXPECT_EQ(tracker.update(4, {0, 1, 2, 3}),
            (std::vector<int>{0, 2}));
  EXPECT_EQ(tracker.update(4, {0, 1, 2, 3}),
            (std::vector<int>{0, 2, 3}));
  EXPECT_EQ(tracker.update(4, {0, 1, 2, 3}),
            (std::vector<int>{0, 1, 2, 3}));
}

TEST(PcmCommitment, CommitmentsStayLatchedWhenCliqueMembershipChanges) {
  liorf::pcm::CommitmentTracker tracker({3, 1});

  EXPECT_EQ(tracker.update(4, {0, 1, 2}),
            (std::vector<int>{0, 1, 2}));
  EXPECT_EQ(tracker.update(4, {1, 2, 3}),
            (std::vector<int>{0, 1, 2, 3}));
  EXPECT_EQ(tracker.update(4, {}),
            (std::vector<int>{0, 1, 2, 3}));
}
