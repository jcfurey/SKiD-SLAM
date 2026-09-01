#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "skid_loop_detection.hpp"

namespace {

using liorf::loop_detection::Candidate;
using liorf::loop_detection::Comparison;
using liorf::loop_detection::Config;
using liorf::loop_detection::Descriptor;
using liorf::loop_detection::Index;

constexpr int kFeatureDim = 8;
constexpr int kSectors = 12;

Config testConfig() {
  Config config;
  config.knn_feature_dim = kFeatureDim;
  config.num_sectors = kSectors;
  config.num_nearest_matches = 4;
  config.num_match_candidates = 2;
  config.distance_threshold = 0.2F;
  config.min_index_gap = 3;
  config.min_time_gap_s = 5.0;
  return config;
}

// A place whose range key is driven by `seed` and whose angle key is a single
// bump in sector `bump_sector`, so a known yaw is recoverable.
Descriptor makePlace(float seed, int bump_sector) {
  Descriptor descriptor;
  descriptor.range = Eigen::VectorXf::Zero(kFeatureDim);
  for (int i = 0; i < kFeatureDim; ++i) {
    descriptor.range(i) = 1.0F + seed * static_cast<float>(i);
  }
  descriptor.angular = Eigen::VectorXf::Constant(kSectors, 0.1F);
  descriptor.angular(((bump_sector % kSectors) + kSectors) % kSectors) = 4.0F;
  return descriptor;
}

}  // namespace

TEST(SkidLoopDetectionConfig, RejectsInvalidValues) {
  EXPECT_TRUE(liorf::loop_detection::validate(testConfig()).empty());

  Config config = testConfig();
  config.num_match_candidates = config.num_nearest_matches + 1;
  EXPECT_FALSE(liorf::loop_detection::validate(config).empty());

  config = testConfig();
  config.distance_threshold = 0.0F;
  EXPECT_FALSE(liorf::loop_detection::validate(config).empty());

  config = testConfig();
  config.min_time_gap_s = std::nan("");
  EXPECT_FALSE(liorf::loop_detection::validate(config).empty());
}

TEST(SkidLoopDetectionYaw, MapsSectorShiftsIntoWrappedRadians) {
  EXPECT_FLOAT_EQ(0.0F, liorf::loop_detection::sectorShiftToYaw(0, kSectors));
  EXPECT_FLOAT_EQ(
    static_cast<float>(2.0 * M_PI / kSectors),
    liorf::loop_detection::sectorShiftToYaw(1, kSectors));
  // Half a turn stays at +pi; anything beyond wraps negative.
  EXPECT_FLOAT_EQ(
    static_cast<float>(M_PI),
    liorf::loop_detection::sectorShiftToYaw(kSectors / 2, kSectors));
  EXPECT_FLOAT_EQ(
    static_cast<float>(-2.0 * M_PI / kSectors),
    liorf::loop_detection::sectorShiftToYaw(kSectors - 1, kSectors));
  EXPECT_FLOAT_EQ(
    0.0F, liorf::loop_detection::sectorShiftToYaw(kSectors, kSectors));
}

TEST(SkidLoopDetectionCompare, IdenticalPlacesHaveZeroDistanceAndZeroYaw) {
  const Descriptor place = makePlace(0.3F, 2);
  const Comparison comparison = liorf::loop_detection::compare(place, place);
  ASSERT_TRUE(comparison.valid());
  EXPECT_NEAR(0.0F, comparison.range_distance, 1.0e-6F);
  EXPECT_EQ(0, comparison.sector_shift);
  EXPECT_NEAR(0.0F, comparison.yaw_rad, 1.0e-6F);
}

TEST(SkidLoopDetectionCompare, RecoversTheYawBetweenTwoObservations) {
  // The candidate observes the same place with its bump three sectors later,
  // so the query must be rotated by three sectors of azimuth to explain it.
  const Descriptor query = makePlace(0.3F, 1);
  const Descriptor candidate = makePlace(0.3F, 4);

  const Comparison comparison =
    liorf::loop_detection::compare(query, candidate);
  ASSERT_TRUE(comparison.valid());
  EXPECT_EQ(3, comparison.sector_shift);
  EXPECT_NEAR(
    static_cast<float>(3.0 * 2.0 * M_PI / kSectors),
    comparison.yaw_rad,
    1.0e-6F);
  // Yaw does not change the range key.
  EXPECT_NEAR(0.0F, comparison.range_distance, 1.0e-6F);
}

TEST(SkidLoopDetectionCompare, RejectsUnusableDescriptors) {
  const Descriptor place = makePlace(0.3F, 1);

  Descriptor mismatched = place;
  mismatched.range = Eigen::VectorXf::Ones(kFeatureDim + 1);
  EXPECT_FALSE(liorf::loop_detection::compare(place, mismatched).valid());

  Descriptor zero = place;
  zero.range = Eigen::VectorXf::Zero(kFeatureDim);
  EXPECT_FALSE(liorf::loop_detection::compare(place, zero).valid());

  Descriptor nonfinite = place;
  nonfinite.range(0) = std::nan("");
  EXPECT_FALSE(liorf::loop_detection::compare(place, nonfinite).valid());
}

TEST(SkidLoopDetectionIndex, RejectsMisshapenOrNonFiniteEntries) {
  Index index(testConfig());
  ASSERT_TRUE(index.error().empty());

  Descriptor wrong_size = makePlace(0.3F, 0);
  wrong_size.angular = Eigen::VectorXf::Ones(kSectors + 1);
  EXPECT_EQ(static_cast<std::size_t>(-1), index.add(wrong_size, 0.0));

  Descriptor nonfinite = makePlace(0.3F, 0);
  nonfinite.angular(0) = std::nan("");
  EXPECT_EQ(static_cast<std::size_t>(-1), index.add(nonfinite, 0.0));

  Descriptor zero_range = makePlace(0.3F, 0);
  zero_range.range = Eigen::VectorXf::Zero(kFeatureDim);
  EXPECT_EQ(static_cast<std::size_t>(-1), index.add(zero_range, 0.0));

  EXPECT_EQ(static_cast<std::size_t>(-1),
            index.add(makePlace(0.3F, 0), std::nan("")));

  EXPECT_EQ(0u, index.size());
}

TEST(SkidLoopDetectionIndex, InvalidConfigurationStoresNothing) {
  Config config = testConfig();
  config.knn_feature_dim = 0;
  Index index(config);
  EXPECT_FALSE(index.error().empty());
  EXPECT_EQ(static_cast<std::size_t>(-1), index.add(makePlace(0.3F, 0), 0.0));
  EXPECT_TRUE(index.searchLatest().empty());
}

TEST(SkidLoopDetectionIndex, SkipsNeighborsInsideTheRevisitExclusion) {
  Index index(testConfig());
  // Ten observations of one place, one second apart. Every entry is a perfect
  // descriptor match, so only the exclusion policy can filter them.
  for (int i = 0; i < 10; ++i) {
    ASSERT_EQ(static_cast<std::size_t>(i),
              index.add(makePlace(0.3F, 0), static_cast<double>(i)));
  }

  // The 5 s time gap dominates the 3-keyframe index gap here.
  const std::vector<Candidate> candidates = index.searchLatest();
  ASSERT_EQ(2u, candidates.size());
  EXPECT_LE(candidates[0].index, 4u);
  EXPECT_LE(candidates[1].index, 4u);
}

TEST(SkidLoopDetectionIndex, ReturnsTheClosestDescriptorFirst) {
  Config config = testConfig();
  config.min_index_gap = 0;
  config.min_time_gap_s = 0.0;
  Index index(config);

  // Three distinct places, then a revisit of the middle one.
  index.add(makePlace(0.05F, 0), 0.0);   // 0
  index.add(makePlace(0.90F, 0), 1.0);   // 1
  index.add(makePlace(2.50F, 0), 2.0);   // 2
  const Descriptor revisit = makePlace(0.90F, 5);
  index.add(revisit, 3.0);               // 3

  const std::vector<Candidate> candidates = index.searchLatest();
  ASSERT_FALSE(candidates.empty());
  EXPECT_EQ(1u, candidates[0].index);
  EXPECT_NEAR(0.0F, candidates[0].range_distance, 1.0e-6F);
  // The revisit's bump sits in sector 5 and the stored place's in sector 0, so
  // the query rotates seven sectors forward, which wraps past pi to a negative
  // yaw.
  EXPECT_EQ(7, candidates[0].sector_shift);
  EXPECT_NEAR(
    static_cast<float>(7.0 * 2.0 * M_PI / kSectors - 2.0 * M_PI),
    candidates[0].yaw_rad,
    1.0e-6F);
  for (std::size_t i = 1; i < candidates.size(); ++i) {
    EXPECT_LE(candidates[0].range_distance, candidates[i].range_distance);
  }
}

TEST(SkidLoopDetectionIndex, RejectsCandidatesBeyondTheDistanceThreshold) {
  Config config = testConfig();
  config.min_index_gap = 0;
  config.min_time_gap_s = 0.0;
  config.distance_threshold = 1.0e-4F;
  Index index(config);

  index.add(makePlace(0.05F, 0), 0.0);
  index.add(makePlace(5.00F, 0), 1.0);

  EXPECT_TRUE(index.searchLatest().empty());
}

TEST(SkidLoopDetectionIndex, HonorsTheCandidateBudget) {
  Config config = testConfig();
  config.min_index_gap = 0;
  config.min_time_gap_s = 0.0;
  config.num_match_candidates = 1;
  Index index(config);

  for (int i = 0; i < 6; ++i) {
    index.add(makePlace(0.3F, i), static_cast<double>(i));
  }

  EXPECT_EQ(1u, index.searchLatest().size());
}

TEST(SkidLoopDetectionIndex, ClearDropsEveryStoredPlace) {
  Config config = testConfig();
  config.min_index_gap = 0;
  config.min_time_gap_s = 0.0;
  Index index(config);
  index.add(makePlace(0.3F, 0), 0.0);
  index.add(makePlace(0.3F, 0), 1.0);
  ASSERT_FALSE(index.searchLatest().empty());

  index.clear();
  EXPECT_EQ(0u, index.size());
  EXPECT_TRUE(index.searchLatest().empty());
}
