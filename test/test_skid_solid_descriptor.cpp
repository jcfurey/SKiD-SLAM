#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "skid_solid_descriptor.hpp"

namespace {

using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

constexpr int kSectors = 12;
constexpr int kRangeBins = 8;
constexpr int kHeights = 8;
constexpr int kMaxRange = 40;
constexpr double kFovUp = 15.0;
constexpr double kFovDown = -15.0;

liorf::solid::Params testParams() {
  liorf::solid::Params params;
  params.max_range_m = kMaxRange;
  params.knn_feature_dim = kRangeBins;
  params.num_sectors = kSectors;
  params.num_heights = kHeights;
  params.fov_up_deg = kFovUp;
  params.fov_down_deg = kFovDown;
  params.min_points = 100;
  return params;
}

void addPoint(
  const PointCloud::Ptr& cloud,
  double range_m,
  double azimuth_deg,
  double elevation_deg) {
  const double azimuth = azimuth_deg * M_PI / 180.0;
  const double elevation = elevation_deg * M_PI / 180.0;
  pcl::PointXYZI point;
  point.x = static_cast<float>(range_m * std::cos(azimuth));
  point.y = static_cast<float>(range_m * std::sin(azimuth));
  point.z = static_cast<float>(range_m * std::tan(elevation));
  point.intensity = 1.0F;
  cloud->push_back(point);
}

// A scene rotated `sector_offset` sectors about the sensor's z axis.
//
// Points sit at sector centres so a whole-sector rotation moves occupancy
// between bins without straddling a boundary. Occupancy grows with both the
// sector and the elevation bin: the elevation contrast is what lets SOLiD
// normalize its elevation weights, and the azimuth contrast is what makes the
// scene asymmetric, so exactly one circular shift can explain a rotation.
PointCloud::Ptr makeScene(int sector_offset) {
  const double sector_width_deg = 360.0 / kSectors;
  const double height_width_deg = (kFovUp - kFovDown) / kHeights;
  PointCloud::Ptr cloud(new PointCloud());
  for (int sector = 0; sector < kSectors; ++sector) {
    double azimuth_deg =
      (static_cast<double>(sector + sector_offset) + 0.5) * sector_width_deg;
    azimuth_deg = std::fmod(std::fmod(azimuth_deg, 360.0) + 360.0, 360.0);
    for (int height = 0; height < kHeights; ++height) {
      const double elevation_deg =
        kFovDown + (static_cast<double>(height) + 0.5) * height_width_deg;
      for (int k = 0; k <= height + sector; ++k) {
        addPoint(cloud, 3.0 + 2.0 * static_cast<double>(k),
                 azimuth_deg, elevation_deg);
      }
    }
  }
  return cloud;
}

}  // namespace

TEST(SkidSolidDescriptorConfig, RejectsInvalidValues) {
  EXPECT_TRUE(liorf::solid::validate(testParams()).empty());

  liorf::solid::Params params = testParams();
  params.num_sectors = 0;
  EXPECT_FALSE(liorf::solid::validate(params).empty());

  params = testParams();
  params.fov_up_deg = params.fov_down_deg;
  EXPECT_FALSE(liorf::solid::validate(params).empty());

  params = testParams();
  params.min_points = 0;
  EXPECT_FALSE(liorf::solid::validate(params).empty());
}

TEST(SkidSolidDescriptor, DescribesAScanWithTheConfiguredDimensions) {
  const liorf::loop_detection::Descriptor descriptor =
    liorf::solid::describe(makeScene(0), testParams());

  ASSERT_TRUE(descriptor.sized(kRangeBins, kSectors));
  EXPECT_TRUE(descriptor.finite());
  EXPECT_GT(descriptor.range.norm(), 0.0F);
  EXPECT_GT(descriptor.angular.norm(), 0.0F);
}

TEST(SkidSolidDescriptor, RejectsScansThatCannotDescribeAPlace) {
  const liorf::solid::Params params = testParams();

  EXPECT_EQ(0, liorf::solid::describe(nullptr, params).range.size());

  PointCloud::Ptr sparse(new PointCloud());
  addPoint(sparse, 5.0, 10.0, 0.0);
  EXPECT_EQ(0, liorf::solid::describe(sparse, params).range.size());

  liorf::solid::Params invalid = params;
  invalid.num_heights = 0;
  EXPECT_EQ(0, liorf::solid::describe(makeScene(0), invalid).range.size());
}

TEST(SkidSolidDescriptor, RejectsScansWithNoElevationContrast) {
  // SOLiD normalizes its elevation weights by the observed occupancy range,
  // so a scan whose elevation bins are all equally full divides by zero.
  const double height_width_deg = (kFovUp - kFovDown) / kHeights;
  PointCloud::Ptr uniform(new PointCloud());
  for (int height = 0; height < kHeights; ++height) {
    const double elevation_deg =
      kFovDown + (static_cast<double>(height) + 0.5) * height_width_deg;
    for (int i = 0; i < 16; ++i) {
      addPoint(uniform, 5.0 + static_cast<double>(i), 45.0, elevation_deg);
    }
  }
  ASSERT_GE(uniform->size(), 100u);

  EXPECT_EQ(0, liorf::solid::describe(uniform, testParams()).range.size());
}

TEST(SkidSolidDescriptor, YawLeavesTheRangeKeyAloneAndShiftsTheAngleKey) {
  const liorf::solid::Params params = testParams();
  constexpr int kRotationSectors = 3;

  const liorf::loop_detection::Descriptor stored =
    liorf::solid::describe(makeScene(0), params);
  const liorf::loop_detection::Descriptor revisit =
    liorf::solid::describe(makeScene(kRotationSectors), params);
  ASSERT_TRUE(stored.sized(kRangeBins, kSectors));
  ASSERT_TRUE(revisit.sized(kRangeBins, kSectors));

  // R-SOLiD ignores azimuth, which is what makes it usable as a retrieval key.
  EXPECT_NEAR(0.0F, (stored.range - revisit.range).cwiseAbs().maxCoeff(),
              1.0e-3F);

  // Observing the scene `kRotationSectors` sectors further round means the
  // revisit's occupancy must be rotated back by that much to explain the
  // stored place.
  const liorf::loop_detection::Comparison comparison =
    liorf::loop_detection::compare(revisit, stored);
  ASSERT_TRUE(comparison.valid());
  EXPECT_NEAR(0.0F, comparison.range_distance, 1.0e-5F);
  EXPECT_EQ(kSectors - kRotationSectors, comparison.sector_shift);
  EXPECT_NEAR(
    liorf::loop_detection::sectorShiftToYaw(
      kSectors - kRotationSectors, kSectors),
    comparison.yaw_rad,
    1.0e-6F);
}

TEST(SkidSolidDescriptor, RetrievesTheRotatedRevisitFromAnIndex) {
  const liorf::solid::Params params = testParams();

  liorf::loop_detection::Config detection;
  detection.knn_feature_dim = kRangeBins;
  detection.num_sectors = kSectors;
  detection.num_nearest_matches = 4;
  detection.num_match_candidates = 1;
  detection.distance_threshold = 0.05F;
  detection.min_index_gap = 0;
  detection.min_time_gap_s = 0.0;
  liorf::loop_detection::Index index(detection);
  ASSERT_TRUE(index.error().empty());

  ASSERT_EQ(0u, index.add(liorf::solid::describe(makeScene(0), params), 0.0));
  ASSERT_EQ(1u, index.add(liorf::solid::describe(makeScene(6), params), 10.0));

  const auto candidates = index.searchLatest();
  ASSERT_EQ(1u, candidates.size());
  EXPECT_EQ(0u, candidates[0].index);
  EXPECT_NEAR(0.0F, candidates[0].range_distance, 1.0e-5F);
}
