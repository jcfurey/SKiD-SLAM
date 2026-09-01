#pragma once

#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "skid_loop_detection.hpp"

// Thin adapter over the vendored SOLiD implementation.
//
// SOLiD's own header declares global `PointType` and `PointXYZIRPYT` symbols
// that collide with the mapping node's definitions, so it is included by
// exactly one translation unit and reached through this interface everywhere
// else.
namespace liorf::solid {

struct Params {
  // Range bound used to bin points, in metres.
  int max_range_m = 80;
  // Length of the R-SOLiD key; must match the loop-detection configuration.
  int knn_feature_dim = 40;
  // Length of the A-SOLiD key; must match the loop-detection configuration.
  int num_sectors = 60;
  // Elevation bins spanning [fov_down_deg, fov_up_deg].
  int num_heights = 64;
  double fov_up_deg = 2.0;
  double fov_down_deg = -24.8;
  // Scans below this size cannot fill enough bins to describe a place.
  int min_points = 100;
};

// Returns an empty string when every parameter value is valid.
std::string validate(const Params& params);

// Builds the SOLiD descriptor pair for `cloud`, which is read but not
// modified. Returns a descriptor with empty vectors when the scan is too
// small or the result is not finite: SOLiD normalizes its elevation weights
// by the observed occupancy range, which collapses to a division by zero for
// degenerate scans.
liorf::loop_detection::Descriptor describe(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
  const Params& params);

}  // namespace liorf::solid
