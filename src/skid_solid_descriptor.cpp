#include "skid_solid_descriptor.hpp"

#include <cmath>
#include <memory>
#include <sstream>

#include "SOLiD/solid.h"

namespace liorf::solid {

std::string validate(const Params& params) {
  std::ostringstream errors;
  const auto require = [&errors](bool condition, const char* message) {
    if (!condition) {
      errors << (errors.tellp() == std::streampos(0) ? "" : "; ") << message;
    }
  };

  require(params.max_range_m > 0, "max_range_m must be positive");
  require(params.knn_feature_dim > 0, "knn_feature_dim must be positive");
  require(params.num_sectors > 0, "num_sectors must be positive");
  require(params.num_heights > 0, "num_heights must be positive");
  require(std::isfinite(params.fov_up_deg) && std::isfinite(params.fov_down_deg),
          "fov_up_deg and fov_down_deg must be finite");
  require(params.fov_up_deg > params.fov_down_deg,
          "fov_up_deg must exceed fov_down_deg");
  require(params.min_points > 0, "min_points must be positive");

  return errors.str();
}

liorf::loop_detection::Descriptor describe(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
  const Params& params) {
  liorf::loop_detection::Descriptor descriptor;
  if (!validate(params).empty() || !cloud ||
      static_cast<int>(cloud->size()) < params.min_points) {
    return descriptor;
  }

  // ptcloud2bin swaps the shared pointer it is handed into the bin it returns,
  // so the bin aliases this cloud rather than copying it. The bin is read-only
  // here and dies with the call, leaving the caller's cloud untouched.
  SOLiD factory(
    params.max_range_m, params.knn_feature_dim, params.num_sectors);
  const SOLiDBin bin = factory.ptcloud2bin(
    cloud,
    params.knn_feature_dim,
    params.num_sectors,
    params.num_heights,
    static_cast<float>(params.fov_up_deg),
    static_cast<float>(params.fov_down_deg),
    params.max_range_m);

  if (bin.rsolid.size() != params.knn_feature_dim ||
      bin.asolid.size() != params.num_sectors ||
      !bin.rsolid.allFinite() || !bin.asolid.allFinite() ||
      bin.rsolid.norm() <= 0.0F) {
    return descriptor;
  }

  descriptor.range = bin.rsolid;
  descriptor.angular = bin.asolid;
  return descriptor;
}

}  // namespace liorf::solid
