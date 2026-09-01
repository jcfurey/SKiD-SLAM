#include <map>
#include <stdexcept>
#include <string>
#include <variant>

#include <gtest/gtest.h>

#include "skid_registration_params.hpp"

namespace {

// Stands in for a ROS node: records every declaration and serves overrides,
// so the shared helper can be exercised without a running node.
class FakeParameterServer {
 public:
  using Value = std::variant<bool, int, double>;

  void override_parameter(const std::string& name, Value value) {
    overrides_[name] = value;
  }

  const std::map<std::string, Value>& declared() const { return declared_; }

  template <typename T>
  T get(const std::string& name, const T& default_value) const {
    const auto override_it = overrides_.find(name);
    if (override_it != overrides_.end()) {
      return std::get<T>(override_it->second);
    }
    return default_value;
  }

  auto declarer() {
    return [this](const std::string& name, auto default_value) {
      using T = decltype(default_value);
      const T value = this->get<T>(name, default_value);
      this->declared_[name] = Value(value);
      return value;
    };
  }

 private:
  std::map<std::string, Value> overrides_;
  std::map<std::string, Value> declared_;
};

constexpr const char* kPrefix = "loopClosure.registration.";

}  // namespace

TEST(SkidRegistrationParams, DefaultsMatchTheDocumentedConfiguration) {
  FakeParameterServer server;
  const liorf::registration::Config config =
    liorf::registration::declareConfig(server.declarer(), kPrefix, 3.0);

  const liorf::registration::Config defaults;
  EXPECT_FLOAT_EQ(defaults.coarse_voxel_size_m, config.coarse_voxel_size_m);
  EXPECT_EQ(defaults.coarse_use_voxel_sampling,
            config.coarse_use_voxel_sampling);
  EXPECT_EQ(defaults.coarse_max_correspondences,
            config.coarse_max_correspondences);
  EXPECT_FLOAT_EQ(defaults.coarse_solver_noise_bound_gain,
                  config.coarse_solver_noise_bound_gain);
  EXPECT_EQ(defaults.min_coarse_correspondences,
            config.min_coarse_correspondences);
  EXPECT_EQ(defaults.min_fine_inliers, config.min_fine_inliers);
  EXPECT_EQ(defaults.min_metric_inliers, config.min_metric_inliers);
  EXPECT_DOUBLE_EQ(defaults.fine_max_correspondence_distance_m,
                   config.fine_max_correspondence_distance_m);
  EXPECT_EQ(defaults.fine_max_iterations, config.fine_max_iterations);
  EXPECT_DOUBLE_EQ(defaults.min_overlap_ratio, config.min_overlap_ratio);
  EXPECT_DOUBLE_EQ(defaults.nominal_translation_stddev_m,
                   config.nominal_translation_stddev_m);
  EXPECT_DOUBLE_EQ(defaults.uncertainty_max_variance_scale,
                   config.uncertainty_max_variance_scale);

  // The truncated-MSE gate is the one default the caller supplies, so each
  // node can keep its own historical per-dataset threshold.
  EXPECT_DOUBLE_EQ(3.0, config.max_truncated_mse_m2);
}

TEST(SkidRegistrationParams, EveryParameterIsDeclaredUnderThePrefix) {
  FakeParameterServer server;
  liorf::registration::declareConfig(server.declarer(), kPrefix, 3.0);

  ASSERT_FALSE(server.declared().empty());
  for (const auto& entry : server.declared()) {
    EXPECT_EQ(0u, entry.first.rfind(kPrefix, 0))
      << entry.first << " is declared outside the prefix";
  }
  // One parameter per Config field: every registration knob the paper
  // pipeline reads is reachable from a parameter file, and both nodes offer
  // the same set.
  EXPECT_EQ(29u, server.declared().size());
}

TEST(SkidRegistrationParams, OverridesReachTheConfiguration) {
  FakeParameterServer server;
  server.override_parameter(
    std::string(kPrefix) + "coarse_voxel_size_m", 1.25);
  server.override_parameter(
    std::string(kPrefix) + "coarse_use_quatro", true);
  server.override_parameter(
    std::string(kPrefix) + "fine_num_threads", 8);
  server.override_parameter(
    std::string(kPrefix) + "max_truncated_mse_m2", 0.75);
  server.override_parameter(
    std::string(kPrefix) + "min_metric_inliers", 42);

  const liorf::registration::Config config =
    liorf::registration::declareConfig(server.declarer(), kPrefix, 3.0);

  EXPECT_FLOAT_EQ(1.25F, config.coarse_voxel_size_m);
  EXPECT_TRUE(config.coarse_use_quatro);
  EXPECT_EQ(8, config.fine_num_threads);
  EXPECT_DOUBLE_EQ(0.75, config.max_truncated_mse_m2);
  EXPECT_EQ(42u, config.min_metric_inliers);
}

TEST(SkidRegistrationParams, NegativeCountsAreRejected) {
  FakeParameterServer server;
  server.override_parameter(
    std::string(kPrefix) + "min_coarse_inliers", -1);

  EXPECT_THROW(
    liorf::registration::declareConfig(server.declarer(), kPrefix, 3.0),
    std::invalid_argument);
}

TEST(SkidRegistrationParams, InvalidValuesAreRejectedByTheSharedValidator) {
  FakeParameterServer server;
  server.override_parameter(
    std::string(kPrefix) + "min_overlap_ratio", 1.5);

  EXPECT_THROW(
    liorf::registration::declareConfig(server.declarer(), kPrefix, 3.0),
    std::invalid_argument);
}
