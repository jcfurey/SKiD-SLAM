#include "skid_loop_detection.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace liorf::loop_detection {
namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 2.0F * kPi;

Eigen::VectorXf normalized(const Eigen::VectorXf& vector) {
  const float norm = vector.norm();
  if (!std::isfinite(norm) || norm <= 0.0F) {
    return Eigen::VectorXf::Zero(vector.size());
  }
  return vector / norm;
}

}  // namespace

bool Descriptor::sized(int knn_feature_dim, int num_sectors) const noexcept {
  return knn_feature_dim > 0 && num_sectors > 0 &&
         range.size() == knn_feature_dim && angular.size() == num_sectors;
}

bool Descriptor::finite() const noexcept {
  return range.allFinite() && angular.allFinite();
}

bool Comparison::valid() const noexcept {
  return std::isfinite(range_distance) && std::isfinite(angular_distance);
}

std::string validate(const Config& config) {
  std::ostringstream errors;
  const auto require = [&errors](bool condition, const char* message) {
    if (!condition) {
      errors << (errors.tellp() == std::streampos(0) ? "" : "; ") << message;
    }
  };

  require(config.knn_feature_dim > 0, "knn_feature_dim must be positive");
  require(config.num_sectors > 0, "num_sectors must be positive");
  require(config.num_nearest_matches > 0,
          "num_nearest_matches must be positive");
  require(config.num_match_candidates > 0,
          "num_match_candidates must be positive");
  require(config.num_match_candidates <= config.num_nearest_matches,
          "num_match_candidates must not exceed num_nearest_matches");
  require(std::isfinite(config.distance_threshold) &&
            config.distance_threshold > 0.0F,
          "distance_threshold must be finite and positive");
  require(config.min_index_gap >= 0, "min_index_gap must not be negative");
  require(std::isfinite(config.min_time_gap_s) && config.min_time_gap_s >= 0.0,
          "min_time_gap_s must be finite and non-negative");

  return errors.str();
}

float sectorShiftToYaw(int sector_shift, int num_sectors) noexcept {
  if (num_sectors <= 0) {
    return 0.0F;
  }
  const int wrapped =
    ((sector_shift % num_sectors) + num_sectors) % num_sectors;
  float yaw = static_cast<float>(wrapped) * kTwoPi /
              static_cast<float>(num_sectors);
  if (yaw > kPi) {
    yaw -= kTwoPi;
  }
  return yaw;
}

Comparison compare(const Descriptor& query, const Descriptor& candidate) {
  Comparison comparison;
  if (query.range.size() == 0 || candidate.range.size() == 0 ||
      query.range.size() != candidate.range.size() ||
      query.angular.size() == 0 || candidate.angular.size() == 0 ||
      query.angular.size() != candidate.angular.size() ||
      !query.finite() || !candidate.finite()) {
    return comparison;
  }

  const float query_norm = query.range.norm();
  const float candidate_norm = candidate.range.norm();
  if (!std::isfinite(query_norm) || !std::isfinite(candidate_norm) ||
      query_norm <= 0.0F || candidate_norm <= 0.0F) {
    return comparison;
  }

  const float similarity =
    query.range.dot(candidate.range) / (query_norm * candidate_norm);
  if (!std::isfinite(similarity)) {
    return comparison;
  }
  comparison.range_distance = 1.0F - similarity;

  // Circular shift of the query's angle-wise key that best explains the
  // candidate's. Sector `i` of the query lands on sector `i + shift` of the
  // candidate, so `shift` sector widths is the azimuth rotation from the query
  // frame into the candidate frame.
  const int num_sectors = static_cast<int>(query.angular.size());
  float best_distance = std::numeric_limits<float>::infinity();
  int best_shift = 0;
  Eigen::VectorXf shifted(num_sectors);
  for (int shift = 0; shift < num_sectors; ++shift) {
    for (int i = 0; i < num_sectors; ++i) {
      shifted((i + shift) % num_sectors) = query.angular(i);
    }
    const float distance = (candidate.angular - shifted).cwiseAbs().sum();
    if (distance < best_distance) {
      best_distance = distance;
      best_shift = shift;
    }
  }

  comparison.angular_distance = best_distance;
  comparison.sector_shift = best_shift;
  comparison.yaw_rad = sectorShiftToYaw(best_shift, num_sectors);
  return comparison;
}

Index::Index(const Config& config)
  : config_(config), error_(validate(config)) {}

std::size_t Index::add(const Descriptor& descriptor, double time_s) {
  constexpr std::size_t kRejected = static_cast<std::size_t>(-1);
  if (!error_.empty()) {
    return kRejected;
  }
  if (!descriptor.sized(config_.knn_feature_dim, config_.num_sectors) ||
      !descriptor.finite() || !std::isfinite(time_s)) {
    return kRejected;
  }

  Eigen::VectorXf key = normalized(descriptor.range);
  if (key.isZero(0.0F)) {
    return kRejected;
  }

  descriptors_.push_back(descriptor);
  normalized_range_.push_back(std::move(key));
  times_.push_back(time_s);
  return descriptors_.size() - 1;
}

const Descriptor& Index::descriptor(std::size_t index) const {
  if (index >= descriptors_.size()) {
    throw std::out_of_range("liorf::loop_detection::Index::descriptor");
  }
  return descriptors_[index];
}

double Index::time(std::size_t index) const {
  if (index >= times_.size()) {
    throw std::out_of_range("liorf::loop_detection::Index::time");
  }
  return times_[index];
}

std::vector<Candidate> Index::search(
  const Descriptor& descriptor,
  double time_s,
  std::size_t query_index) const {
  std::vector<Candidate> candidates;
  if (!error_.empty() || descriptors_.empty()) {
    return candidates;
  }
  if (!descriptor.sized(config_.knn_feature_dim, config_.num_sectors) ||
      !descriptor.finite() || !std::isfinite(time_s)) {
    return candidates;
  }

  const Eigen::VectorXf key = normalized(descriptor.range);
  if (key.isZero(0.0F)) {
    return candidates;
  }

  // Shortlist by Euclidean distance between normalized yaw-invariant keys.
  std::vector<std::pair<float, std::size_t>> shortlist;
  shortlist.reserve(descriptors_.size());
  for (std::size_t i = 0; i < descriptors_.size(); ++i) {
    if (i == query_index) {
      continue;
    }
    const std::ptrdiff_t index_gap =
      static_cast<std::ptrdiff_t>(query_index) -
      static_cast<std::ptrdiff_t>(i);
    if (std::llabs(static_cast<long long>(index_gap)) <
        static_cast<long long>(config_.min_index_gap)) {
      continue;
    }
    if (std::fabs(time_s - times_[i]) < config_.min_time_gap_s) {
      continue;
    }
    shortlist.emplace_back(
      (normalized_range_[i] - key).squaredNorm(), i);
  }
  if (shortlist.empty()) {
    return candidates;
  }

  const std::size_t width = std::min<std::size_t>(
    shortlist.size(), static_cast<std::size_t>(config_.num_nearest_matches));
  std::partial_sort(
    shortlist.begin(), shortlist.begin() + width, shortlist.end());

  // Evaluate the full SOLiD distance only on the shortlist.
  std::vector<Candidate> scored;
  scored.reserve(width);
  for (std::size_t i = 0; i < width; ++i) {
    const std::size_t stored = shortlist[i].second;
    const Comparison comparison = compare(descriptor, descriptors_[stored]);
    if (!comparison.valid() ||
        comparison.range_distance > config_.distance_threshold) {
      continue;
    }
    scored.push_back(Candidate{
      stored, comparison.range_distance, comparison.sector_shift,
      comparison.yaw_rad});
  }
  if (scored.empty()) {
    return candidates;
  }

  std::stable_sort(
    scored.begin(), scored.end(),
    [](const Candidate& lhs, const Candidate& rhs) {
      return lhs.range_distance < rhs.range_distance;
    });
  const std::size_t kept = std::min<std::size_t>(
    scored.size(), static_cast<std::size_t>(config_.num_match_candidates));
  candidates.assign(scored.begin(), scored.begin() + kept);
  return candidates;
}

std::vector<Candidate> Index::searchLatest() const {
  if (descriptors_.empty()) {
    return {};
  }
  const std::size_t latest = descriptors_.size() - 1;
  return search(descriptors_[latest], times_[latest], latest);
}

void Index::clear() {
  descriptors_.clear();
  normalized_range_.clear();
  times_.clear();
}

}  // namespace liorf::loop_detection
