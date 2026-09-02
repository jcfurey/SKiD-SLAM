#include "skid_comms.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <sstream>

namespace liorf::comms {
namespace {

void appendError(std::ostringstream& errors, const char* message) {
  errors << (errors.tellp() == std::streampos(0) ? "" : "; ") << message;
}

}  // namespace

std::string validate(const Config& config) {
  std::ostringstream errors;
  const auto require = [&errors](bool condition, const char* message) {
    if (!condition) {
      appendError(errors, message);
    }
  };

  require(config.max_pending_announcements > 0,
          "max_pending_announcements must be positive");
  require(config.max_cached_scans > 0, "max_cached_scans must be positive");
  require(config.max_cached_scan_bytes > 0,
          "max_cached_scan_bytes must be positive");
  require(config.max_inflight_requests > 0,
          "max_inflight_requests must be positive");
  require(std::isfinite(config.request_timeout_s) &&
            config.request_timeout_s > 0.0,
          "request_timeout_s must be finite and positive");
  require(config.max_request_attempts > 0,
          "max_request_attempts must be positive");
  require(config.max_deferred_candidates > 0,
          "max_deferred_candidates must be positive");
  require(std::isfinite(config.max_deferred_age_s) &&
            config.max_deferred_age_s > 0.0,
          "max_deferred_age_s must be finite and positive");

  return errors.str();
}

bool ScanKey::valid() const noexcept {
  return !robot_id.empty() && keyframe_index >= 0;
}

std::string ScanKey::str() const {
  std::ostringstream text;
  text << robot_id << '#' << keyframe_index;
  return text.str();
}

std::size_t ScanKeyHash::operator()(const ScanKey& key) const noexcept {
  const std::size_t robot = std::hash<std::string>()(key.robot_id);
  const std::size_t index =
    std::hash<std::int64_t>()(key.keyframe_index);
  return robot ^ (index + 0x9e3779b97f4a7c15ULL + (robot << 6) + (robot >> 2));
}

const char* toString(RequestDecision decision) noexcept {
  switch (decision) {
    case RequestDecision::kSend: return "send";
    case RequestDecision::kAlreadyPending: return "already_pending";
    case RequestDecision::kThrottled: return "throttled";
    case RequestDecision::kAbandoned: return "abandoned";
    case RequestDecision::kInvalidKey: return "invalid_key";
  }
  return "unknown";
}

// --------------------------------------------------------------------------
// ScanCache
// --------------------------------------------------------------------------

ScanCache::ScanCache(const Config& config)
  : config_(config), error_(validate(config)) {}

bool ScanCache::contains(const ScanKey& key) const {
  return entries_.find(key) != entries_.end();
}

std::vector<ScanKey> ScanCache::insert(const ScanKey& key, std::size_t bytes) {
  std::vector<ScanKey> evicted;
  if (!error_.empty() || !key.valid()) {
    ++rejected_;
    evicted.push_back(key);
    return evicted;
  }

  // Replacing an existing entry frees its old size first, so a re-insert
  // never counts the same scan twice.
  erase(key);

  if (bytes > config_.max_cached_scan_bytes) {
    ++rejected_;
    evicted.push_back(key);
    return evicted;
  }

  order_.push_back(key);
  Entry entry;
  entry.bytes = bytes;
  entry.order = std::prev(order_.end());
  entries_.emplace(key, entry);
  bytes_ += bytes;

  while (!order_.empty() &&
         (entries_.size() > config_.max_cached_scans ||
          bytes_ > config_.max_cached_scan_bytes)) {
    const ScanKey oldest = order_.front();
    if (oldest == key) {
      // The entry just inserted is the only one left and still does not fit.
      break;
    }
    erase(oldest);
    ++evicted_;
    evicted.push_back(oldest);
  }
  return evicted;
}

bool ScanCache::touch(const ScanKey& key) {
  const auto it = entries_.find(key);
  if (it == entries_.end()) {
    return false;
  }
  order_.splice(order_.end(), order_, it->second.order);
  it->second.order = std::prev(order_.end());
  return true;
}

bool ScanCache::erase(const ScanKey& key) {
  const auto it = entries_.find(key);
  if (it == entries_.end()) {
    return false;
  }
  bytes_ -= it->second.bytes;
  order_.erase(it->second.order);
  entries_.erase(it);
  return true;
}

void ScanCache::clear() {
  entries_.clear();
  order_.clear();
  bytes_ = 0;
}

// --------------------------------------------------------------------------
// RequestTracker
// --------------------------------------------------------------------------

RequestTracker::RequestTracker(const Config& config)
  : config_(config), error_(validate(config)) {}

RequestDecision RequestTracker::request(const ScanKey& key, double now_s) {
  if (!error_.empty()) {
    return RequestDecision::kInvalidKey;
  }
  if (!key.valid() || !std::isfinite(now_s)) {
    return RequestDecision::kInvalidKey;
  }
  if (abandoned_.find(key) != abandoned_.end()) {
    return RequestDecision::kAbandoned;
  }

  const auto it = pending_.find(key);
  if (it != pending_.end()) {
    return RequestDecision::kAlreadyPending;
  }
  if (pending_.size() >= config_.max_inflight_requests) {
    ++throttled_;
    return RequestDecision::kThrottled;
  }

  Pending entry;
  entry.sent_at_s = now_s;
  entry.attempts = 1;
  pending_.emplace(key, entry);
  ++sent_;
  return RequestDecision::kSend;
}

double RequestTracker::complete(const ScanKey& key, double now_s) {
  // A response proves the peer is answering, so clear any abandonment first.
  // This runs even for a late reply that arrived after the attempt budget was
  // spent, which is exactly the case that must not stay blocked.
  abandoned_.erase(key);

  const auto it = pending_.find(key);
  if (it == pending_.end()) {
    return -1.0;
  }
  const double latency = now_s - it->second.sent_at_s;
  pending_.erase(it);
  return latency;
}

RequestTracker::Expired RequestTracker::expire(double now_s) {
  Expired expired;
  if (!std::isfinite(now_s)) {
    return expired;
  }

  for (auto it = pending_.begin(); it != pending_.end();) {
    if (now_s - it->second.sent_at_s < config_.request_timeout_s) {
      ++it;
      continue;
    }
    const ScanKey key = it->first;
    if (it->second.attempts >= config_.max_request_attempts) {
      pending_.erase(it++);
      abandoned_.insert(key);
      expired.abandoned.push_back(key);
      continue;
    }
    it->second.sent_at_s = now_s;
    ++it->second.attempts;
    ++retried_;
    ++sent_;
    expired.resend.push_back(key);
    ++it;
  }

  // Deterministic order keeps logs and tests reproducible across hash layouts.
  const auto by_key = [](const ScanKey& lhs, const ScanKey& rhs) {
    return lhs.robot_id != rhs.robot_id
             ? lhs.robot_id < rhs.robot_id
             : lhs.keyframe_index < rhs.keyframe_index;
  };
  std::sort(expired.resend.begin(), expired.resend.end(), by_key);
  std::sort(expired.abandoned.begin(), expired.abandoned.end(), by_key);
  return expired;
}

bool RequestTracker::pending(const ScanKey& key) const {
  return pending_.find(key) != pending_.end();
}

bool RequestTracker::isAbandoned(const ScanKey& key) const {
  return abandoned_.find(key) != abandoned_.end();
}

bool RequestTracker::forget(const ScanKey& key) {
  return abandoned_.erase(key) > 0;
}

void RequestTracker::clear() {
  pending_.clear();
  abandoned_.clear();
}

// --------------------------------------------------------------------------
// DeferredCandidateQueue
// --------------------------------------------------------------------------

DeferredCandidateQueue::DeferredCandidateQueue(const Config& config)
  : config_(config), error_(validate(config)) {}

bool DeferredCandidateQueue::park(DeferredCandidate candidate) {
  if (!error_.empty() || candidate.missing.empty() ||
      !std::isfinite(candidate.parked_at_s)) {
    return false;
  }
  for (const auto& key : candidate.missing) {
    if (!key.valid()) {
      return false;
    }
  }

  while (parked_.size() >= config_.max_deferred_candidates) {
    parked_.pop_front();
    ++dropped_;
  }
  parked_.push_back(std::move(candidate));
  return true;
}

std::vector<DeferredCandidate> DeferredCandidateQueue::release(
  const ScanKey& key) {
  std::vector<DeferredCandidate> ready;
  for (auto it = parked_.begin(); it != parked_.end();) {
    auto& missing = it->missing;
    missing.erase(
      std::remove(missing.begin(), missing.end(), key), missing.end());
    if (missing.empty()) {
      ready.push_back(std::move(*it));
      it = parked_.erase(it);
    } else {
      ++it;
    }
  }
  return ready;
}

std::size_t DeferredCandidateQueue::expire(double now_s) {
  return expireCandidates(now_s).size();
}

std::vector<DeferredCandidate> DeferredCandidateQueue::expireCandidates(
  double now_s) {
  std::vector<DeferredCandidate> removed;
  if (!std::isfinite(now_s)) {
    return removed;
  }
  for (auto it = parked_.begin(); it != parked_.end();) {
    if (now_s - it->parked_at_s > config_.max_deferred_age_s) {
      removed.push_back(std::move(*it));
      it = parked_.erase(it);
      ++expired_;
    } else {
      ++it;
    }
  }
  return removed;
}

void DeferredCandidateQueue::clear() {
  parked_.clear();
}

// --------------------------------------------------------------------------
// TransferStats
// --------------------------------------------------------------------------

void TransferStats::recordSent(std::size_t bytes) noexcept {
  ++messages_sent_;
  bytes_sent_ += bytes;
}

void TransferStats::recordReceived(std::size_t bytes) noexcept {
  ++messages_received_;
  bytes_received_ += bytes;
}

void TransferStats::recordLatency(double seconds) noexcept {
  if (!std::isfinite(seconds) || seconds < 0.0) {
    ++rejected_latency_samples_;
    return;
  }
  ++latency_samples_;
  latency_sum_s_ += seconds;
  if (!(seconds <= latency_max_s_)) {
    latency_max_s_ = seconds;
  }
}

double TransferStats::meanLatencySeconds() const noexcept {
  if (latency_samples_ == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return latency_sum_s_ / static_cast<double>(latency_samples_);
}

double TransferStats::maxLatencySeconds() const noexcept {
  return latency_max_s_;
}

std::string TransferStats::summary() const {
  std::ostringstream text;
  text.setf(std::ios::fixed);
  text.precision(3);
  text << "sent " << messages_sent_ << " msg / "
       << (static_cast<double>(bytes_sent_) / 1024.0) << " kiB, received "
       << messages_received_ << " msg / "
       << (static_cast<double>(bytes_received_) / 1024.0) << " kiB";
  if (latency_samples_ > 0) {
    text << ", latency mean " << (1000.0 * meanLatencySeconds())
         << " ms max " << (1000.0 * maxLatencySeconds()) << " ms over "
         << latency_samples_ << " samples";
  } else {
    text << ", no latency samples";
  }
  return text.str();
}

void TransferStats::reset() noexcept {
  messages_sent_ = 0;
  messages_received_ = 0;
  bytes_sent_ = 0;
  bytes_received_ = 0;
  latency_samples_ = 0;
  rejected_latency_samples_ = 0;
  latency_sum_s_ = 0.0;
  latency_max_s_ = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace liorf::comms
