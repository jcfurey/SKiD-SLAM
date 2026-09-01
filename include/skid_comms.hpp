#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <list>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// Communication policy for the distributed map-fusion node.
//
// The paper's argument for a lightweight descriptor is a communication
// argument: peers exchange descriptors continuously and move a full scan only
// when a candidate justifies it. That requires bounded queues, a bounded scan
// cache, defined retry and backpressure behaviour, and byte/latency
// accounting.
//
// Everything here is policy: what to keep, what to drop, what to ask for, and
// what to report. The payloads themselves stay in the node that owns them, so
// this header needs no ROS or PCL dependency and the policy is testable on its
// own.
namespace liorf::comms {

struct Config {
  // Announcements held per peer while its link is down.
  std::size_t max_pending_announcements = 100;

  // Retained peer scans, bounded by both entry count and total bytes.
  std::size_t max_cached_scans = 200;
  std::size_t max_cached_scan_bytes = 256u * 1024u * 1024u;

  // Backpressure on outstanding scan requests.
  std::size_t max_inflight_requests = 8;
  double request_timeout_s = 5.0;
  std::size_t max_request_attempts = 3;

  // Loop candidates parked while their scans are in flight.
  std::size_t max_deferred_candidates = 64;
  double max_deferred_age_s = 60.0;
};

// Returns an empty string when every configuration value is valid.
std::string validate(const Config& config);

// Identifies one platform's keyframe scan.
struct ScanKey {
  std::string robot_id;
  std::int64_t keyframe_index = -1;

  bool valid() const noexcept;
  std::string str() const;

  bool operator==(const ScanKey& other) const noexcept {
    return keyframe_index == other.keyframe_index && robot_id == other.robot_id;
  }
  bool operator!=(const ScanKey& other) const noexcept {
    return !(*this == other);
  }
};

struct ScanKeyHash {
  std::size_t operator()(const ScanKey& key) const noexcept;
};

// FIFO queue with a hard capacity.
//
// Pushing into a full queue drops the oldest entry: a peer that has been
// unreachable for a while is better served by recent places than by a backlog
// of stale ones. Drops are counted rather than silently absorbed.
template <typename T>
class BoundedQueue {
 public:
  explicit BoundedQueue(std::size_t capacity = 1) : capacity_(capacity) {}

  bool empty() const noexcept { return entries_.empty(); }
  std::size_t size() const noexcept { return entries_.size(); }
  std::size_t capacity() const noexcept { return capacity_; }
  std::size_t dropped() const noexcept { return dropped_; }

  // Returns true when the push displaced an older entry.
  bool push(T value) {
    if (capacity_ == 0) {
      ++dropped_;
      return true;
    }
    bool displaced = false;
    while (entries_.size() >= capacity_) {
      entries_.pop_front();
      ++dropped_;
      displaced = true;
    }
    entries_.push_back(std::move(value));
    return displaced;
  }

  bool pop(T& out) {
    if (entries_.empty()) {
      return false;
    }
    out = std::move(entries_.front());
    entries_.pop_front();
    return true;
  }

  void clear() { entries_.clear(); }

 private:
  std::size_t capacity_;
  std::size_t dropped_ = 0;
  std::deque<T> entries_;
};

// Least-recently-used retention policy for received scans.
//
// The cache records sizes and use order only. insert() returns the keys the
// caller must now release, so the node stays the single owner of the point
// clouds while the eviction rule remains testable on its own.
class ScanCache {
 public:
  explicit ScanCache(const Config& config);

  // Non-empty when the configuration was rejected; such a cache holds nothing.
  const std::string& error() const noexcept { return error_; }

  bool contains(const ScanKey& key) const;
  std::size_t size() const noexcept { return entries_.size(); }
  std::size_t bytes() const noexcept { return bytes_; }
  std::size_t evicted() const noexcept { return evicted_; }
  std::size_t rejected() const noexcept { return rejected_; }

  // Records that `key` is held and occupies `bytes`. Re-inserting a key
  // replaces its size and marks it most recently used. Returns the keys that
  // must be released to stay inside both bounds, least recently used first.
  //
  // An entry larger than the whole byte budget is rejected rather than
  // emptying the cache for something that cannot fit: it comes back as its
  // own eviction.
  std::vector<ScanKey> insert(const ScanKey& key, std::size_t bytes);

  // Marks an existing entry most recently used.
  bool touch(const ScanKey& key);
  bool erase(const ScanKey& key);
  void clear();

 private:
  struct Entry {
    std::size_t bytes = 0;
    std::list<ScanKey>::iterator order;
  };

  Config config_;
  std::string error_;
  std::size_t bytes_ = 0;
  std::size_t evicted_ = 0;
  std::size_t rejected_ = 0;
  std::list<ScanKey> order_;  // least recently used at the front
  std::unordered_map<ScanKey, Entry, ScanKeyHash> entries_;
};

enum class RequestDecision {
  kSend,            // transmit the request now
  kAlreadyPending,  // an identical request is outstanding
  kThrottled,       // the in-flight cap is reached; ask again later
  kAbandoned,       // this key's attempt budget is spent
  kInvalidKey,
};

const char* toString(RequestDecision decision) noexcept;

// Outstanding scan requests: an in-flight cap, a timeout, a bounded retry
// budget, and round-trip latency.
//
// A key whose budget is spent stays abandoned until forget() is called, so a
// silent peer cannot be asked forever. Abandonment is a reported state, not a
// silent give-up.
class RequestTracker {
 public:
  explicit RequestTracker(const Config& config);

  // Non-empty when the configuration was rejected; such a tracker sends
  // nothing.
  const std::string& error() const noexcept { return error_; }

  RequestDecision request(const ScanKey& key, double now_s);

  // Round-trip seconds for a completed request, or a negative value when
  // nothing was outstanding for this key.
  double complete(const ScanKey& key, double now_s);

  struct Expired {
    std::vector<ScanKey> resend;     // budget remains; resend these
    std::vector<ScanKey> abandoned;  // budget spent; give up on these
  };

  // Moves timed-out requests out of flight.
  Expired expire(double now_s);

  bool pending(const ScanKey& key) const;
  bool isAbandoned(const ScanKey& key) const;
  // Allows a previously abandoned key to be requested again.
  bool forget(const ScanKey& key);

  std::size_t inflight() const noexcept { return pending_.size(); }
  std::size_t sent() const noexcept { return sent_; }
  std::size_t retried() const noexcept { return retried_; }
  std::size_t abandoned() const noexcept { return abandoned_.size(); }
  std::size_t throttled() const noexcept { return throttled_; }
  void clear();

 private:
  struct Pending {
    double sent_at_s = 0.0;
    std::size_t attempts = 0;
  };

  Config config_;
  std::string error_;
  std::size_t sent_ = 0;
  std::size_t retried_ = 0;
  std::size_t throttled_ = 0;
  std::unordered_map<ScanKey, Pending, ScanKeyHash> pending_;
  std::unordered_set<ScanKey, ScanKeyHash> abandoned_;
};

// One inter-robot loop candidate parked until the scans it needs arrive.
struct DeferredCandidate {
  int query_bin = -1;
  int candidate_bin = -1;
  int sector_shift = 0;
  double parked_at_s = 0.0;
  std::vector<ScanKey> missing;
};

// Bounded store of parked candidates.
//
// When full, the oldest is dropped: a candidate whose scan never arrived is
// not worth more memory than the freshest one that might still complete.
class DeferredCandidateQueue {
 public:
  explicit DeferredCandidateQueue(const Config& config);

  const std::string& error() const noexcept { return error_; }

  // Parks a candidate that is still waiting on at least one scan. Returns
  // false when the candidate was rejected outright.
  bool park(DeferredCandidate candidate);

  // Records that `key` arrived and returns every parked candidate with
  // nothing left to wait for, oldest first.
  std::vector<DeferredCandidate> release(const ScanKey& key);

  // Drops candidates parked longer than the configured age. Returns how many.
  std::size_t expire(double now_s);

  std::size_t size() const noexcept { return parked_.size(); }
  std::size_t dropped() const noexcept { return dropped_; }
  std::size_t expired() const noexcept { return expired_; }
  void clear();

 private:
  Config config_;
  std::string error_;
  std::size_t dropped_ = 0;
  std::size_t expired_ = 0;
  std::deque<DeferredCandidate> parked_;
};

// Byte and latency accounting for one channel.
class TransferStats {
 public:
  void recordSent(std::size_t bytes) noexcept;
  void recordReceived(std::size_t bytes) noexcept;
  // Non-finite or negative samples are counted as rejected, not averaged in.
  void recordLatency(double seconds) noexcept;

  std::uint64_t messagesSent() const noexcept { return messages_sent_; }
  std::uint64_t messagesReceived() const noexcept { return messages_received_; }
  std::uint64_t bytesSent() const noexcept { return bytes_sent_; }
  std::uint64_t bytesReceived() const noexcept { return bytes_received_; }
  std::uint64_t latencySamples() const noexcept { return latency_samples_; }
  std::uint64_t rejectedLatencySamples() const noexcept {
    return rejected_latency_samples_;
  }

  // Both are NaN until a sample is recorded.
  double meanLatencySeconds() const noexcept;
  double maxLatencySeconds() const noexcept;

  std::string summary() const;
  void reset() noexcept;

 private:
  std::uint64_t messages_sent_ = 0;
  std::uint64_t messages_received_ = 0;
  std::uint64_t bytes_sent_ = 0;
  std::uint64_t bytes_received_ = 0;
  std::uint64_t latency_samples_ = 0;
  std::uint64_t rejected_latency_samples_ = 0;
  double latency_sum_s_ = 0.0;
  double latency_max_s_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace liorf::comms
